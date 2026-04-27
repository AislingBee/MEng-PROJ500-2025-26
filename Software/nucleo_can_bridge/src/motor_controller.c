/**
 * Motor Controller Firmware for RobStride Motors
 * ================================================
 * 
 * Receives control commands via CAN and manages:
 *  - Motor enable/disable
 *  - Zero position calibration
 *  - Jogging (velocity control)
 *  - Point-to-point motion (position control)
 *  - Telemetry feedback (position, velocity, torque, temperature)
 * 
 * Command Types:
 *   COMM_ENABLE (3)          : Enable motor
 *   COMM_DISABLE (4)         : Disable motor
 *   COMM_SET_ZERO (6)        : Zero the position
 *   Parameter writes         : Set motor parameters
 * 
 * Parameter IDs:
 *   0x7005  = MODE           (0=disable, 1=pos control, 7=velocity control)
 *   0x7016  = POSITION_TARGET (float radians)
 *   0x7024  = PP_SPEED_LIMIT  (float rad/s)
 *   0x7025  = PP_ACCEL        (float rad/s²)
 */

#include "stm32f4xx_hal.h"
#include <string.h>
#include <math.h>
#include "motor_types.h"
#include "motor_controller.h"

/* ═══════════════════════════════════════════════════════════════════ */
/* SYSTEM CONSTANTS */
/* ═══════════════════════════════════════════════════════════════════ */

#define TELEMETRY_PERIOD_MS       50

/* ═══════════════════════════════════════════════════════════════════ */
/* RUNTIME MOTOR STATE ARRAY */
/* ═══════════════════════════════════════════════════════════════════ */

static MotorState_t motors[NUM_MOTOR_CONFIGS];

/* ═══════════════════════════════════════════════════════════════════ */
/* HAL HANDLES */
/* ═══════════════════════════════════════════════════════════════════ */

static CAN_HandleTypeDef   hcan1;
static UART_HandleTypeDef  huart3;
static TIM_HandleTypeDef   htim2;   /* 1ms tick for control loop */

#define SERIAL_BUF_SIZE    512
static volatile uint8_t    serial_buf[SERIAL_BUF_SIZE];
static volatile uint16_t   serial_head = 0;
static volatile uint16_t   serial_tail = 0;
static uint8_t             rx_byte;

#define LED1_PIN   GPIO_PIN_0
#define LED2_PIN   GPIO_PIN_7
#define LED3_PIN   GPIO_PIN_14
#define LED_PORT   GPIOB

/* ═══════════════════════════════════════════════════════════════════ */
/* FUNCTION DECLARATIONS */
/* ═══════════════════════════════════════════════════════════════════ */

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART3_Init(void);
static void CAN1_Init(void);
static void TIM2_Init(void);
static void UART_Send(const uint8_t *data, uint16_t len);
static void TX(const char *s);
static void TX_Hex32(uint32_t v);
static void TX_Dec(uint32_t v);

void MotorController_Init(void);
MotorState_t* MotorController_GetMotor(uint8_t motor_id);
void MotorController_ProcessCommand(uint32_t ext_id, uint8_t *data, uint8_t dlc);
void MotorController_ControlLoop(void);
static void MotorController_SendTelemetry(MotorState_t *motor);

static void SendCanFrame(uint32_t ext_id, uint8_t *data, uint8_t dlc);
static void ProcessSerialData(void);
static void ForwardCanToSerial(void);

static inline uint16_t serial_available(void);
static inline uint8_t serial_peek(uint16_t off);
static inline void serial_discard(uint16_t n);

/* ═══════════════════════════════════════════════════════════════════ */
/* INLINE UTILITIES */
/* ═══════════════════════════════════════════════════════════════════ */

static inline uint16_t serial_available(void) {
    uint16_t h = serial_head, t = serial_tail;
    return (h >= t) ? (h - t) : (SERIAL_BUF_SIZE - t + h);
}

static inline uint8_t serial_peek(uint16_t off) {
    return serial_buf[(serial_tail + off) % SERIAL_BUF_SIZE];
}

static inline void serial_discard(uint16_t n) {
    serial_tail = (serial_tail + n) % SERIAL_BUF_SIZE;
}

static const char hx[] = "0123456789ABCDEF";

static void TX_Hex32(uint32_t v) {
    char b[11] = {'0','x',0,0,0,0,0,0,0,0,0};
    for (int i=0;i<8;i++) b[2+i] = hx[(v>>((7-i)*4))&0xF];
    TX(b);
}

static void TX_Dec(uint32_t v) {
    char b[11]; int i=10; b[i]=0;
    if(v==0){TX("0");return;}
    while(v>0){b[--i]='0'+(v%10);v/=10;}
    TX(&b[i]);
}

/* ═══════════════════════════════════════════════════════════════════ */
/* MOTOR CONTROLLER IMPLEMENTATION */
/* ═══════════════════════════════════════════════════════════════════ */

void MotorController_Init(void) {
    /* Initialize motors from configuration array */
    for (uint32_t i = 0; i < NUM_MOTOR_CONFIGS; i++) {
        const MotorConfig_t *cfg = &MOTOR_CONFIGS[i];
        MotorState_t *m = &motors[i];
        
        m->config = cfg;
        m->motor_id = cfg->motor_id;
        m->mode = MODE_DISABLED;
        m->enabled = cfg->enabled_on_startup;
        
        /* Initialize from config */
        m->position = cfg->initial_position;
        m->velocity = cfg->initial_velocity;
        m->position_target = cfg->initial_position;
        m->velocity_target = 0.0f;
        m->pp_speed_limit = cfg->pp_speed_limit;
        m->pp_accel = cfg->pp_accel;
        m->position_range = cfg->position_range;
        m->velocity_range = cfg->velocity_range;
        m->torque_range = cfg->torque_range;
        
        /* Dynamic control gains (can be tuned via ROS) */
        m->kp = cfg->pp_speed_limit * 2.0f;  /* Default: proportional to speed limit */
        m->kd = 0.1f;                        /* Default: light damping */
        m->feedforward_torque = 0.0f;        /* Default: no feedforward */
        
        /* Runtime state */
        m->torque = 0.0f;
        m->temperature = 25.0f;
        m->jog_direction = 0;
    }
}

MotorState_t* MotorController_GetMotor(uint8_t motor_id) {
    for (uint32_t i = 0; i < NUM_MOTOR_CONFIGS; i++) {
        if (motors[i].motor_id == motor_id) {
            return &motors[i];
        }
    }
    return NULL;
}

/**
 * Process incoming CAN commands
 */
void MotorController_ProcessCommand(uint32_t ext_id, uint8_t *data, uint8_t dlc) {
    uint8_t comm_type = (ext_id >> 24) & 0x1F;
    uint8_t sender_id = (ext_id >> 8) & 0xFF;
    uint8_t motor_id = ext_id & 0xFF;

    MotorState_t *motor = MotorController_GetMotor(motor_id);
    if (!motor) {
        TX("CMD: unknown motor ");
        TX_Dec(motor_id);
        TX("\r\n");
        return;
    }

    if (comm_type == COMM_ENABLE) {
        TX("CMD: ENABLE motor ");
        TX_Dec(motor_id);
        TX("\r\n");
        motor->enabled = 1;
        motor->mode = MODE_POSITION_CONTROL;
        motor->velocity_target = 0.0f;
        motor->jog_direction = 0;
    }
    else if (comm_type == COMM_DISABLE) {
        TX("CMD: DISABLE motor ");
        TX_Dec(motor_id);
        TX("\r\n");
        motor->enabled = 0;
        motor->mode = MODE_DISABLED;
        motor->velocity_target = 0.0f;
        motor->jog_direction = 0;
    }
    else if (comm_type == COMM_SET_ZERO) {
        TX("CMD: ZERO motor ");
        TX_Dec(motor_id);
        TX("\r\n");
        motor->position = 0.0f;
        motor->position_target = 0.0f;
    }
    else if ((comm_type == 0x10 || comm_type == 0x00) && dlc >= 6) {
        /* Parameter write: bytes 0-1 = param_id (little-endian), bytes 2-5 = value */
        uint16_t param_id = ((uint16_t)data[1] << 8) | data[0];
        uint32_t raw_value = ((uint32_t)data[5] << 24) | ((uint32_t)data[4] << 16) |
                             ((uint32_t)data[3] << 8) | data[2];
        float float_value;
        memcpy(&float_value, &raw_value, sizeof(float));

        switch (param_id) {
        case PARAM_MODE: {
            uint32_t mode_val = raw_value;
            TX("PARAM: MODE=");
            TX_Dec(mode_val);
            TX(" motor ");
            TX_Dec(motor_id);
            TX("\r\n");
            motor->mode = mode_val;
            if (mode_val == MODE_DISABLED) {
                motor->enabled = 0;
                motor->velocity_target = 0.0f;
                motor->jog_direction = 0;
            }
        }
        break;

        case PARAM_POSITION_TARGET:
            TX("PARAM: POS_TARGET=");
            TX_Dec((uint32_t)float_value);
            TX(" motor ");
            TX_Dec(motor_id);
            TX("\r\n");
            motor->position_target = float_value;
            motor->jog_direction = 0;
            break;

        case PARAM_PP_SPEED_LIMIT:
            TX("PARAM: PP_SPEED=");
            TX_Dec((uint32_t)float_value);
            TX(" motor ");
            TX_Dec(motor_id);
            TX("\r\n");
            motor->pp_speed_limit = float_value;
            break;

        case PARAM_PP_ACCEL:
            TX("PARAM: PP_ACCEL=");
            TX_Dec((uint32_t)float_value);
            TX(" motor ");
            TX_Dec(motor_id);
            TX("\r\n");
            motor->pp_accel = float_value;
            break;

        case PARAM_KP:
            TX("PARAM: KP=");
            TX_Dec((uint32_t)float_value);
            TX(" motor ");
            TX_Dec(motor_id);
            TX("\r\n");
            motor->kp = float_value;
            break;

        case PARAM_KD:
            TX("PARAM: KD=");
            TX_Dec((uint32_t)float_value);
            TX(" motor ");
            TX_Dec(motor_id);
            TX("\r\n");
            motor->kd = float_value;
            break;

        case PARAM_FEEDFORWARD_TORQUE:
            TX("PARAM: FEEDFORWARD_TORQUE=");
            TX_Dec((uint32_t)float_value);
            TX(" motor ");
            TX_Dec(motor_id);
            TX("\r\n");
            motor->feedforward_torque = float_value;
            break;

        default:
            TX("PARAM: unknown param ");
            TX_Hex32(param_id);
            TX("\r\n");
            break;
        }
    }
}

/**
 * Main control loop - runs at ~1kHz via timer
 * Updates motor state based on mode and control targets
 */
void MotorController_ControlLoop(void) {
    static uint32_t last_telemetry[NUM_MOTOR_CONFIGS] = {0};
    uint32_t now = HAL_GetTick();

    for (uint32_t i = 0; i < NUM_MOTOR_CONFIGS; i++) {
        MotorState_t *m = &motors[i];

        if (!m->enabled) {
            m->velocity = 0.0f;
            continue;
        }

        if (m->mode == MODE_VELOCITY_JOG) {
            /* Direct velocity control for jogging */
            m->velocity = m->velocity_target;
            m->position += m->velocity * 0.001f;  /* Integrate over 1ms */
        }
        else if (m->mode == MODE_POSITION_CONTROL) {
            /* PD controller for position tracking */
            float error = m->position_target - m->position;
            float dt = 0.001f;  /* 1ms */
            
            /* PD control: cmd_vel = kp * error + kd * velocity_error */
            /* velocity_error is used to compute damping term */
            float velocity_error = -m->velocity;  /* Derivative feedback */
            float cmd_vel = (m->kp * error) + (m->kd * velocity_error);
            
            /* Limit velocity */
            if (cmd_vel > m->pp_speed_limit) cmd_vel = m->pp_speed_limit;
            if (cmd_vel < -m->pp_speed_limit) cmd_vel = -m->pp_speed_limit;
            
            m->velocity = cmd_vel;
            m->position += m->velocity * dt;
            
            /* Update torque estimate: tau = kp * error + feedforward */
            m->torque = (m->kp * error * 0.1f) + m->feedforward_torque;  /* Scale torque estimate */
        }

        /* Clamp position to range */
    if (m->position > m->position_range / 2.0f) m->position = m->position_range / 2.0f;
    if (m->position < -m->position_range / 2.0f) m->position = -m->position_range / 2.0f;
        /* Send telemetry periodically */
        if (now - last_telemetry[i] >= TELEMETRY_PERIOD_MS) {
            MotorController_SendTelemetry(m);
            last_telemetry[i] = now;
        }
    }
}

/**
 * Encode motor state as telemetry frame and send via CAN
 * Format: 4x uint16_t big-endian (pos, vel, torq, temp)
 */
static void MotorController_SendTelemetry(MotorState_t *motor) {
    /* Convert to normalized uint16 (0-65535, centered at 32767) */
    uint16_t pos_u16 = (uint16_t)((motor->position / motor->position_range + 1.0f) * 32767.0f);
    uint16_t vel_u16 = (uint16_t)((motor->velocity / motor->velocity_range + 1.0f) * 32767.0f);
    uint16_t torq_u16 = (uint16_t)((motor->torque / motor->torque_range + 1.0f) * 32767.0f);
    uint16_t temp_u16 = (uint16_t)(motor->temperature / TEMP_SCALE);

    uint8_t data[8];
    data[0] = (pos_u16 >> 8) & 0xFF;
    data[1] = pos_u16 & 0xFF;
    data[2] = (vel_u16 >> 8) & 0xFF;
    data[3] = vel_u16 & 0xFF;
    data[4] = (torq_u16 >> 8) & 0xFF;
    data[5] = torq_u16 & 0xFF;
    data[6] = (temp_u16 >> 8) & 0xFF;
    data[7] = temp_u16 & 0xFF;

    /* Build CAN ID: COMM_OPERATION_STATUS(2) << 24 | HOST_ID << 8 | motor_id */
    uint32_t ext_id = (COMM_OPERATION_STATUS << 24) | (HOST_ID << 8) | motor->motor_id;

    SendCanFrame(ext_id, data, 8);
}

/* ═══════════════════════════════════════════════════════════════════ */
/* CAN / UART COMMUNICATION */
/* ═══════════════════════════════════════════════════════════════════ */

static void ProcessSerialData(void) {
    while (serial_available() >= 7) {
        if (serial_peek(0) != 0x41 || serial_peek(1) != 0x54) {
            serial_discard(1);
            continue;
        }

        if (serial_peek(2) == 0x2B) {
            /* AT+AT command (self-test) */
            if (serial_available() < 7) return;
            if (serial_peek(3) == 0x41 && serial_peek(4) == 0x54 &&
                serial_peek(5) == 0x0D && serial_peek(6) == 0x0A) {
                serial_discard(7);
                TX("OK\r\n");
                continue;
            }
            serial_discard(1);
            continue;
        }

        if (serial_available() < 7) return;
        uint8_t dlc = serial_peek(6);
        if (dlc > 8) {
            serial_discard(2);
            continue;
        }

        uint16_t flen = 2 + 4 + 1 + dlc + 2;
        if (serial_available() < flen) return;

        uint32_t reg32 = ((uint32_t)serial_peek(2) << 24) |
                         ((uint32_t)serial_peek(3) << 16) |
                         ((uint32_t)serial_peek(4) << 8) |
                         ((uint32_t)serial_peek(5));
        uint8_t data[8];
        for (uint8_t i = 0; i < dlc; i++) {
            data[i] = serial_peek(7 + i);
        }
        serial_discard(flen);

        uint32_t ext_id = reg32 >> 3;
        
        HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
        
        /* Process command and send it to motors */
        MotorController_ProcessCommand(ext_id, data, dlc);
        
        /* Also forward to CAN */
        SendCanFrame(ext_id, data, dlc);
        
        HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
    }
}

static void SendCanFrame(uint32_t ext_id, uint8_t *data, uint8_t dlc) {
    CAN_TxHeaderTypeDef h;
    uint32_t mb;
    h.ExtId = ext_id;
    h.IDE = CAN_ID_EXT;
    h.RTR = CAN_RTR_DATA;
    h.DLC = dlc;
    h.TransmitGlobalTime = DISABLE;

    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        if (HAL_GetTick() - t0 > 50) {
            TX("TX:mbox timeout\r\n");
            return;
        }
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &h, data, &mb) != HAL_OK) {
        TX("TX:add fail\r\n");
        return;
    }

    t0 = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan1, mb)) {
        if (HAL_GetTick() - t0 > 50) {
            TX("TX:no ACK\r\n");
            return;
        }
    }
}

static void ForwardCanToSerial(void) {
    CAN_RxHeaderTypeDef rxH;
    uint8_t rd[8];

    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0) {
        if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxH, rd) != HAL_OK) continue;

        uint32_t eid = (rxH.IDE == CAN_ID_EXT) ? rxH.ExtId : rxH.StdId;
        uint32_t r32 = (eid << 3) | 0x04;
        uint8_t d = rxH.DLC;
        if (d > 8) d = 8;

        uint8_t f[17];
        uint8_t l = 0;
        f[l++] = 0x41;
        f[l++] = 0x54;
        f[l++] = (r32 >> 24) & 0xFF;
        f[l++] = (r32 >> 16) & 0xFF;
        f[l++] = (r32 >> 8) & 0xFF;
        f[l++] = r32 & 0xFF;
        f[l++] = d;
        for (uint8_t i = 0; i < d; i++) f[l++] = rd[i];
        f[l++] = 0x0D;
        f[l++] = 0x0A;

        UART_Send(f, l);
    }
}

/* ═══════════════════════════════════════════════════════════════════ */
/* MAIN & HAL SETUP */
/* ═══════════════════════════════════════════════════════════════════ */

int main(void) {
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART3_Init();
    CAN1_Init();
    TIM2_Init();
    MotorController_Init();

    TX("\r\n=== RobStride Motor Controller v1 ===\r\n");
    TX("Nucleo F429ZI\r\n");
    TX("Motors: ");
    for (uint32_t i = 0; i < NUM_MOTOR_CONFIGS; i++) {
        if (i > 0) TX(", ");
        TX_Dec(MOTOR_CONFIGS[i].motor_id);
    }
    TX("\r\n");

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    uint32_t last_hb = 0, last_err = 0;

    while (1) {
        ProcessSerialData();
        ForwardCanToSerial();
        MotorController_ControlLoop();

        uint32_t now = HAL_GetTick();
        if (now - last_hb >= 500) {
            HAL_GPIO_TogglePin(LED_PORT, LED1_PIN);
            last_hb = now;
        }

        if (now - last_err >= 5000) {
            uint32_t esr = hcan1.Instance->ESR;
            uint8_t tec = (esr >> 16) & 0xFF;
            uint8_t rec = (esr >> 24) & 0xFF;
            if (tec || rec) {
                TX("CAN ERR TEC=");
                TX_Dec(tec);
                TX(" REC=");
                TX_Dec(rec);
                TX(" ESR=");
                TX_Hex32(esr);
                TX("\r\n");
                HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_RESET);
            }
            last_err = now;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════ */
/* CLOCK & PERIPHERAL INIT */
/* ═══════════════════════════════════════════════════════════════════ */

static void SystemClock_Config(void) {
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState = RCC_HSE_BYPASS;
    osc.PLL.PLLState = RCC_PLL_ON;
    osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8;
    osc.PLL.PLLN = 360;
    osc.PLL.PLLP = RCC_PLLP_DIV2;
    osc.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        memset(&osc, 0, sizeof(osc));
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLState = RCC_PLL_ON;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM = 16;
        osc.PLL.PLLN = 336;
        osc.PLL.PLLP = RCC_PLLP_DIV2;
        osc.PLL.PLLQ = 7;
        HAL_RCC_OscConfig(&osc);
    }

    HAL_PWREx_EnableOverDrive();

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

static void GPIO_Init(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = LED1_PIN | LED2_PIN | LED3_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
    HAL_GPIO_WritePin(LED_PORT, LED1_PIN | LED2_PIN | LED3_PIN, GPIO_PIN_RESET);
}

static void UART3_Init(void) {
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    g.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &g);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = 921600;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart3);

    HAL_NVIC_SetPriority(USART3_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
}

static void CAN1_Init(void) {
    GPIO_InitTypeDef g = {0};
    CAN_FilterTypeDef f;

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    g.Mode = GPIO_MODE_AF_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &g);

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = ENABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) {
        TX("CAN init failed\r\n");
    }

    memset(&f, 0, sizeof(f));
    f.FilterBank = 0;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &f);

    if (HAL_CAN_Start(&hcan1) != HAL_OK) {
        TX("CAN start failed\r\n");
    }
}

/**
 * Timer 2 for 1kHz control loop tick
 */
static void TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 168 - 1;  /* 168MHz / 168 = 1MHz */
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000 - 1;  /* 1MHz / 1000 = 1kHz */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        TX("TIM2 init failed\r\n");
    }

    HAL_TIM_Base_Start_IT(&htim2);
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ═══════════════════════════════════════════════════════════════════ */
/* UART & IRQ HANDLERS */
/* ═══════════════════════════════════════════════════════════════════ */

static void UART_Send(const uint8_t *d, uint16_t len) {
    HAL_UART_Transmit(&huart3, (uint8_t *)d, len, 50);
}

static void TX(const char *s) {
    UART_Send((const uint8_t *)s, strlen(s));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
    if (h->Instance == USART3) {
        uint16_t n = (serial_head + 1) % SERIAL_BUF_SIZE;
        if (n != serial_tail) {
            serial_buf[serial_head] = rx_byte;
            serial_head = n;
        }
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

void USART3_IRQHandler(void) {
    HAL_UART_IRQHandler(&huart3);
}

void TIM2_IRQHandler(void) {
    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        MotorController_ControlLoop();
    }
}

void SysTick_Handler(void) {
    HAL_IncTick();
}
