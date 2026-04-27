/**
 * Nucleo F429ZI — CAN-Serial Bridge v3 (with loopback self-test)
 * CAN1 on PB8 (RX) / PB9 (TX), USART3 on PD8/PD9 at 921600
 */
#include "stm32f4xx_hal.h"
#include <string.h>

static CAN_HandleTypeDef  hcan1;
static UART_HandleTypeDef huart3;

#define SERIAL_BUF_SIZE  512
static volatile uint8_t  serial_buf[SERIAL_BUF_SIZE];
static volatile uint16_t serial_head = 0;
static volatile uint16_t serial_tail = 0;
static uint8_t           rx_byte;

#define LED1_PIN   GPIO_PIN_0
#define LED2_PIN   GPIO_PIN_7
#define LED3_PIN   GPIO_PIN_14
#define LED_PORT   GPIOB

static void SystemClock_Config(void);
static void GPIO_Init(void);
static void UART3_Init(void);
static int  CAN1_Init(uint32_t mode);
static void UART_Send(const uint8_t *d, uint16_t len);
static void TX(const char *s);
static void ProcessSerialData(void);
static void SendCanFrame(uint32_t ext_id, uint8_t *data, uint8_t dlc);
static void ForwardCanToSerial(void);

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

/* ═══════════════════════════════════════════════════════ */
static int loopback_test(void)
{
    /* Stop CAN if running, re-init in loopback mode */
    HAL_CAN_DeInit(&hcan1);
    if (!CAN1_Init(CAN_MODE_LOOPBACK)) {
        TX("LOOPBACK: init fail\r\n");
        return 0;
    }

    /* Send a test frame */
    CAN_TxHeaderTypeDef txH;
    uint32_t mb;
    uint8_t td[8] = {0xDE,0xAD,0xBE,0xEF,0x01,0x02,0x03,0x04};
    txH.ExtId = 0x12345; txH.IDE = CAN_ID_EXT;
    txH.RTR = CAN_RTR_DATA; txH.DLC = 8;
    txH.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan1, &txH, td, &mb) != HAL_OK) {
        TX("LOOPBACK: tx fail\r\n");
        return 0;
    }

    /* Wait for TX complete */
    uint32_t t0 = HAL_GetTick();
    while (HAL_CAN_IsTxMessagePending(&hcan1, mb)) {
        if (HAL_GetTick() - t0 > 100) {
            TX("LOOPBACK: tx timeout\r\n");
            return 0;
        }
    }

    /* Wait for RX */
    t0 = HAL_GetTick();
    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) == 0) {
        if (HAL_GetTick() - t0 > 100) {
            TX("LOOPBACK: rx timeout\r\n");
            return 0;
        }
    }

    CAN_RxHeaderTypeDef rxH;
    uint8_t rd[8];
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &rxH, rd);

    if (rxH.ExtId == 0x12345 && rd[0]==0xDE && rd[3]==0xEF) {
        TX("LOOPBACK: PASS\r\n");
        return 1;
    }
    TX("LOOPBACK: data mismatch\r\n");
    return 0;
}

/* ═══════════════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    UART3_Init();

    TX("\r\n=== Nucleo CAN Bridge v3 ===\r\n");
    TX("Pins: PD0=CAN_RX  PD1=CAN_TX\r\n");

    /* Self-test: CAN loopback */
    TX("--- Loopback self-test ---\r\n");
    int lb_ok = loopback_test();

    /* Switch to normal mode */
    HAL_CAN_DeInit(&hcan1);
    TX("--- Switching to NORMAL mode ---\r\n");
    if (!CAN1_Init(CAN_MODE_NORMAL)) {
        TX("CAN NORMAL init FAILED\r\n");
        HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
    } else {
        TX("CAN NORMAL mode OK\r\n");
    }

    uint32_t esr = hcan1.Instance->ESR;
    TX("ESR="); TX_Hex32(esr); TX("\r\n");

    if (lb_ok)
        TX("CAN peripheral OK. If no motor response, check CAN wiring.\r\n");
    else
        TX("CAN peripheral FAILED loopback!\r\n");

    TX("CAN Bridge Ready\r\n");

    HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

    uint32_t last_hb = 0, last_err = 0;

    while (1) {
        ProcessSerialData();
        ForwardCanToSerial();

        uint32_t now = HAL_GetTick();
        if (now - last_hb >= 500) {
            HAL_GPIO_TogglePin(LED_PORT, LED1_PIN);
            last_hb = now;
        }
        if (now - last_err >= 5000) {
            esr = hcan1.Instance->ESR;
            uint8_t tec = (esr>>16)&0xFF, rec = (esr>>24)&0xFF;
            if (tec || rec) {
                TX("ERR TEC="); TX_Dec(tec);
                TX(" REC="); TX_Dec(rec);
                TX(" ESR="); TX_Hex32(esr); TX("\r\n");
                HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(LED_PORT, LED3_PIN, GPIO_PIN_RESET);
            }
            last_err = now;
        }
    }
}

/* ═══════════════════════════════════════════════════════ */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_BYPASS;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLM = 8; osc.PLL.PLLN = 360;
    osc.PLL.PLLP = RCC_PLLP_DIV2; osc.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
        memset(&osc, 0, sizeof(osc));
        osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
        osc.HSIState = RCC_HSI_ON;
        osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
        osc.PLL.PLLState = RCC_PLL_ON;
        osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
        osc.PLL.PLLM = 16; osc.PLL.PLLN = 336;
        osc.PLL.PLLP = RCC_PLLP_DIV2; osc.PLL.PLLQ = 7;
        HAL_RCC_OscConfig(&osc);
    }
    HAL_PWREx_EnableOverDrive();

    clk.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_HCLK|
                    RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV4;
    clk.APB2CLKDivider = RCC_HCLK_DIV2;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_5);
}

static void GPIO_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = LED1_PIN|LED2_PIN|LED3_PIN;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
    HAL_GPIO_WritePin(LED_PORT, LED1_PIN|LED2_PIN|LED3_PIN, GPIO_PIN_RESET);
    for (int i=0;i<6;i++){HAL_GPIO_TogglePin(LED_PORT,LED1_PIN|LED2_PIN|LED3_PIN);HAL_Delay(80);}
    HAL_GPIO_WritePin(LED_PORT, LED1_PIN|LED2_PIN|LED3_PIN, GPIO_PIN_RESET);
}

static void UART3_Init(void)
{
    GPIO_InitTypeDef g = {0};
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    g.Pin = GPIO_PIN_8|GPIO_PIN_9;
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF7_USART3;
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

/* Returns 1 on success */
static int CAN1_Init(uint32_t mode)
{
    GPIO_InitTypeDef g = {0};
    CAN_FilterTypeDef f;

    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /* PD0=CAN1_RX, PD1=CAN1_TX  AF9 */
    g.Pin = GPIO_PIN_0|GPIO_PIN_1;
    g.Mode = GPIO_MODE_AF_PP; g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH; g.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &g);

    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = mode;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = ENABLE;
    hcan1.Init.AutoWakeUp = ENABLE;
    hcan1.Init.AutoRetransmission = DISABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;

    if (HAL_CAN_Init(&hcan1) != HAL_OK) return 0;

    memset(&f, 0, sizeof(f));
    f.FilterBank = 0; f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation = ENABLE;
    HAL_CAN_ConfigFilter(&hcan1, &f);

    if (HAL_CAN_Start(&hcan1) != HAL_OK) return 0;
    return 1;
}

/* ═══════════════════════════════════════════════════════ */
static void ProcessSerialData(void)
{
    while (serial_available() >= 7) {
        if (serial_peek(0)!=0x41 || serial_peek(1)!=0x54) {
            serial_discard(1); continue;
        }
        if (serial_peek(2)==0x2B) {
            if (serial_available()<7) return;
            if (serial_peek(3)==0x41&&serial_peek(4)==0x54&&
                serial_peek(5)==0x0D&&serial_peek(6)==0x0A) {
                serial_discard(7); TX("OK\r\n"); continue;
            }
            serial_discard(1); continue;
        }
        if (serial_available()<7) return;
        uint8_t dlc = serial_peek(6);
        if (dlc>8) { serial_discard(2); continue; }
        uint16_t flen = 2+4+1+dlc+2;
        if (serial_available()<flen) return;

        uint32_t reg32 = ((uint32_t)serial_peek(2)<<24)|
                         ((uint32_t)serial_peek(3)<<16)|
                         ((uint32_t)serial_peek(4)<<8)|
                         ((uint32_t)serial_peek(5));
        uint8_t data[8];
        for (uint8_t i=0;i<dlc;i++) data[i]=serial_peek(7+i);
        serial_discard(flen);

        uint32_t ext_id = reg32 >> 3;
        HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_SET);
        SendCanFrame(ext_id, data, dlc);
        HAL_GPIO_WritePin(LED_PORT, LED2_PIN, GPIO_PIN_RESET);
    }
}

static void SendCanFrame(uint32_t ext_id, uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef h;
    uint32_t mb;
    h.ExtId=ext_id; h.IDE=CAN_ID_EXT; h.RTR=CAN_RTR_DATA;
    h.DLC=dlc; h.TransmitGlobalTime=DISABLE;

    uint32_t t0=HAL_GetTick();
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1)==0)
        if(HAL_GetTick()-t0>50){TX("TX:mbox timeout\r\n");return;}

    if(HAL_CAN_AddTxMessage(&hcan1,&h,data,&mb)!=HAL_OK){
        TX("TX:add fail\r\n");return;
    }
    t0=HAL_GetTick();
    while(HAL_CAN_IsTxMessagePending(&hcan1,mb))
        if(HAL_GetTick()-t0>50){TX("TX:no ACK\r\n");return;}
}

static void ForwardCanToSerial(void)
{
    CAN_RxHeaderTypeDef rxH; uint8_t rd[8];
    while(HAL_CAN_GetRxFifoFillLevel(&hcan1,CAN_RX_FIFO0)>0){
        if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&rxH,rd)!=HAL_OK) continue;
        uint32_t eid=(rxH.IDE==CAN_ID_EXT)?rxH.ExtId:rxH.StdId;
        uint32_t r32=(eid<<3)|0x04;
        uint8_t d=rxH.DLC; if(d>8)d=8;
        uint8_t f[17]; uint8_t l=0;
        f[l++]=0x41;f[l++]=0x54;
        f[l++]=(r32>>24)&0xFF;f[l++]=(r32>>16)&0xFF;
        f[l++]=(r32>>8)&0xFF;f[l++]=r32&0xFF;
        f[l++]=d;
        for(uint8_t i=0;i<d;i++)f[l++]=rd[i];
        f[l++]=0x0D;f[l++]=0x0A;
        UART_Send(f,l);
    }
}

/* ═══════════════════════════════════════════════════════ */
static void UART_Send(const uint8_t *d, uint16_t len) {
    HAL_UART_Transmit(&huart3,(uint8_t*)d,len,50);
}
static void TX(const char *s) { UART_Send((const uint8_t*)s,strlen(s)); }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *h) {
    if(h->Instance==USART3){
        uint16_t n=(serial_head+1)%SERIAL_BUF_SIZE;
        if(n!=serial_tail){serial_buf[serial_head]=rx_byte;serial_head=n;}
        HAL_UART_Receive_IT(&huart3,&rx_byte,1);
    }
}
void USART3_IRQHandler(void){HAL_UART_IRQHandler(&huart3);}
void SysTick_Handler(void){HAL_IncTick();}
