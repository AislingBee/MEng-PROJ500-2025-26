#!/usr/bin/env python3
# Publishes: /robot_command (RobotCommand) - 1 joint, steps between +target_q and -target_q
# Subscribes: /motor_feedback (MotorFeedback) - logs position and velocity
#
# Logs are written to <Software>/logs/<timestamp>/
#   commands.csv  - time_s, q_des_rad, kp, kd, tau_ff_Nm
#   feedback.csv  - time_s, joint_name, q_rad, q_dot_rad_s

import csv
import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from motor_test.msg import RobotCommand, MotorFeedback
from motor_test.common import get_software_log_dir


class SingleMotorTest(Node):
    def __init__(self):
        super().__init__('single_motor_test')

        self.declare_parameter('joint_name',    'motor_1')
        self.declare_parameter('target_q',      0.4)       # rad
        self.declare_parameter('kp',            20.0)      # Nm/rad
        self.declare_parameter('kd',            1.0)       # Nm.s/rad
        self.declare_parameter('tau_ff',        0.0)       # Nm feedforward
        self.declare_parameter('step_duration', 4.0)       # seconds per step
        self.declare_parameter('rate_hz',       50.0)
        self.declare_parameter('command_topic', 'robot_command')
        self.declare_parameter('feedback_topic', 'motor_feedback')
        self.declare_parameter('log_dir', str(get_software_log_dir()))

        self.joint_name     = self.get_parameter('joint_name').value
        self.target_q       = float(self.get_parameter('target_q').value)
        self.kp             = float(self.get_parameter('kp').value)
        self.kd             = float(self.get_parameter('kd').value)
        self.tau_ff         = float(self.get_parameter('tau_ff').value)
        self.step_duration  = float(self.get_parameter('step_duration').value)
        self.rate_hz        = float(self.get_parameter('rate_hz').value)
        self.command_topic  = self.get_parameter('command_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        log_base            = self.get_parameter('log_dir').value

        # -- Log file setup --------------------------------------------------
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = Path(log_base) / timestamp
        log_dir.mkdir(parents=True, exist_ok=True)

        self._cmd_csv = open(log_dir / 'commands.csv',  'w', newline='', encoding='utf-8')
        self._fbk_csv = open(log_dir / 'feedback.csv',  'w', newline='', encoding='utf-8')
        self._log     = open(log_dir / 'motor_test.log', 'w', encoding='utf-8')
        self._cmd_writer = csv.writer(self._cmd_csv)
        self._fbk_writer = csv.writer(self._fbk_csv)
        self._cmd_writer.writerow(['time_s', 'q_des_rad', 'kp', 'kd', 'tau_ff_Nm'])
        self._fbk_writer.writerow(['time_s', 'joint_name', 'q_rad', 'q_dot_rad_s'])
        self._cmd_csv.flush()
        self._fbk_csv.flush()
        self.get_logger().info(f'Logging to {log_dir}')
        # --------------------------------------------------------------------

        self.publisher = self.create_publisher(RobotCommand, self.command_topic, 1)
        self.subscription = self.create_subscription(
            MotorFeedback,
            self.feedback_topic,
            self._feedback_callback,
            10,
        )
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_callback)

        self.get_logger().info(
            f'SingleMotorTest ready: joint="{self.joint_name}" '
            f'target_q=+/-{self.target_q:.3f} rad  kp={self.kp}  kd={self.kd}  '
            f'step={self.step_duration}s  rate={self.rate_hz}Hz'
        )

    # ------------------------------------------------------------------
    def _now_s(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _timer_callback(self) -> None:
        elapsed_s = self._now_s()
        step = int(elapsed_s / self.step_duration) % 2
        q_des = self.target_q if step == 0 else -self.target_q

        msg = RobotCommand()
        msg.joint_names = [self.joint_name]
        msg.q_des   = [q_des]
        msg.qd_des  = [0.0]
        msg.kp      = [self.kp]
        msg.kd      = [self.kd]
        msg.tau_ff  = [self.tau_ff]
        self.publisher.publish(msg)

        self._cmd_writer.writerow([f'{elapsed_s:.4f}', f'{q_des:.6f}',
                                   f'{self.kp:.4f}', f'{self.kd:.4f}',
                                   f'{self.tau_ff:.4f}'])
        self._cmd_csv.flush()
        self._log.write(
            f'[{elapsed_s:.4f}]  CMD   joint={self.joint_name}'
            f'  q_des={q_des:.6f}  kp={self.kp:.4f}  kd={self.kd:.4f}  tau={self.tau_ff:.4f}\n')
        self._log.flush()

    # ------------------------------------------------------------------
    def _feedback_callback(self, msg: MotorFeedback) -> None:
        t = self._now_s()
        for motor in msg.motors:
            self._fbk_writer.writerow([f'{t:.4f}', motor.name,
                                       f'{motor.q:.6f}', f'{motor.q_dot:.6f}'])
            self._log.write(
                f'[{t:.4f}]  FBK   joint={motor.name}'
                f'  q={motor.q:.6f}  q_dot={motor.q_dot:.6f}\n')
        self._fbk_csv.flush()
        self._log.flush()

        for motor in msg.motors:
            if motor.name == self.joint_name:
                self.get_logger().info(
                    f'{motor.name}: q={motor.q:.4f} rad  q_dot={motor.q_dot:.4f} rad/s'
                )
                return
        if msg.motors:
            m = msg.motors[0]
            self.get_logger().info(
                f'[{m.name}]: q={m.q:.4f} rad  q_dot={m.q_dot:.4f} rad/s'
            )

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._cmd_csv.close()
        self._fbk_csv.close()
        self._log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SingleMotorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
