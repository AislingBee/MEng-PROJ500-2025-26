#!/usr/bin/env python3
# Publishes: /robot_command (RobotCommand) - N joints, all step ±target_q simultaneously
# Subscribes: /motor_feedback (MotorFeedback) - logs per-joint position and velocity
#
# Logs are written to <Software>/logs/<timestamp>/
#   commands.csv  - time_s, joint_name, q_des_rad, kp, kd, tau_ff_Nm
#   feedback.csv  - time_s, joint_name, q_rad, q_dot_rad_s

import csv
import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from motor_test.msg import RobotCommand, MotorFeedback
from motor_test.common import get_software_log_dir


class MultiMotorTest(Node):
    def __init__(self):
        super().__init__('multi_motor_test')

        self.declare_parameter('motor_count',    2)
        self.declare_parameter('joint_names',    '')        # comma-separated, e.g. "motor_1,motor_2"
        self.declare_parameter('target_q',       0.4)
        self.declare_parameter('kp',             20.0)
        self.declare_parameter('kd',             1.0)
        self.declare_parameter('tau_ff',         0.0)
        self.declare_parameter('step_duration',  4.0)
        self.declare_parameter('rate_hz',        50.0)
        self.declare_parameter('command_topic',  'robot_command')
        self.declare_parameter('feedback_topic', 'motor_feedback')
        self.declare_parameter('log_dir',        str(get_software_log_dir()))

        motor_count    = int(self.get_parameter('motor_count').value)
        names_param    = self.get_parameter('joint_names').value
        self.target_q  = float(self.get_parameter('target_q').value)
        self.kp        = float(self.get_parameter('kp').value)
        self.kd        = float(self.get_parameter('kd').value)
        self.tau_ff    = float(self.get_parameter('tau_ff').value)
        self.step_dur  = float(self.get_parameter('step_duration').value)
        self.rate_hz   = float(self.get_parameter('rate_hz').value)
        cmd_topic      = self.get_parameter('command_topic').value
        fbk_topic      = self.get_parameter('feedback_topic').value
        log_base       = self.get_parameter('log_dir').value

        # Build joint name list
        if names_param.strip():
            self.joint_names = [n.strip() for n in names_param.split(',') if n.strip()]
            if len(self.joint_names) != motor_count:
                self.get_logger().warning(
                    f'joint_names has {len(self.joint_names)} entries but motor_count={motor_count}; '
                    'using generated names')
                self.joint_names = [f'motor_{i + 1}' for i in range(motor_count)]
        else:
            self.joint_names = [f'motor_{i + 1}' for i in range(motor_count)]

        self.n = len(self.joint_names)

        # -- Log file setup --------------------------------------------------
        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        log_dir = Path(log_base) / timestamp
        log_dir.mkdir(parents=True, exist_ok=True)

        self._cmd_csv = open(log_dir / 'commands.csv',  'w', newline='', encoding='utf-8')
        self._fbk_csv = open(log_dir / 'feedback.csv',  'w', newline='', encoding='utf-8')
        self._log     = open(log_dir / 'motor_test.log', 'w', encoding='utf-8')
        self._cmd_writer = csv.writer(self._cmd_csv)
        self._fbk_writer = csv.writer(self._fbk_csv)
        self._cmd_writer.writerow(['time_s', 'joint_name', 'q_des_rad', 'kp', 'kd', 'tau_ff_Nm'])
        self._fbk_writer.writerow(['time_s', 'joint_name', 'q_rad', 'q_dot_rad_s'])
        self._cmd_csv.flush()
        self._fbk_csv.flush()
        self.get_logger().info(f'Logging to {log_dir}')
        # --------------------------------------------------------------------

        self.publisher = self.create_publisher(RobotCommand, cmd_topic, 1)
        self.subscription = self.create_subscription(
            MotorFeedback, fbk_topic, self._feedback_callback, 10)

        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.rate_hz, self._timer_callback)

        self.get_logger().info(
            f'MultiMotorTest ready: {self.n} motors {self.joint_names} '
            f'target_q=±{self.target_q:.3f} rad  kp={self.kp}  kd={self.kd}  '
            f'step={self.step_dur}s  rate={self.rate_hz}Hz'
        )

    # ------------------------------------------------------------------
    def _now_s(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _timer_callback(self):
        t = self._now_s()
        # Alternate between +target_q and -target_q every step_duration seconds
        phase = int(t / self.step_dur) % 2
        q_des = self.target_q if phase == 0 else -self.target_q

        msg = RobotCommand()
        msg.joint_names = self.joint_names
        msg.q_des       = [q_des]   * self.n
        msg.qd_des      = [0.0]     * self.n
        msg.kp          = [self.kp] * self.n
        msg.kd          = [self.kd] * self.n
        msg.tau_ff      = [self.tau_ff] * self.n
        self.publisher.publish(msg)

        for name in self.joint_names:
            self._cmd_writer.writerow([f'{t:.4f}', name, f'{q_des:.6f}',
                                        self.kp, self.kd, self.tau_ff])
            self._log.write(
                f'[{t:.4f}]  CMD   joint={name}'
                f'  q_des={q_des:.6f}  kp={self.kp:.4f}  kd={self.kd:.4f}  tau={self.tau_ff:.4f}\n')
        self._cmd_csv.flush()
        self._log.flush()

    # ------------------------------------------------------------------
    def _feedback_callback(self, msg: MotorFeedback):
        t = self._now_s()
        for entry in msg.entries:
            self._fbk_writer.writerow([
                f'{t:.4f}', entry.name,
                f'{entry.position:.6f}', f'{entry.velocity:.6f}'
            ])
            self._log.write(
                f'[{t:.4f}]  FBK   joint={entry.name}'
                f'  q={entry.position:.6f}  q_dot={entry.velocity:.6f}\n')
        self._fbk_csv.flush()
        self._log.flush()

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._cmd_csv.close()
        self._fbk_csv.close()
        self._log.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiMotorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
