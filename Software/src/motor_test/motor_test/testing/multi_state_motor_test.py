#!/usr/bin/env python3
# Multi-state 2+ motor test: cycles through configurable command states
# Publishes: /robot_command (RobotCommand) - N joints, step through defined states
# Subscribes: /motor_feedback (MotorFeedback) - logs per-joint position and velocity
#
# States are defined as tuples of (q_des_values,) where q_des_values is a list
# matching joint count. The test cycles through each state at state_duration.
#
# Log files in <Software>/logs/<timestamp>/

import csv
import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from motor_test.msg import RobotCommand, MotorFeedback
from motor_test.common import get_software_log_dir


class MultiStateMotorTest(Node):
    def __init__(self):
        super().__init__('multi_state_motor_test')

        self.declare_parameter('motor_count',    2)
        self.declare_parameter('joint_names',    '')           # comma-separated
        self.declare_parameter('states_json',    '[]')         # JSON list of states
        self.declare_parameter('kp',             50.0)
        self.declare_parameter('kd',             2.0)
        self.declare_parameter('tau_ff',         0.0)
        self.declare_parameter('state_duration',  3.0)
        self.declare_parameter('rate_hz',        50.0)
        self.declare_parameter('command_topic',  'robot_command')
        self.declare_parameter('feedback_topic', 'motor_feedback')
        self.declare_parameter('log_dir',        str(get_software_log_dir()))

        motor_count    = int(self.get_parameter('motor_count').value)
        names_param    = self.get_parameter('joint_names').value
        states_str     = self.get_parameter('states_json').value
        self.kp        = float(self.get_parameter('kp').value)
        self.kd        = float(self.get_parameter('kd').value)
        self.tau_ff    = float(self.get_parameter('tau_ff').value)
        self.state_dur = float(self.get_parameter('state_duration').value)
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

        # Parse states from JSON
        import json
        try:
            states_raw = json.loads(states_str) if states_str.strip() else []
            self.states = []
            for state_def in states_raw:
                if isinstance(state_def, (list, tuple)) and len(state_def) == self.n:
                    self.states.append(list(state_def))
                else:
                    self.get_logger().warning(f'Skipping invalid state: {state_def}')
            
            if not self.states:
                self.get_logger().warning('No valid states defined; using default alternating pattern')
                self.states = [
                    [0.0] * self.n,
                    [0.5] * self.n,
                    [-0.5] * self.n,
                ]
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse states_json: {e}')
            self.states = [[0.0] * self.n, [0.5] * self.n, [-0.5] * self.n]

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
        self.last_state_idx = -1

        self.get_logger().info(
            f'MultiStateMotorTest ready: {self.n} motors {self.joint_names} '
            f'states={self.states}  kp={self.kp}  kd={self.kd}  '
            f'state_duration={self.state_dur}s  rate={self.rate_hz}Hz'
        )

    # ------------------------------------------------------------------
    def _now_s(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    # ------------------------------------------------------------------
    def _timer_callback(self):
        t = self._now_s()
        
        # Cycle through states
        state_idx = int(t / self.state_dur) % len(self.states)
        
        # Log state transition
        if state_idx != self.last_state_idx:
            self.last_state_idx = state_idx
            q_des_list = self.states[state_idx]
            self.get_logger().info(f'[{t:.2f}] → STATE {state_idx}: {q_des_list}')
        
        q_des_list = self.states[state_idx]

        msg = RobotCommand()
        msg.joint_names = self.joint_names
        msg.q_des       = q_des_list
        msg.qd_des      = [0.0]     * self.n
        msg.kp          = [self.kp] * self.n
        msg.kd          = [self.kd] * self.n
        msg.tau_ff      = [self.tau_ff] * self.n
        self.publisher.publish(msg)

        for i, name in enumerate(self.joint_names):
            q_des = q_des_list[i]
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
    node = MultiStateMotorTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
