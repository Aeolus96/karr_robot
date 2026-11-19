"""K.A.R.R. drive node: cmd_vel -> steering angles + wheel speeds, and joint states -> wheel odometry.

Subscribes  cmd_vel (geometry_msgs/Twist, or TwistStamped with use_stamped:=true; linear.x and angular.z used), joint_states
Publishes   <steering_controller>/commands, <wheel_controller>/commands (std_msgs/Float64MultiArray, ros2_control
            forward command controllers), odom (nav_msgs/Odometry), TF odom -> base_footprint (publish_tf),
            cmd_vel_limited (geometry_msgs/Twist: the command after the speed/turn limits, for debugging)

cmd_vel -> limits (limits.py: speed, acceleration, turn radius vs speed, swing turns) -> kinematics (kinematics.py) -> motors.
Wheels slow down while the steering motors lag behind their targets.

Change the steering mode while stopped:  ros2 param set /karr_drive steering_mode four_wheel
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster

from karr_control.kinematics import MODES, Module, body_twist, module_commands
from karr_control.limits import CommandLimiter, Limits, alignment_scale


class DriveNode(Node):
    def __init__(self):
        super().__init__('karr_drive')
        p = self.declare_parameter
        p('steering_mode', 'front')          # front | four_wheel
        p('modules', ['fl', 'fr', 'bl', 'br'])
        p('module_x', [0.3556, 0.3556, -0.3556, -0.3556])
        p('module_y', [0.2286, -0.2286, 0.2286, -0.2286])
        p('steered_modules', ['fl', 'fr', 'bl', 'br'])   # modules that have a steering motor
        p('wheel_radius', 0.1225)
        p('max_wheel_speed', 21.9)           # rad/s
        p('steering_limit', 1.5708)          # rad
        p('cmd_timeout', 0.5)                # s without cmd_vel -> stop
        p('rate', 50.0)                      # Hz
        p('use_stamped', False)
        p('publish_tf', True)
        p('odom_frame', 'odom')
        p('base_frame', 'base_footprint')
        p('steering_controller', 'steering_controller')
        p('wheel_controller', 'wheel_controller')
        for name, default in vars(Limits()).items():
            p(f'limits.{name}', default)
        p('steering_alignment.full_speed_below', 0.1)   # rad of steering error: full speed below this
        p('steering_alignment.stop_above', 0.5)         # rad: wheels stopped above this
        p('mode_switch_max_speed', 0.05)                # m/s or rad/s: steering_mode only changes below this

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.modules = [Module(n, x, y) for n, x, y in zip(g('modules'), g('module_x'), g('module_y'))]
        self.steered = list(g('steered_modules'))
        self.radius = g('wheel_radius')
        self.max_speed = g('max_wheel_speed')
        self.steer_limit = g('steering_limit')
        self.timeout = g('cmd_timeout')
        self.mode = g('steering_mode')
        self._check_mode(self.mode)
        self.limiter = CommandLimiter(Limits(**{n: type(d)(g(f'limits.{n}')) for n, d in vars(Limits()).items()}))
        self.align = (g('steering_alignment.full_speed_below'), g('steering_alignment.stop_above'))
        self.mode_switch_max = g('mode_switch_max_speed')

        self.steer_joints = [f'{n}_steering' for n in self.steered]
        self.wheel_joints = [f'{m.name}_wheel' for m in self.modules]
        self.cmd = (0.0, 0.0)
        self.last_cmd = None
        self.angles = {m.name: 0.0 for m in self.modules}
        self.pose = [0.0, 0.0, 0.0]
        self.last_js = None
        self.measured_angles = None        # steering angles from joint_states
        self.measured_twist = (0.0, 0.0, 0.0)
        self.last_step = None

        self.steer_pub = self.create_publisher(Float64MultiArray, f"/{g('steering_controller')}/commands", 10)
        self.wheel_pub = self.create_publisher(Float64MultiArray, f"/{g('wheel_controller')}/commands", 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.limited_pub = self.create_publisher(Twist, 'cmd_vel_limited', 10)
        self.tf = TransformBroadcaster(self) if g('publish_tf') else None
        if g('use_stamped'):
            self.create_subscription(TwistStamped, 'cmd_vel', lambda m: self._on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        self.create_subscription(JointState, 'joint_states', self._on_joints, 20)
        self.create_timer(1.0 / g('rate'), self._step)
        self.add_on_set_parameters_callback(self._on_params)
        lim = self.limiter.limits
        self.get_logger().info(f'steering_mode={self.mode}, steered modules: {" ".join(self.steered)}, '
                               f'max {lim.max_speed} m/s fwd / {lim.max_reverse_speed} m/s rev, '
                               f'lateral accel {lim.max_lateral_accel} m/s^2')

    # --- parameters --------------------------------------------------------------------------------------------
    def _check_mode(self, mode):
        if mode not in MODES:
            raise ValueError(f'steering_mode must be one of {MODES}')
        if mode == 'four_wheel':
            missing = [m.name for m in self.modules if m.name not in self.steered]
            if missing:
                raise ValueError(f'four_wheel needs a steering motor on every module; missing: {missing}')

    def _on_params(self, params):
        for prm in params:
            if prm.name == 'steering_mode':
                try:
                    self._check_mode(prm.value)
                except ValueError as e:
                    return SetParametersResult(successful=False, reason=str(e))
                moving = max(abs(self.limiter.vx), abs(self.limiter.wz), *map(abs, self.measured_twist))
                if moving > self.mode_switch_max and prm.value != self.mode:
                    return SetParametersResult(successful=False,
                                               reason='stop the robot first: steering_mode only changes at standstill')
                self.mode = prm.value
                self.get_logger().info(f'steering_mode -> {self.mode}')
        return SetParametersResult(successful=True)

    # --- command path ------------------------------------------------------------------------------------------
    def _on_cmd(self, t):
        self.cmd = (t.linear.x, t.angular.z)
        self.last_cmd = self.get_clock().now()

    def _step(self):
        now = self.get_clock().now()
        dt = 0.0 if self.last_step is None else (now - self.last_step).nanoseconds * 1e-9
        self.last_step = now
        dt = min(max(dt, 0.0), 0.1)       # sim time can pause or jump
        stale = self.last_cmd is None or (now - self.last_cmd).nanoseconds * 1e-9 > self.timeout
        vx_req, wz_req = (0.0, 0.0) if stale else self.cmd   # timeout: slow down at max_decel

        # Don't speed up while the steering motors are still far from their last targets
        if self.measured_angles is None:
            scale = 0.0                   # no steering feedback yet: don't drive
        else:
            scale = alignment_scale([self.angles[n] - self.measured_angles[n] for n in self.steered], *self.align)

        vx, wz = self.limiter.step(vx_req, wz_req, dt, speed_up=scale)
        cmds = module_commands(vx, wz, self.modules, self.mode, self.radius, self.steer_limit,
                               self.max_speed, prev_angles=self.angles)
        self.angles = {n: a for n, (a, _) in cmds.items()}
        self.steer_pub.publish(Float64MultiArray(data=[cmds[n][0] for n in self.steered]))
        self.wheel_pub.publish(Float64MultiArray(data=[cmds[m.name][1] * scale for m in self.modules]))
        out = Twist()
        out.linear.x, out.angular.z = vx * scale, wz * scale
        self.limited_pub.publish(out)

    # --- odometry ----------------------------------------------------------------------------------------------
    def _on_joints(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(j in idx for j in self.wheel_joints) or not msg.velocity:
            return
        angles = {m.name: (msg.position[idx[f'{m.name}_steering']] if f'{m.name}_steering' in idx else 0.0)
                  for m in self.modules}
        self.measured_angles = angles
        speeds = {m.name: msg.velocity[idx[f'{m.name}_wheel']] for m in self.modules}
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_js is None:
            self.last_js = stamp
            return
        dt = (stamp - self.last_js).nanoseconds * 1e-9
        self.last_js = stamp
        if dt <= 0 or dt > 1.0:
            return
        vx, vy, wz = body_twist(self.modules, angles, speeds, self.radius)
        self.measured_twist = (vx, vy, wz)
        x, y, th = self.pose
        mid = th + 0.5 * wz * dt
        x += (vx * math.cos(mid) - vy * math.sin(mid)) * dt
        y += (vx * math.sin(mid) + vy * math.cos(mid)) * dt
        th = math.atan2(math.sin(th + wz * dt), math.cos(th + wz * dt))
        self.pose = [x, y, th]

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = vx, vy, wz
        self.odom_pub.publish(odom)
        if self.tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = odom.child_frame_id
            t.transform.translation.x, t.transform.translation.y = x, y
            t.transform.rotation = odom.pose.pose.orientation
            self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = DriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
