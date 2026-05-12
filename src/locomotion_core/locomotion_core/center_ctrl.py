#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist


def clamp(v, lo, hi): return max(lo, min(hi, v))
def wrap_pi(rad): return (rad + math.pi) % (2.0 * math.pi) - math.pi

def wrap_deg(d: float) -> float:
    """Wrap degrees to (-180, 180]."""
    return (d + 180.0) % 360.0 - 180.0


class StageRecenteringController(Node):
    def __init__(self):
        super().__init__("stage_recentering_controller")

        # Topics (MATCH your defaults)
        self.declare_parameter("stage_topic", "stage_pose")
        self.declare_parameter("cmd_vel_topic", "/rov_cmd_vel")

        # Targets
        self.declare_parameter("x_center_m", 0.12)
        self.declare_parameter("y_center_m", 0.12)
        self.declare_parameter("theta_target_deg", 0.0)

        # Gains (decoupled)
        self.declare_parameter("kx", 45.0)   # vx = -kx * ex
        self.declare_parameter("ky", 45.0)   # vy = -ky * ey
        self.declare_parameter("kw", 80.0)   # wz = -kw * theta_err_rad

        # Deadbands (decoupled)
        self.declare_parameter("deadband_x_m", 0.015)
        self.declare_parameter("deadband_y_m", 0.015)
        self.declare_parameter("deadband_w_rad", math.radians(8.0))

        # Output limits (decoupled)
        self.declare_parameter("max_vx", 6.0)
        self.declare_parameter("max_vy", 6.0)
        self.declare_parameter("max_wz", 6.0)

        # Your physical convention: stage +Y extends to the RIGHT
        # ROS cmd_vel +Y is LEFT => flip Y error
        self.declare_parameter("stage_y_is_right", True)

        # Safety: stop if topic stops publishing
        self.declare_parameter("sensor_timeout_s", 0.25)

        # Motion-gated engagement: ignore boot drift until stage moves
        self.declare_parameter("require_motion_to_engage", True)
        self.declare_parameter("arm_on_boot", False)
        self.declare_parameter("motion_threshold_mps", 0.02)

        # Rate
        self.declare_parameter("publish_rate_hz", 50.0)

        # ---- Load params ----
        p = self.get_parameter
        self.stage_topic = p("stage_topic").value
        self.cmd_vel_topic = p("cmd_vel_topic").value

        self.x_c = float(p("x_center_m").value)
        self.y_c = float(p("y_center_m").value)
        self.theta_target_deg = float(p("theta_target_deg").value)

        self.kx = float(p("kx").value)
        self.ky = float(p("ky").value)
        self.kw = float(p("kw").value)

        self.db_x = float(p("deadband_x_m").value)
        self.db_y = float(p("deadband_y_m").value)
        self.db_w = float(p("deadband_w_rad").value)

        self.max_vx = float(p("max_vx").value)
        self.max_vy = float(p("max_vy").value)
        self.max_wz = float(p("max_wz").value)

        self.stage_y_is_right = bool(p("stage_y_is_right").value)

        self.timeout_s = float(p("sensor_timeout_s").value)
        self.require_motion = bool(p("require_motion_to_engage").value)
        self.arm_on_boot = bool(p("arm_on_boot").value)
        self.motion_thresh = float(p("motion_threshold_mps").value)

        rate_hz = float(p("publish_rate_hz").value)
        self.period = 1.0 / max(1e-3, rate_hz)

        # ---- State ----
        self.last_sensor_time_sec = 0.0
        self.last_x = self.x_c
        self.last_y = self.y_c

        # Theta unwrap
        self.prev_theta_deg = None
        self.theta_unwrapped_deg = 0.0

        # Motion detection
        self.prev_x = None
        self.prev_y = None
        self.prev_time_sec = None

        # Armed latch (ignore boot drift unless arm_on_boot=True)
        self.armed = True if self.arm_on_boot else False

        # ROS I/O
        self.pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.sub = self.create_subscription(Pose, self.stage_topic, self.on_pose, 10)
        self.timer = self.create_timer(self.period, self.tick)

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def on_pose(self, msg: Pose):
        now_sec = self.now_sec()

        x = float(msg.position.x)
        y = float(msg.position.y)
        theta_deg = float(msg.orientation.z)  # your convention: [-180, +180]

        self.last_x = x
        self.last_y = y
        self.last_sensor_time_sec = now_sec

        # --- unwrap theta ---
        if self.prev_theta_deg is None:
            self.prev_theta_deg = theta_deg
            self.theta_unwrapped_deg = theta_deg
        else:
            d = wrap_deg(theta_deg - self.prev_theta_deg)
            self.theta_unwrapped_deg += d
            self.prev_theta_deg = theta_deg

        # --- arm only on motion (ignores boot drift) ---
        if self.prev_x is None:
            self.prev_x = x
            self.prev_y = y
            self.prev_time_sec = now_sec
            return

        dt = now_sec - self.prev_time_sec
        if dt > 1e-4:
            dx = x - self.prev_x
            dy = y - self.prev_y
            speed = math.sqrt(dx * dx + dy * dy) / dt
            if speed >= self.motion_thresh:
                self.armed = True

        self.prev_x = x
        self.prev_y = y
        self.prev_time_sec = now_sec

    def tick(self):
        now_sec = self.now_sec()
        stop = Twist()

        # 1) stop if sensor feed dies
        if (now_sec - self.last_sensor_time_sec) > self.timeout_s:
            self.pub.publish(stop)
            return

        # 2) ignore boot drift until armed
        if self.require_motion and not self.armed:
            self.pub.publish(stop)
            return

        # --- DECOUPLED errors ---
        ex = self.last_x - self.x_c                      # affects vx only
        ey = self.last_y - self.y_c                      # affects vy only
       # if self.stage_y_is_right:
       #     ey *= -1.0                                   # fix sign for ROS +Y (left)

        theta_err_deg = self.theta_unwrapped_deg - self.theta_target_deg
        theta_err_rad = -1*(wrap_pi(math.radians(theta_err_deg)))  # affects wz only

        # --- Per-axis deadband (only that axis goes to 0) ---
        if abs(ex) < self.db_x:
            ex = 0.0
        if abs(ey) < self.db_y:
            ey = 0.0
        if theta_err_rad < -0.15:
            theta_err_rad = -0.15
        if theta_err_rad > 0.15:
            theta_err_rad = 0.15
        if abs(theta_err_rad) < self.db_w:
            theta_err_rad = 0.0

        # If fully centered, stop and disarm (so it won't chase passive drift)
        if ex == 0.0 and ey == 0.0 and theta_err_rad == 0.0:
            self.pub.publish(stop)
            if self.require_motion and not self.arm_on_boot:
                self.armed = False
            return

        # --- Decoupled P control ---
        #if theta_err_rad != 0:
            #vx = clamp(-self.kx * 2 * ex/theta_err_rad, -self.max_vx, self.max_vx)
            #vy = clamp(-self.ky * 2 * ey/theta_err_rad, -self.max_vy, self.max_vy)
            #wz = clamp(-self.kw * theta_err_rad, -self.max_wz, self.max_wz)
        #else:
        vx = clamp(-self.kx * ex, -self.max_vx, self.max_vx)
        vy = clamp(-self.ky * ey, -self.max_vy, self.max_vy)
        wz = clamp(-self.kw * theta_err_rad, -self.max_wz, self.max_wz)
        
        #print(ex)
        #print('theta_err')
        #print(theta_err_rad)
        #if ex > 0:
        #    vx = -1.0
        #elif ex < 0:
        #    vx = 1.0
        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = 0.0
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = StageRecenteringController()
    try:
        rclpy.spin(node)
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

