import rclpy
from rclpy.node import Node
import numpy as np
from collections import deque

from ngc_utils.qos_profiles import default_qos_profile
from ngc_utils.math_utils import RotationMatrix, mapToPiPi
from ngc_utils.geo_utils import add_distance_to_lat_lon, calculate_distance_north_east
from ngc_utils.ngc_base_node import NgcBaseNode
from ngc_interfaces.msg import StateEstimate, Tau, GNSS, HeadingDevice
from std_msgs.msg import Bool, Float64MultiArray


class StateEstimator(Node):
    """
    6-DOF Extended Kalman Filter for USV state estimation.
    State vector: [north, east, psi, u, v, r]
    Fuses GNSS position, gyrocompass heading, and body-frame velocities.
    """

    def __init__(self):
        super().__init__('state_est')

        self.base = NgcBaseNode(self, ['simulator', 'vessel', 'control'])
        self.simulation_config = self.base.simulator_config
        self.vessel_model = self.base.vessel_model
        self.control = self.base.control_config

        # Subscriptions
        self.create_subscription(GNSS, "gnss_sim_meas", self._gnss_cb, default_qos_profile)
        self.create_subscription(HeadingDevice, "gyrocompass_sim_meas", self._gyro_cb, default_qos_profile)
        self.create_subscription(Tau, "tau_control", self._tau_cb, default_qos_profile)
        self.create_subscription(Bool, "update_control", self._update_params_cb, default_qos_profile)

        # Publishers
        self.state_est_pub = self.create_publisher(StateEstimate, "state_estimate", default_qos_profile)
        self.adaptive_r_pub = self.create_publisher(Float64MultiArray, "adaptive_r", default_qos_profile)

        # Measurement state
        self.latitude_meas = None
        self.longitude_meas = None
        self.heading_deg_meas = None
        self.surge_vel_meas = None
        self.sway_vel_meas = None
        self.rate_of_turn_degsec_meas = None

        # Estimated state
        self.latitude_est = None
        self.longitude_est = None
        self.heading_deg_est = None
        self.surge_vel_est = None
        self.sway_vel_est = None
        self.rate_of_turn_degsec_est = None

        # Control input
        self.surge_x_force = 0.0
        self.sway_y_force = 0.0
        self.yaw_n_moment = 0.0

        # EKF covariance
        self.P = np.eye(6)

        # Load Q and R from config
        ekf_cfg = self.control['state_estimator']['ekf']
        q = ekf_cfg['process_noise']
        r = ekf_cfg['measurement_noise']

        self.Q = np.diag([q['north'], q['east'], q['heading'],
                          q['surge_vel'], q['sway_vel'], q['yaw_rate']])

        self.R = np.diag([r['north'], r['east'], r['heading'],
                          r['surge_vel'], r['sway_vel'], r['yaw_rate']])

        # R_min floors for adaptive R (based on standalone GNSS specs)
        self.R_min = np.array([1.44, 1.44, 0.04, 0.0025, 0.0025, 0.0076])

        # Adaptive Q/R via running variance buffer
        self.use_adaptive_qr = self.control['state_estimator']['use_adaptive_qr']
        self.buffer_size = 100  # 100 steps = 10 s at 10 Hz
        self.measurement_buffer = [deque(maxlen=self.buffer_size) for _ in range(6)]
        self.adaptation_delay = 200
        self.step_counter = 0

        # 10 Hz timer
        self.step_size = 0.1
        self.timer = self.create_timer(self.step_size, self._step)
        self.get_logger().info("State estimator initialized.")

    def compute_F_jacobian(self, psi, u, v, r):
        """
        Linearized discrete-time process Jacobian: F = I + dt * A.
        Includes nonlinear hydrodynamic damping derivatives from vessel model.
        """
        hc = self.vessel_model.hydrodynamic_coefficients['nonlinear_damping']
        Minv = self.vessel_model.MassInv3Dof

        X_uu = hc['X_uu']
        Y_vv = hc['Y_vv']
        N_rr = hc['N_rr']

        A = np.zeros((6, 6))

        # Kinematic rows
        A[0, 2] = -u * np.sin(psi) - v * np.cos(psi)
        A[0, 3] = np.cos(psi)
        A[0, 4] = -np.sin(psi)
        A[1, 2] = u * np.cos(psi) - v * np.sin(psi)
        A[1, 3] = np.sin(psi)
        A[1, 4] = np.cos(psi)
        A[2, 5] = 1.0

        # Dynamic rows — damping Jacobian scaled by inverse mass
        A[3, 3] = Minv[0, 0] * (-2.0 * X_uu * abs(u))
        A[4, 4] = Minv[1, 1] * (-2.0 * Y_vv * abs(v))
        A[5, 5] = Minv[2, 2] * (-2.0 * N_rr * abs(r))

        return np.eye(6) + self.step_size * A

    # ── Callbacks ──

    def _gnss_cb(self, msg: GNSS):
        if not msg.signal_valid:
            self.latitude_meas = None
            self.longitude_meas = None
            self.surge_vel_meas = None
            self.sway_vel_meas = None
            return

        self.latitude_meas = msg.latitude
        self.longitude_meas = msg.longitude

        if self.longitude_est is None:
            self.longitude_est = self.longitude_meas
        if self.latitude_est is None:
            self.latitude_est = self.latitude_meas

        # Decompose SOG/COG into body-frame velocities
        psi = np.deg2rad(self.heading_deg_est) if self.heading_deg_est is not None else 0.0
        v_north = msg.sog * np.cos(np.deg2rad(msg.cog))
        v_east = msg.sog * np.sin(np.deg2rad(msg.cog))
        self.surge_vel_meas = v_north * np.cos(psi) + v_east * np.sin(psi)
        self.sway_vel_meas = -v_north * np.sin(psi) + v_east * np.cos(psi)

        if self.surge_vel_est is None:
            self.surge_vel_est = self.surge_vel_meas
        if self.sway_vel_est is None:
            self.sway_vel_est = self.sway_vel_meas

    def _gyro_cb(self, msg: HeadingDevice):
        if not msg.signal_valid:
            self.heading_deg_meas = None
            self.rate_of_turn_degsec_meas = None
            return

        self.heading_deg_meas = msg.heading_deg
        self.rate_of_turn_degsec_meas = msg.rot_degsec

        if self.heading_deg_est is None:
            self.heading_deg_est = self.heading_deg_meas
        if self.rate_of_turn_degsec_est is None:
            self.rate_of_turn_degsec_est = self.rate_of_turn_degsec_meas

    def _tau_cb(self, msg: Tau):
        self.surge_x_force = msg.surge_x
        self.sway_y_force = msg.sway_y
        self.yaw_n_moment = msg.yaw_n

    def _update_params_cb(self, msg):
        self.base.load_and_initialize_configs('control')
        self.control = self.base.control_config
        self.use_adaptive_qr = self.control['state_estimator']['use_adaptive_qr']

        if self.use_adaptive_qr:
            self.step_counter = 0
            self.measurement_buffer = [deque(maxlen=self.buffer_size) for _ in range(6)]

        q = self.control['state_estimator']['ekf']['process_noise']
        r = self.control['state_estimator']['ekf']['measurement_noise']
        self.Q = np.diag([q['north'], q['east'], q['heading'],
                          q['surge_vel'], q['sway_vel'], q['yaw_rate']])

        if not self.use_adaptive_qr:
            self.R = np.diag([r['north'], r['east'], r['heading'],
                              r['surge_vel'], r['sway_vel'], r['yaw_rate']])

    # ── Main loop ──

    def _step(self):
        self.base.publish_node_status()

        # Need at least one valid fix before we can start
        if self.latitude_est is None or self.heading_deg_est is None:
            return

        # ── Prediction (always runs — dead-reckons on the vessel model) ──
        velocity_k = np.array([
            self.surge_vel_est,
            self.sway_vel_est,
            np.deg2rad(self.rate_of_turn_degsec_est)
        ], dtype=float)

        Minv = self.vessel_model.MassInv3Dof
        damping = self.vessel_model.get_3dof_hydrodynamic_damping(velocity_k)
        tau = np.array([self.surge_x_force, self.sway_y_force, self.yaw_n_moment], dtype=float)

        acceleration = Minv @ (damping + tau)
        velocity_pred = velocity_k + self.step_size * acceleration

        ned_vel = RotationMatrix(0.0, 0.0, np.deg2rad(self.heading_deg_est)) @ velocity_pred
        delta_north = self.step_size * ned_vel[0]
        delta_east = self.step_size * ned_vel[1]
        delta_psi = self.step_size * velocity_pred[2]

        self.latitude_est, self.longitude_est = add_distance_to_lat_lon(
            self.latitude_est, self.longitude_est, delta_north, delta_east)
        self.heading_deg_est = np.rad2deg(mapToPiPi(np.deg2rad(self.heading_deg_est) + delta_psi))
        self.surge_vel_est = velocity_pred[0]
        self.sway_vel_est = velocity_pred[1]
        self.rate_of_turn_degsec_est = np.rad2deg(velocity_pred[2])

        # Covariance prediction
        psi = np.deg2rad(self.heading_deg_est)
        F = self.compute_F_jacobian(psi, velocity_k[0], velocity_k[1], velocity_k[2])
        self.P = F @ self.P @ F.T + self.Q

        # ── Correction (only for channels with valid measurements) ──
        has_position = self.latitude_meas is not None and self.longitude_meas is not None
        has_heading = self.heading_deg_meas is not None
        has_velocity = (self.surge_vel_meas is not None
                        and self.sway_vel_meas is not None
                        and self.rate_of_turn_degsec_meas is not None)

        if has_position or has_heading or has_velocity:
            y_tilde = np.zeros(6)
            # Mask: 1 for channels with valid data, 0 for missing
            H_mask = np.zeros(6)

            if has_position:
                dn, de = calculate_distance_north_east(
                    self.latitude_est, self.longitude_est,
                    self.latitude_meas, self.longitude_meas)
                y_tilde[0] = dn
                y_tilde[1] = de
                H_mask[0] = 1.0
                H_mask[1] = 1.0

            if has_heading:
                y_tilde[2] = mapToPiPi(
                    np.deg2rad(self.heading_deg_meas) - np.deg2rad(self.heading_deg_est))
                H_mask[2] = 1.0

            if has_velocity:
                y_tilde[3] = self.surge_vel_meas - self.surge_vel_est
                y_tilde[4] = self.sway_vel_meas - self.sway_vel_est
                y_tilde[5] = (np.deg2rad(self.rate_of_turn_degsec_meas)
                              - np.deg2rad(self.rate_of_turn_degsec_est))
                H_mask[3] = 1.0
                H_mask[4] = 1.0
                H_mask[5] = 1.0

            H = np.diag(H_mask)
            S = H @ self.P @ H.T + self.R
            K = self.P @ H.T @ np.linalg.inv(S)
            correction = K @ y_tilde
            self.P = (np.eye(6) - K @ H) @ self.P

            # Apply correction
            self.latitude_est, self.longitude_est = add_distance_to_lat_lon(
                self.latitude_est, self.longitude_est, correction[0], correction[1])
            self.heading_deg_est = np.rad2deg(
                mapToPiPi(np.deg2rad(self.heading_deg_est) + correction[2]))
            self.surge_vel_est += correction[3]
            self.sway_vel_est += correction[4]
            self.rate_of_turn_degsec_est += np.rad2deg(correction[5])

        # ── Publish state estimate ──
        msg = StateEstimate()
        msg.latitude = self.latitude_est
        msg.longitude = self.longitude_est
        msg.heading_deg = self.heading_deg_est
        msg.u_ms = self.surge_vel_est
        msg.v_ms = self.sway_vel_est
        msg.r_degsec = self.rate_of_turn_degsec_est
        self.state_est_pub.publish(msg)

        # ── Adaptive R ──
        self.step_counter += 1

        if has_position:
            self.measurement_buffer[0].append(y_tilde[0])
            self.measurement_buffer[1].append(y_tilde[1])
        if has_heading:
            self.measurement_buffer[2].append(np.deg2rad(self.heading_deg_meas))
        if self.surge_vel_meas is not None:
            self.measurement_buffer[3].append(self.surge_vel_meas)
        if self.sway_vel_meas is not None:
            self.measurement_buffer[4].append(self.sway_vel_meas)
        if self.rate_of_turn_degsec_meas is not None:
            self.measurement_buffer[5].append(np.deg2rad(self.rate_of_turn_degsec_meas))

        if self.use_adaptive_qr:
            for i in range(6):
                if len(self.measurement_buffer[i]) == self.buffer_size:
                    self.R[i, i] = max(np.var(self.measurement_buffer[i]), self.R_min[i])

        r_msg = Float64MultiArray()
        r_msg.data = [self.R[i, i] for i in range(6)]
        self.adaptive_r_pub.publish(r_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
