# This code is written in Python 3 environment

import numpy as np
from numba import njit, float64, int64
from numba.experimental import jitclass

# Stanley Controller  with Feed Forward Term
spec = [('_kp', float64), ('_ki', float64), ('_kd', float64), ('_ff_params', float64[:]),
        ('_sat_long_max', float64), ('_sat_long_min', float64),
        ('_ev', float64), ('_ev_last', float64), ('_ev_sum', float64), ('_ev_int_state', int64),
        ('_ks', float64), ('_kv', float64), ('_l', float64),
        ('_dead_band_lat', float64), ('_sat_lat_max', float64),
        ('_sat_lat_min', float64), ('_e_lat', float64), ('_e_yaw', float64),
        ('_waypoints', float64[:, :]), ('_closest_idx', int64),
        ('_kv_yaw', float64), ('_kv_lat', float64), ('_min_vel_move', float64),
        ('_fc_long', float64), ('_fc_lat', float64),
        ('_cs_long', float64), ('_cs_lat', float64),]

@jitclass(spec)
class Controller(object):
    def __init__(self, kp, ki, kd, feed_forward_params, sat_long,\
                 ks, kv, length, lateral_dead_band, sat_lat,\
                 waypoints, min_vel_move, kv_yaw, kv_lat,
                 fc_long, fc_lat):
        # In this version, the integral term will be clamped based on the
        # saturation value and the feed-forward term

        # The parameters of the longitudinal controller
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self._ff_params = feed_forward_params
        self._sat_long_max = max(sat_long[0], sat_long[1])
        self._sat_long_min = min(sat_long[0], sat_long[1])
        self._ev = 0.
        self._ev_sum = 0.
        self._ev_last = 0.
        self._ev_int_state = 0 # 0 --> No Saturation || 1 --> Saturation (+) || -1 --> Saturation (-1)

        # The parameters of the lateral controller
        self._ks = ks
        self._kv = kv
        self._l = length
        self._dead_band_lat = lateral_dead_band
        self._sat_lat_max = np.fmax(sat_lat[0], sat_lat[1])
        self._sat_lat_min = np.fmin(sat_lat[0], sat_lat[1])
        self._e_lat = 0.
        self._e_yaw = 0.

        # Waypoints (n, 5) -> x, y, yaw, v, curvature
        self._waypoints = waypoints
        self._closest_idx = 0

        # Additional longitudinal control
        self._min_vel_move = min_vel_move
        self._kv_yaw = kv_yaw
        self._kv_lat = kv_lat

        # LPF cuf-off frequency
        self._fc_long = 2.*np.pi*fc_long # rad/s
        self._fc_lat = 2.*np.pi*fc_lat # rad/s

        # Initialize the control signal
        self._cs_long = 0.0
        self._cs_lat = 0.0

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def reset_integral_derivative(self):
        self._ev_sum = 0.0
        self._ev_last = 0.0

    def get_error(self):
        return self._ev, self._e_lat, self._e_yaw

    def get_closest_index(self):
        return self._closest_idx

    def get_instantaneous_setpoint(self):
        out = np.copy(self._waypoints[self._closest_idx])
        if out[3] <= 0.05: # If the desired speed is too low or STOP
            None
        else:
            out[3] = np.fmax(out[3] / (1.0 + self._kv_lat*self._e_lat**2 + self._kv_yaw*self._e_yaw**2), self._min_vel_move)
        return out

    def _update_error(self, x, y, v, yaw):
        # Waypoints (n, 5) -> x, y, yaw, v, curvature

        # Find the closest waypoint
        self._closest_idx = np.argmin(np.sum(np.square(self._waypoints[:, :2] - np.array([x, y])), axis=-1))

        # Find the yaw error
        self._e_yaw = self._waypoints[self._closest_idx, 2] - yaw
        self._e_yaw = (self._e_yaw + np.pi) % (2 * np.pi) - np.pi # Wrap the angle to [-pi, pi)

        # Find the lateral or crosstrack error
        if self._closest_idx == 0:
            idx = 1
        else:
            idx = self._closest_idx
        y2 = self._waypoints[idx, 1]
        x2 = self._waypoints[idx, 0]
        y1 = self._waypoints[idx - 1, 1]
        x1 = self._waypoints[idx - 1, 0]
        dy = y2 - y1
        dx = x2 - x1
        c = dx*y1 - dy*x1
        self._e_lat = (dy*x + c - dx*y) \
                        / (np.sqrt(dx**2 + dy**2) + 10**(-32))

        # Find the velocity error
        # self._ev = self._waypoints[self._closest_idx, 3] - v
        self._ev = self.get_instantaneous_setpoint()[3] - v

    def _feed_forward_longitudinal(self, v):
        if v < 0.1:
            return 0.
        else:
            return self._ff_params[0] * (1. - np.exp(self._ff_params[1] * v)) + self._ff_params[2]

    def _feed_forward_lateral(self):
        temp = self._l * self._waypoints[self._closest_idx, -1]
        if np.abs(temp) >= 1.:
            temp = np.sign(temp)
        return np.fmax(np.fmin(np.arcsin(temp), self._sat_lat_max), self._sat_lat_min) # From -pi/2 to pi/2

    def calculate_control_signal(self, dt, x, y, v, yaw):
        # Waypoints (n, 5) -> x, y, yaw, v, curvature
        self._update_error(x, y, v, yaw)

        # Longitudinal control
        ff_long = self._feed_forward_longitudinal(self._waypoints[self._closest_idx, 3])
        ev_dot = (self._ev - self._ev_last) / dt

        if self._ev_int_state == 0: # No saturation
            self._ev_sum = self._ev_sum + self._ev * dt
        elif self._ev_int_state == 1: # Saturation (Max.)
            if self._ev < 0:
                self._ev_sum = self._ev_sum + self._ev * dt
        elif self._ev_int_state == -1: # Saturation (Min.)
            if self._ev > 0:
                self._ev_sum = self._ev_sum + self._ev * dt

        cs_long = ff_long +\
                    self._kp * self._ev +\
                    self._ki * self._ev_sum +\
                    self._kd * ev_dot

        if cs_long > self._sat_long_max: # Saturation (+)
            cs_long = self._sat_long_max
            self._ev_int_state = 1
        elif cs_long < self._sat_long_min: # Saturation (-)
            cs_long = self._sat_long_min
            self._ev_int_state = -1
        else: # Not saturated
            self._ev_int_state = 0

        self._cs_long = (self._cs_long + dt*self._fc_long*cs_long) / (1. + dt*self._fc_long)
        self._ev_last = self._ev

        # STANLEY Controller
        temp = 0.0
        if np.abs(self._e_lat) > self._dead_band_lat:
            temp = self._e_lat

        a = self._feed_forward_lateral()
        b = self._e_yaw
        c = np.arctan(self._ks * temp / (self._kv + v))
        d = a + b + c # Use Stanley !

        cs_lat = max(min(d, self._sat_lat_max), self._sat_lat_min)
        self._cs_lat = (self._cs_lat + dt*self._fc_lat*cs_lat) / (1. + dt*self._fc_lat)

        return self._cs_long, self._cs_lat

print("Compilling the Controller class ...")
controller = Controller(0.5, 0.1, 0.1, np.array([1., 2.]),
                        np.array([-1., 1.]), 2.0, 0.1, 2.5,
                        0.01, np.array([-np.pi/3., np.pi/3.]),
                        np.random.randn(100, 5), 0.5, 1., 1.,
                        5., 5.)
controller.update_waypoints(np.random.randn(100, 5))
controller.reset_integral_derivative()
_ = controller.get_error()
_ = controller.get_instantaneous_setpoint()
_ = controller.get_closest_index()
# controller._update_error(0., 0., 1.0, 0.)
# _ = controller._feed_forward_longitudinal(2.5)
# _ = controller._feed_forward_lateral()
_ = controller.calculate_control_signal(0.01, 0., 0., 1.0, 0.)
print("The Controller class has been compiled !")
#########################################################################################
