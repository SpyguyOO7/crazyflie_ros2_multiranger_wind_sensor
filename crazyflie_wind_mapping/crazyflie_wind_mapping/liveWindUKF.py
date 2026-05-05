

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
from scipy.signal import butter, filtfilt
ukf = None

class AirflowMagnitudeUKF:
    """
    UKF estimating relative airflow magnitude only.

    State: [va_mag] - airflow speed magnitude
    Measurement: [B_mag] - magnetic field magnitude
    """

    def __init__(self, dt, Q_airflow=0.5, R_magnetic=None, R_odom=0.003):
        self.dt = dt
        # December model
        self.inv_a = -0.000213
        self.inv_b =  0.03955
        self.inv_c =  0.03621
        # OSL model
        # self.inv_a = -.001732  # -0.000213
        # self.inv_b = 0.1088  # 0.03955
        # self.inv_c = 0.1061  # 0.03621


        self.R_odom = R_odom

        # State dimension
        self.dim_x = 1  # [va_mag]
        self.dim_z = 1  # [B_mag]

        # Sigma points
        points = MerweScaledSigmaPoints(n=self.dim_x, alpha=0.1, beta=2.0, kappa=0.0)

        # Create UKF
        self.ukf = UKF(
            dim_x=self.dim_x,
            dim_z=self.dim_z,
            dt=dt,
            fx=self._fx,
            hx=self._hx,
            points=points
        )

        # Initial state: zero airflow
        self.ukf.x = np.array([0.0])

        # Initial covariance
        self.ukf.P = np.array([[1.0]])

        # Process noise
        self.ukf.Q = np.array([[Q_airflow]])

        # Measurement noise
        if R_magnetic is not None:
            self.ukf.R = np.array([[R_magnetic]])
        else:
            # Default: approximate from your 2x2 matrix (use average variance)
            self.ukf.R = np.array([[0.74]])

    def _fx(self, x, dt):
        """Process model: random walk on magnitude."""
        return x

    def _hx(self, x):
        """Measurement model: v_mag -> B_mag using quadratic model."""
        va_mag = np.maximum(x[0], 0)  # Ensure non-negative

        # B_mag = a*v^2 + b*v + c
        # December model
        a = 29.47
        b = -7.7
        c = 5.44

        # OSL model
        # a = 10.91
        # b = -5.486
        # c = 2.739


        B_mag = a * va_mag ** 2 + b * va_mag + c
        return np.array([B_mag])

    def predict(self):
        """Prediction step."""
        self.ukf.P = 0.5 * (self.ukf.P + self.ukf.P.T)
        self.ukf.P += np.eye(self.dim_x) * 1e-6
        self.ukf.predict()

        # Ensure non-negative after predict
        self.ukf.x[0] = max(self.ukf.x[0], 0)

    def update_with_velocity(self, v_mag):
        """Soft constraint from odometry velocity magnitude."""
        z = np.array([v_mag])
        H = np.array([[1.0]])
        R_odom = np.array([[self.R_odom]])

        y = z - self.ukf.x
        S = H @ self.ukf.P @ H.T + R_odom
        K = self.ukf.P @ H.T / S[0, 0]

        self.ukf.x = self.ukf.x + K.flatten() * y[0]
        self.ukf.P = (np.eye(1) - K @ H) @ self.ukf.P

        # Ensure non-negative
        self.ukf.x[0] = max(self.ukf.x[0], 0)

    def update(self, B_mag):
        """Measurement update with magnetic field magnitude."""
        z = np.array([B_mag])

        self.ukf.P = 0.5 * (self.ukf.P + self.ukf.P.T)
        self.ukf.P += np.eye(self.dim_x) * 1e-6


        self.ukf.update(z)

        # Ensure non-negative
        self.ukf.x[0] = max(self.ukf.x[0], 0)

    def B_to_velocity(self, B_mag):
        """Estimate airflow speed from B magnitude."""
        v = self.inv_a * B_mag ** 2 + self.inv_b * B_mag + self.inv_c
        return np.maximum(v, 0)

    @property
    def airflow_mag(self):
        return self.ukf.x[0]

def init_ukf():
    global ukf
    Q_airflow = 0.005#0.01
    R_odom = 0.003
    R_magnetic = 0.74
    ukf = AirflowMagnitudeUKF(0.02, Q_airflow=Q_airflow,
                              R_magnetic=R_magnetic, R_odom=R_odom)
# =============================================================================
# Run UKF
# =============================================================================

def run_ukf(B,v_mag):
    """Run magnitude-only UKF on a dataset and return results."""
    global ukf
    # ukf = AirflowMagnitudeUKF(0.01, Q_airflow=Q_airflow,
    #                           R_magnetic=R_magnetic, R_odom=R_odom)
    if ukf is None:
        init_ukf()

    ukf.predict()
    ukf.update(B)
    ukf.update_with_velocity(v_mag)

    airflow_mag = ukf.airflow_mag

    wind_mag = abs(ukf.airflow_mag - v_mag)

    empirical = ukf.B_to_velocity(B)

    return airflow_mag,wind_mag,empirical

def reset_ukf():
    """Reset the global UKF instance."""
    global ukf
    ukf = None


#airflow, wind = liveWindUKF.run_ukf(B,v_mag)
