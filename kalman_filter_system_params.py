import numpy as np


class KalmanFilter:
    C = np.array([(1, 0)], dtype=float)
    C_T = np.transpose(C)
    Q = C.dot(np.transpose(C))
    R = 1

    def __init__(self, P_k, x_k, standard_deviation, discrete_step):
        self.D = standard_deviation
        self.h = discrete_step
        self.A = np.array([(1, self.h), (0, 1)], dtype=float)
        self.A_T = np.transpose(self.A)
        self.B = np.array([[self.h**2/2], [self.h]], dtype=float) * standard_deviation
        self.BQB_T = self.B.dot(self.Q).dot(np.transpose(self.B))
        self.DRD_T = self.D * self.D
        self.P_k = P_k
        self.x_k = x_k

    # Discrete Time Riccati Equation
    def _DTRE_update_of_Pk(self, P_k):
        DTRE_1 = self.A.dot(P_k).dot(self.A_T) + self.BQB_T
        DTRE_2 = self.A.dot(P_k).dot(self.C_T)
        DTRE_3 = self.C.dot(P_k).dot(self.A_T)
        DTRE_4 = self.C.dot(P_k).dot(self.C_T) + self.DRD_T
        DTRE_5 = DTRE_2.dot(np.linalg.inv(DTRE_4)).dot(DTRE_3)
        return DTRE_1 - DTRE_5

    def _update_kalman_gain(self, P_k):
        kalman_1 = np.linalg.inv(self.C.dot(P_k).dot(self.C_T) + self.DRD_T)
        kalman_2 = np.linalg.inv(kalman_1)
        return P_k.dot(self.C_T).dot(kalman_1)

    def _update_state(self, K_k, x_k, y_k):
        state_1 = self.A - self.A.dot(K_k).dot(self.C)
        state_2 = state_1.dot(x_k)
        state_3 = self.A.dot(K_k).dot(y_k)
        return state_2 + state_3

    def update(self, y_k):
        self.P_k = self._DTRE_update_of_Pk(self.P_k)
        gain_update = self._update_kalman_gain(self.P_k)
        self.x_k = self._update_state(gain_update, self.x_k, y_k)
