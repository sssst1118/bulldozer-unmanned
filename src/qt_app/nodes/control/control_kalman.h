/**
 * @file control_kalman.h
 * @brief 卡尔曼滤波速度融合
 */
#ifndef CONTROL_KALMAN_H
#define CONTROL_KALMAN_H

namespace control {

class KalmanFilter {
public:
    KalmanFilter() : v_hat_(0), P_(1), counter_(1) {}

    double filter(double IMU_velocity, double LLA_velocity) {
        const double A = 1.0, H = 1.0, Q = 0.5;
        const double R_ins = 0.4, R_lla = 0.5;

        double z, R;
        if (counter_ % 2 == 1) { z = IMU_velocity; R = R_ins; }
        else                   { z = LLA_velocity;  R = R_lla; }

        double v_hat_minus = A * v_hat_;
        double P_minus = A * P_ * A + Q;
        double K = P_minus * H / (H * P_minus * H + R);
        v_hat_ = v_hat_minus + K * (z - H * v_hat_minus);
        P_ = (1 - K * H) * P_minus;

        counter_++;
        return v_hat_;
    }

    void reset() { v_hat_ = 0; P_ = 1; counter_ = 0; }

private:
    double v_hat_;
    double P_;
    int counter_;
};

} // namespace control
#endif // CONTROL_KALMAN_H
