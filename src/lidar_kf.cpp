#include <iostream>
#include "kf.hpp"

bool GetLidarData(double x, double y, double time_stamp)
{
    return true;
}

int main()
{
    double m_x = 0.0, m_y = 0.0;
    double last_timestamp = 0.0, cur_timestamp = 0.0;
    KalmanFilter kf;
    while(GetLidarData(m_x, m_y, cur_timestamp))
    {
        if(!kf.isInitialized())
        {
            last_timestamp = cur_timestamp;
            Eigen::VectorXd x_in(4, 1);
            x_in << m_x, m_y, 0, 0;
            kf.Initialization(x_in);
            Eigen::VectorXd P_in(4, 4);
            P_in << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 100.0, 0.0,
                0.0, 0.0, 0.0, 100.0;
            kf.SetP(P_in);
            Eigen::MatrixXd Q_in(4, 4);
            Q_in << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;
            kf.SetQ(Q_in);
            Eigen::MatrixXd H_in(2, 4);
            H_in << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0;
            kf.SetH(H_in);
            Eigen::MatrixXd R_in(2, 2);
            R_in << 0.0225, 0.0,
                0.0, 0.0225;
            kf.SetR(R_in);
            continue;
        }
        double delta = cur_timestamp - last_timestamp;
        last_timestamp = cur_timestamp;
        Eigen::MatrixXd F_in(4, 4);
        F_in << 1.0, 0.0, delta, 0.0,
            0.0, 1.0, 0.0, delta,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
        kf.SetF(F_in);
        kf.Prediction();
        Eigen::Vector2d z;
        z << m_x, m_y;
        kf.MeasurementUpdate(z);
        Eigen::VectorXd x_out = kf.GetX();
        std::cout << "kalman output x: " << x_out(0)
                  << " y: " << x_out(1) << std::endl;
    }
    return 0;
}
