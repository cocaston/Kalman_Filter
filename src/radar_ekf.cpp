#include <iostream>
#include <math.h>
#include "ekf.hpp"

bool getRadarData(double& rho, double& theta, double& rho_dot, double& now)
{
    return true;
}

int main()
{
    double m_rho = 0.0, m_theta = 0.0, m_rho_dot = 0.0;
    double last_timestamp = 0.0, cur_timestamp = 0.0;
    ExtendedKalmanFilter ekf;
    while(getRadarData(m_rho, m_theta, m_rho_dot, cur_timestamp))
    {
        if(!ekf.isInitialized())
        {
            last_timestamp = cur_timestamp;
            Eigen::VectorXd x_in(4, 1);
            x_in << m_rho * cos(m_theta), m_rho * sin(m_theta),
                m_rho_dot * cos(m_theta), m_rho_dot * sin(m_theta);
            ekf.Initialization(x_in);

            Eigen::MatrixXd P_in(4, 4);
            P_in << 1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 10.0, 0.0,
                0.0, 0.0, 0.0, 10.0;
            ekf.SetP(P_in);
            Eigen::MatrixXd Q_in = Eigen::MatrixXd::Identity(4, 4);
            ekf.SetQ(Q_in);
            Eigen::MatrixXd R_in(3, 3);
            R_in << 0.09, 0, 0,
                0, 0.0009, 0,
                0, 0, 0.09;
            ekf.SetR(R_in);
            continue;
        }

        double delta_t = cur_timestamp - last_timestamp;
        last_timestamp = cur_timestamp;
        Eigen::MatrixXd F_in = Eigen::Matrix4d();
        F_in(0, 3) = delta_t;
        F_in(1, 3) = delta_t;
        ekf.SetF(F_in);
        ekf.Prediction();

        Eigen::VectorXd z(3, 1);
        z << m_rho, m_theta, m_rho_dot;
        ekf.MeasurementUpdate(z);

        Eigen::VectorXd x_out = ekf.GetX();
        std::cout << "Kalman output x: " << x_out(0) << "y: " << x_out(1) << std::endl;
    }
}