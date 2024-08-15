#ifndef __EKF_H
#define __EKF_H

#include "Eigen/Dense"

class ExtendedKalmanFilter
{
public:
    ExtendedKalmanFilter()
    {
        is_initialized_ = false;
    }

    ~ExtendedKalmanFilter(){}

    bool isInitialized()
    {
        return is_initialized_;
    }

    // 初始化卡尔曼滤波需要输入一个初始状态量x_in，一般使用第一次测量值
    void Initialization(Eigen::VectorXd x_in)
    {
        x_ = x_in;
        is_initialized_ = true;
    }

    void SetF(Eigen::MatrixXd F_in)
    {
        F_ = F_in;
    }

    void SetP(Eigen::MatrixXd P_in)
    {
        P_ = P_in;
    }

    void SetQ(Eigen::MatrixXd Q_in)
    {
        Q_ = Q_in;
    }

    void SetR(Eigen::MatrixXd R_in)
    {
        R_ = R_in;
    }

    Eigen::VectorXd GetX()
    {
        return x_;
    }

    void Prediction()
    {
        x_ = F_ * x_;
        auto Ft = F_.transpose();
        P_ = F_ * P_ * Ft + Q_;
    }

    void CalculateJacobianMatrix()
    {
        Eigen::MatrixXd Hj(3, 4);
        float px = x_(0);
        float py = x_(1);
        float vx = x_(2);
        float vy = x_(3);

        float c1 = px * px + py * py;
        float c2 = sqrt(c1);
        float c3 = (c1 * c2);

        if(fabs(c1) < 0.0001)
        {
            H_ = Hj;
            return;
        }

        Hj << (px / c2), (py / c2), 0, 0,
            -(py / c1), (px / c1), 0, 0,
            py * (vx * py - vy * px) / c3, px * (px * vy - py * vx) / c3, px / c2, py / c2;

    }

    void MeasurementUpdate(Eigen::VectorXd& z)
    {
        double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
        double theta = atan2(x_(1), x_(0));
        double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
        Eigen::VectorXd h = Eigen::VectorXd(3);
        h << rho, theta, rho_dot;

        Eigen::VectorXd y = z - h;

        CalculateJacobianMatrix();

        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
        x_ = x_ + (K * y);
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(x_.size(), x_.size());
        P_ = (I - K * H_) * P_;
    }

private:
    bool is_initialized_;
    Eigen::VectorXd x_;
    Eigen::VectorXd F_;
    Eigen::VectorXd Q_;
    Eigen::VectorXd R_;
    Eigen::VectorXd H_;
    Eigen::VectorXd P_;
};

#endif