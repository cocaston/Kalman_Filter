#ifndef __KF_H
#define __KF_H

#include "Eigen/Dense"

class KalmanFilter
{
public:
    KalmanFilter()
    {
        is_initialized_ = false;
    }

    ~KalmanFilter()
    {
        
    }

    bool isInitialized()
    {
        return is_initialized_;
    }

    void Initialization(Eigen::VectorXd x_in)
    {
        x_ = x_in;
        is_initialized_ = true;
    }

    void SetF(Eigen::VectorXd F_in)
    {
        F_ = F_in;
    }

    void SetP(Eigen::VectorXd P_in)
    {
        P_ = P_in;
    }

    void SetH(Eigen::VectorXd H_in)
    {
        H_ = H_in;
    }

    void SetQ(Eigen::VectorXd Q_in)
    {
        Q_ = Q_in;
    }

    void SetR(Eigen::VectorXd R_in)
    {
        R_ = R_in;
    }

    Eigen::Vector2d GetX()
    {
        return x_;
    }

    void Prediction()
    {
        x_ = F_ * x_;
        auto Ft = F_.transpose();
        P_ = F_ * P_ * Ft + Q_;
    }

    void MeasurementUpdate(const Eigen::VectorXd& z)
    {
        auto y = z - H_ * x_;
        auto Ht = H_.transpose();
        auto S = H_ * P_ * Ht + R_;
        auto K = P_ * Ht * S.inverse(); // 卡尔曼增益
        x_ = x_ + K * y;
        int size = x_.size();
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(size, size);
        P_ = (I - K * H_) * P_;
    }

private:
    bool is_initialized_;
    Eigen::VectorXd x_; // 状态向量
    Eigen::VectorXd F_; // 状态转移矩阵(state transistion matrix)
    Eigen::VectorXd P_; // 状态协方差矩阵(state covariance matrix)
    Eigen::VectorXd Q_; // 过程噪声(process covariance matrix)
    Eigen::VectorXd H_; // 观测矩阵(Measurement matrix)
    Eigen::VectorXd R_; // 观测噪声矩阵(Measurement covariance matrix)

};

#endif