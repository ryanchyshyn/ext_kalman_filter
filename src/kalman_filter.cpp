#include "kalman_filter.h"

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() :
	x_(4),
	P_(4, 4),
	F_(4, 4),
	H_(2, 4),
	R_(2, 2),
	Q_(4, 4) {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Predict() {
	/**
	TODO:
	  * predict the state
	*/

	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	/**
	TODO:
	  * update the state by using Kalman Filter equations
	*/

	VectorXd y = z - H_ * x_;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	// new state
	x_ = x_ + (K * y);

	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
	/**
	TODO:
	  * update the state by using Extended Kalman Filter equations
	*/

	float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
	float phi = atan2(x_(1), x_(0));
	float rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;

	VectorXd z_pred(3);
	z_pred << rho, phi, rho_dot;

	VectorXd y = z - z_pred;
	// angle normalization
	y[1] = atan2(sin(y[1]), cos(y[1]));
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd K = P_ * Ht * Si;

	// new state
	x_ = x_ + (K * y);

	MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
	P_ = (I - K * H_) * P_;
}
