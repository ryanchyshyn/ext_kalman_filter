#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class KalmanFilter {
public:

	// state vector
	VectorXd x_;

	// state covariance matrix
	MatrixXd P_;

	// state transition matrix
	MatrixXd F_;

	// process covariance matrix
	MatrixXd Q_;

	// measurement matrix
	MatrixXd H_;

	// measurement covariance matrix
	MatrixXd R_;

public:
	/**
	 * Constructor
	 */
	KalmanFilter();

	/**
	 * Destructor
	 */
	virtual ~KalmanFilter();

	/**
	 * Prediction Predicts the state and the state covariance
	 * using the process model
	 * @param delta_T Time between k and k+1 in s
	 */
	void Predict();

	/**
	 * Updates the state by using standard Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void Update(const VectorXd &z);

	/**
	 * Updates the state by using Extended Kalman Filter equations
	 * @param z The measurement at k+1
	 */
	void UpdateEKF(const VectorXd &z);

};

#endif /* KALMAN_FILTER_H_ */
