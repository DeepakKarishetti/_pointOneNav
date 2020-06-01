#ifndef POINT_ONE_NAV_H
#define POINT_ONE_NAV_H

/// Std includes
#include <iostream>
#include <fstream>
#include <cstdlib> // std::atoi
#include <memory>  // std::unique_ptr
#include <iomanip> // std::setprecision

/// Eigen includes
#include <Eigen/Dense> 
#include <Eigen/StdVector> 

/**
 * @file pointOneNav.h
 * @brief contains the main class declaration along with the corresponding function prototypes along with struct and typedef declaration
 */

/// typedef declaration for Eigen matrix
typedef Eigen::MatrixXd mat;

/**
 * @brief filter_input struct initialization to hold all the filter input matrices 
 */
struct filter_input
{
	mat initial_state_estimate;
	mat initial_state_estimate_covariance;
	mat measurement_noise_covariance;
	mat process_noise_covariance;
	mat measurement_history;
};

/**
 * @brief The twoStateClockModel is a class used to assist with the EKF estimation algorithm to provide good data abstaction
 */
class twoStateClockModel
{
private:
	/// Check for command line args to update dt value
	float arg_in;
	bool arg_check;

	/// Initialize system defining matrices
	/// State and covariance 
	mat x_kp1 = mat::Zero(2,1);
	mat xcov_kp1 = mat::Zero(2,1);

	/// Measurement prediction and filter gain
	mat zkp1 = mat::Zero(2,1);
	mat skp1 = mat::Zero(2,2);

	/// State propogation and measurement prediction
	mat x_kp1_k;
	mat df_dxk;
	mat df_dwk;
	mat y_kp1_k;
	mat dh_dxkp1;

public:
	/**
	 * @brief twoStateClockModel class constructor, initialize based on the input command line args
	 * @param arg_input boolean value denoting the presence of any input command line argument
	 * @param arg_val input value to be set for dt
	 */
	twoStateClockModel(bool arg_input, float arg_val);	

	/**
	 * @brief f two-state clock model dynamics function
	 * @param x_k a posteriori state at the last time step
	 * @param w_k a prior estimate of the process noise
	 */
	void f(const mat x_k, const mat w_k);

	/**
	 * @brief h two-state clock model measurement function
	 * @param x_kp1_k a posteriori state for propogating state
	 * @param v_kp1 initial estimate of the process noise
	 */
	void h(const mat x_kp1_k, const mat v_kp1);

	/**
	 * @brief simpleEKF Extended Kalman Filter implementation
	 * @param xk_est a posteriori state at the previous time step k
	 * @param xcovk_est a posteriori state covariance estimate at k
	 * @param w_k a priori estimate of process noise  at k to k+1
	 * @param clock_noise_cov a priori process noise covariance estimate of w_k
	 * @param y_kp1 measurement vector at time k+1
	 * @param v_kp1 measurement observed at time k
	 * @param measurement_cov a priori estimate of the measurement noise covariance to y_kp1
	 */
	void simpleEKF(const mat& xk_est, const mat xcovk_est, const mat w_k, const mat clock_noise_cov, const mat y_kp1, 
					const mat v_kp1, const mat measurement_cov);

	/**
	 * @brief get_xkp1 getter function to obtain class member
	 * @return x_kp1 matrix
	 */
	mat get_xkp1();

	/**
	 * @brief get_zkp1 getter function to obtain class member
	 * @return zkp1 matrix
	 */
	mat get_zkp1();

	/**
	 * @brief get_skp1 getter function to obtain class member
	 * @return skp1 matrix
	 */
	mat get_skp1();

	/**
	 * @brief get_xcov_kp1 getter function to obtain class member
	 * @return xcov_kp1 matrix
	 */
	mat get_xcov_kp1();

	/**
	 * @brief getInputData function used to extract the contents of a file into a matrix
	 * @param file_name the file name, extract input values from
	 * @param input_mat matrix to hold the data from the input file
	 */
	void getInputData(const std::string file_name, mat& input_mat);

	/**
	 * @brief getInputData function used to extract the contents of a file into a vector
	 * @param file_name the file name, extract values from
	 * @param values_from_file vector to hold the data from the input file
	 */
	void getInputData(const std::string file_name, std::vector<std::vector<double> >& values_from_file);

	/**
	 * @brief readFromFile function to read in the contents of an input file
	 * @param file_name the file name, read the values from
	 * @param values_from_file vector to hold the data from the input 
	 * @return boolean value
	 */
	bool readFromFile(const std::string file_name, std::vector<std::vector<double> >& values_from_file);

	/**
	 * @brief checkInputs function to check if all the filter inputs are read in correctly
	 * @param struct that holds all the filter input matrices
	 * @return boolean value
	 */
	bool checkInputs(struct filter_input filter_inputs);

	/**
	 * @brief writeToFile function to write the output from the process to a text file
	 * @param file_name the file name, to write the output to
	 * @param output matrix from which the data is written to file
	 * @return boolean value
	 */
	bool writeToFile(const std::string file_name, const mat output);

	/**
	 * @brief getVectorValues write the contents of a matrix to file and read it into a vector for other operations
	 * @param mat_in matrix from which the data is written to file
	 * @param file_name the file name, to write the output to
	 * @param output_values vector to hold the data from the file
	 * @return boolean value
	 */
	bool getVectorValues(const mat mat_in, const std::string file_name, std::vector<std::vector<double> >& output_values);

	/**
	 * @brief matPlot function to plot the filter output against custom data
	 * @param x vector containing values to be plotted along the x axis
	 * @param y vector containing values to be plotted along the y axis
	 * @param caption the title of the plot
	 * @param x_label the x axis label on the plot
	 * @param y_label the y axis label on the plot
	 * @param gamma_high chi-squared inverse distibution with 2DOF for the given alpha value
	 */
	void matPlot(const std::vector<double> x, const std::vector<double> y, std::string caption, const std::string x_label, const std::string y_label, const float gamma_high);


};

#include "pointOneNav.hpp"
#endif // POINT_ONE_NAV_H