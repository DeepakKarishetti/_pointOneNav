/**
 * @file main.cpp
 * @brief main source file that runs the system containing the recursive Extended Kalman Filter algorithm
 * @date May 29, 2020
 * @author Deepak R Karishetti
 *
 *
 * @note
 *
 * - File descriptions:   
 *	 - main.cpp - top level file that runs the algorithm
 *	 - pointOneNav.h - contains the high level data abstraction with declaration of function prototypes
 *	 - pointOneNav.hpp - contains the corresponding functions definitions
 *	 - matplotlibcpp.h - C++ plotting library built on matplotlib; Source: https://github.com/lava/matplotlib-cpp
 *
 *
 * - Algorithm parameters:
 *	 - EKF - estimate the offset and the drift rate of a clock
 *   
 *	 - The state:
 *	   - Clock error (seconds * C) 
 *	   - Clock frequency error (seconds/second * C)
 *
 *	 - The measurements:
 *	   - Instantaneous clock error (seconds * C)
 *	   - Clock frequency error (seconds/second * C)
 *	   (C being the speed of light)
 *
 *
 * - EKF inputs:
 *   - filter_input/initial_state_estimate.txt - 2x1 the initial state estimates [clock error, clock frequency error]
 *   - filter_input/initial_state_estimate_covariance.txt - 2x2 initial state estimate covariance matrix
 *   - filter_input/measurement_history.txt - 2x1000 measurements observed 
 *   - filter_input/measurement_noise_covariance.txt - 2x2 measurement noise covariance matrix
 *   - filter_input/process_noise_covariance.txt - 2x2 process noise covariance matrix
 *
 *
 * - Outputs:
 *   - filter_output.txt - the algorithm output containing the estimate of offset and drift rate of a clock
 *   - plots - bias estimate plot, rate estimate plot and the NIS filter residual plots 
 *
 ******************************************************************************************************************* */

/// Main header include
#include "pointOneNav.hpp"

/// ***************
//! MAIN FUNCTION |
/// ***************

/** 
 * @brief main
 * @return int
 */
int main(int argc, char** argv)
{
	/// Initialize struct that holds the process filter inputs matrices
	filter_input f_in;

	/// Assigning dt value based on command line args; Implicitly uses dt = 1;
	bool dt_input = false;
	auto dt_val = 1;
	if (argc == 2)
	{
		dt_input = true;
		dt_val = std::atoi(argv[1]);
	}

	/// Unique pointer to initialize the twoStateClockModel class pointer
	std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(dt_input, dt_val));

  	/// read in the process filter inputs from given text files
	clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
	clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
	clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
	clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
	clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

	/// Check to make sure all the filter input files are loaded in
	if (clockModel->checkInputs(f_in))
	{
		/// ***************************************************************
		//! CREATE AND INITIALIZE SYSTEM DEFINING STRUCTURES AND MATRICES |
		/// ***************************************************************
		int n = f_in.measurement_history.cols();

		mat clock_noise_cov = f_in.process_noise_covariance;
		mat measurement_cov = f_in.measurement_noise_covariance;
		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		
		mat w_k = mat::Zero(2,1);
		mat v_kp1 = mat::Zero(2,1); 

		/// Allocate storage for output matrices
		mat xhat_hist = mat::Zero(2,n+1);
		mat nis_hist = mat::Zero(1, n+1);
		std::vector<Eigen::Matrix<double,2,2> > xcov_hist = std::vector<Eigen::Matrix<double,2,2> >(n+1);

		/// Store the initial process filter inputs
		xhat_hist.col(0) = f_in.initial_state_estimate;
		for (auto it=xcov_hist.begin(); it!=xcov_hist.end(); it++)
		{
			(*it).setZero();
		}	
		xcov_hist[0] = f_in.initial_state_estimate_covariance;

		/// Main algorithm that performs estimation using EKF
		for (auto kp1=0; kp1<n; kp1++)
		{
			/// Measurement vector 
			mat y_kp1 = f_in.measurement_history.col(kp1);

			/// Extended Kalman Filter algorithm to update states and covariance 
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);

			/// Assign the filter algorithm output 
			xhat_hist.col(kp1+1) = clockModel->get_xkp1();
			xcov_hist.at(kp1+1) = clockModel->get_xcov_kp1();
			mat temp = (clockModel->get_zkp1().transpose()) * ((clockModel->get_skp1()).inverse() * clockModel->get_zkp1());
			nis_hist(0, kp1+1) = temp(0); 

			/// Passed on to the next loop iteration		
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();	
		}

		/// Write the output values to file; read them and add into vector containers for plotting
		std::vector<std::vector<double> > xhat_hist_values, nis_hist_values;
		clockModel->getVectorValues(xhat_hist, "filter_output.txt", xhat_hist_values);
		clockModel->getVectorValues(nis_hist, "nis_hist.txt", nis_hist_values);

		/// Generate y values, running from 0 to size of output vectors
		std::vector<double> y(n+1);
		std::iota(std::begin(y), std::end(y), 0);

		/// Chi-squared inverse cumulative distribution with 2 degrees of freedom with alpha = 0.05; gamma_high = chi2inv(1-alpha)
		float gamma_high = 5.9915;	

		/// Generate, show and save the plots of the filter outputs using matplotlib-cpp library 
		clockModel->matPlot(xhat_hist_values[0], y, "Bias_estimate", "Receiver time, s", "Clock bias estimate, m", 0.0);
		clockModel->matPlot(xhat_hist_values[1], y, "Rate_estimate", "Receiver time, s", "Clock rate estimate, m/s", 0.0);
		clockModel->matPlot(nis_hist_values[0], y, "NIS_filter_residual_plot", " ", " ", gamma_high);
	}
	else
	{
		std::cerr << "Error getting the filter input files, check the input file names and path!" << std::endl;
		return -1;
	}

	return 0;
}