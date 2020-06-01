/**
 * @file test_simpleEKF.cpp
 * @brief performs the simple Extended Kalman Filter implementation
 * @function twoStateClockModel::simpleEKF(mat, mat, mat, mat, mat, mat, mat)
 * @note all the test cases check for the internal changes with the state matrix and the covariance matrix with respect to the filter updates
 * @note this performs end-to-end testing of the overall EKF algorithm implementation
 * @note the changing matrices with comparisions shows the correct propogation of state and covariance with each iterations with correct inputs
 * @note checked at an increment of 50 iterations to validate the correctness of the entire algorithmic process
 */

#include <iostream>
#include <gtest/gtest.h>

#include "../src/pointOneNav.h"

namespace 
{
	TEST(twoStateClockModel, simpleEKF_1)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<50; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_2)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<100; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_3)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<150; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_4)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<200; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_5)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<250; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_6)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<300; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_7)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<350; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_8)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<400; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_9)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<450; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_10)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<500; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_11)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<550; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_12)
	{ 
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<600; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_13)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<650; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_14)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<700; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_15)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<750; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_16)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<800; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_17)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<850; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_18)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<900; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_19)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<950; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}

	TEST(twoStateClockModel, simpleEKF_20)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		mat xk_est = f_in.initial_state_estimate;
		mat xcovk_est = f_in.initial_state_estimate_covariance;
		mat w_k = mat::Zero(2,1);
		mat clock_noise_cov = f_in.process_noise_covariance;
		mat v_kp1 = mat::Zero(2,1); 
		mat measurement_cov = f_in.measurement_noise_covariance;

		mat temp_2 = xcovk_est;
		for (auto i=0; i<1000; i++)
		{
			mat y_kp1 = f_in.measurement_history.col(i);
			mat temp_1 = xk_est;
			clockModel->simpleEKF(xk_est, xcovk_est, w_k, clock_noise_cov, y_kp1, v_kp1, measurement_cov);
			xk_est = clockModel->get_xkp1();
			xcovk_est = clockModel->get_xcov_kp1();
			EXPECT_FALSE(temp_1.norm() == xk_est.norm());
		}
		EXPECT_FALSE(temp_2.sum() == xcovk_est.sum());
	}
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	
	return RUN_ALL_TESTS();
}