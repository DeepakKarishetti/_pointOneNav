/**
 * @file test_checkInputs.cpp
 * @brief function to check if all the filter inputs are read in correctly
 * @function twoStateClockModel::checkInputs(struct)
 * @note test cases are checked for the function outputs for the correct and incorrect filter input files
 */

#include <iostream>
#include <cstdio>
#include <gtest/gtest.h>

#include "../src/pointOneNav.h"

namespace 
{
	TEST(twoStateClockModel, checkInputs_1)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_EQ(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_2)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("_____initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_3)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_4)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_5)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("_____process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_6)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("_____measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_7)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("_____initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_8)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_9)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("_____process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_10)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("_____process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("_____measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_11)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("_____initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_12)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("_____process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_13)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("_____process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("_____measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_14)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("_____initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("_____process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_15)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("_____process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("_____measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}

	TEST(twoStateClockModel, checkInputs_16)
	{
		filter_input f_in;
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));

		clockModel->getInputData("_____initial_state_estimate.txt", f_in.initial_state_estimate);
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in.initial_state_estimate_covariance);
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in.measurement_noise_covariance);
		clockModel->getInputData("_____process_noise_covariance.txt", f_in.process_noise_covariance);
		clockModel->getInputData("_____measurement_history.txt", f_in.measurement_history);

		bool filter_inputs = clockModel->checkInputs(f_in);
		ASSERT_NE(filter_inputs, true);
	}
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}