/**
 * @file test_getInputData.cpp
 * @brief test cases for the function 
 * @function twoStateClockModel::getInputData(std::string, mat)
 * @note test cases checks for correct input file name and the output generated for the input matrix
 */

#include <iostream>
#include <gtest/gtest.h>

#include "../src/pointOneNav.h"

namespace 
{
	TEST(twoStateClockModel, filter_inputs_1)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("../filter_input/initial_state_estimate.txt", f_in);

		ASSERT_NE(temp.size(), f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_2)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("../filter_input/initial_state_estimate_covariance.txt", f_in);

		ASSERT_NE(temp.size(), f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_3)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("../filter_input/measurement_noise_covariance.txt", f_in);

		ASSERT_NE(temp.size(), f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_4)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("../filter_input/process_noise_covariance.txt", f_in);
		
		ASSERT_NE(temp.size(), f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_5)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("../filter_input/measurement_history.txt", f_in);

		ASSERT_NE(temp.size(), f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_6)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("_____initial_state_estimate.txt", f_in);

		ASSERT_TRUE(temp.size() == f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_7)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", f_in);

		ASSERT_TRUE(temp.size() == f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_8)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("_____measurement_noise_covariance.txt", f_in);

		ASSERT_TRUE(temp.size() == f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_9)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("_____process_noise_covariance.txt", f_in);

		ASSERT_TRUE(temp.size() == f_in.size());
	}

	TEST(twoStateClockModel, filter_inputs_10)
	{
		mat f_in, temp;

		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		clockModel->getInputData("_____measurement_history.txt", f_in);

		ASSERT_TRUE(temp.size() == f_in.size());
	}
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	
	return RUN_ALL_TESTS();
}