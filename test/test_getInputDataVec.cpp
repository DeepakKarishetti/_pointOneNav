/**
 * @file test_getInputDataVec.cpp
 * @brief test cases for the function 
 * @function twoStateClockModel::getInputData(std::string, std::vector)
 * @note test cases checks for correct input file name and the output generated for the input matrix
 */

#include <iostream>
#include <gtest/gtest.h>

#include "../src/pointOneNav.h"

namespace 
{
	TEST(twoStateClockModel, filter_inputs_1)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("../filter_input/initial_state_estimate.txt", read_file_vec);

		ASSERT_NE(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_2)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("../filter_input/initial_state_estimate_covariance.txt", read_file_vec);

		ASSERT_NE(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_3)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("../filter_input/measurement_noise_covariance.txt", read_file_vec);

		ASSERT_NE(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_4)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("../filter_input/process_noise_covariance.txt", read_file_vec);

		ASSERT_NE(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_5)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("../filter_input/measurement_history.txt", read_file_vec);

		ASSERT_NE(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_6)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("_____initial_state_estimate.txt", read_file_vec);

		EXPECT_EQ(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_7)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("_____initial_state_estimate_covariance.txt", read_file_vec);

		EXPECT_EQ(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_8)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("_____measurement_noise_covariance.txt", read_file_vec);

		EXPECT_EQ(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_9)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("_____process_noise_covariance.txt", read_file_vec);

		EXPECT_EQ(initial_vec_size, read_file_vec.size());
	}

	TEST(twoStateClockModel, filter_inputs_10)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > read_file_vec;
		auto initial_vec_size = read_file_vec.size();
		clockModel->getInputData("_____measurement_history.txt", read_file_vec);

		EXPECT_EQ(initial_vec_size, read_file_vec.size());
	}
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	
	return RUN_ALL_TESTS();
}