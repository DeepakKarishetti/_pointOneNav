/**
 * @file test_readFromFile.cpp
 * @brief function to read in the contents of an input file
 * @function twoStateClockModel::readFromFile(std::string, std::vector<std::vector<double>>)
 * @note test cases check for all the correct and incorrect filter input text files
 */

#include <iostream>
#include <gtest/gtest.h>

#include "../src/pointOneNav.h"

namespace 
{
	TEST(twoStateClockModel, readFromFile_1)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("../filter_input/initial_state_estimate.txt", file_contents);

		ASSERT_TRUE(file_read == true);
	}

	TEST(twoStateClockModel, readFromFile_2)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("../filter_input/initial_state_estimate_covariance.txt", file_contents);

		ASSERT_TRUE(file_read == true);
	}

	TEST(twoStateClockModel, readFromFile_3)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("../filter_input/measurement_noise_covariance.txt", file_contents);

		ASSERT_TRUE(file_read == true);
	}

	TEST(twoStateClockModel, readFromFile_4)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("../filter_input/process_noise_covariance.txt", file_contents);

		ASSERT_TRUE(file_read == true);
	}

	TEST(twoStateClockModel, readFromFile_5)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("../filter_input/measurement_history.txt", file_contents);

		ASSERT_TRUE(file_read == true);
	}

	TEST(twoStateClockModel, readFromFile_6)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("_____initial_state_estimate.txt", file_contents);

		EXPECT_TRUE(file_read != true);
	}

	TEST(twoStateClockModel, readFromFile_7)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("_____initial_state_estimate_covariance.txt", file_contents);

		EXPECT_TRUE(file_read != true);
	}

	TEST(twoStateClockModel, readFromFile_8)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("_____measurement_noise_covariance.txt", file_contents);

		EXPECT_TRUE(file_read != true);
	}

	TEST(twoStateClockModel, readFromFile_9)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("_____process_noise_covariance.txt", file_contents);

		EXPECT_TRUE(file_read != true);
	}

	TEST(twoStateClockModel, readFromFile_10)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		std::vector<std::vector<double> > file_contents;
		bool file_read = clockModel->readFromFile("_____measurement_history.txt", file_contents);

		EXPECT_TRUE(file_read != true);
	}
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}