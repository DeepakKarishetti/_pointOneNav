/**
 * @file test_writeToFile.cpp
 * @brief function to write the output matrix from the process to a text file
 * @function twoStateClockModel::writeToFile(std::string, mat)
 * @note test cases are checked for the function outputs for the correct input filename and matrix
 */

#include <iostream>
#include <cstdio>
#include <gtest/gtest.h>

#include "../src/pointOneNav.h"

namespace 
{
	TEST(twoStateClockModel, writeToFile_1)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		mat file_contents = mat::Identity(10,2);
		std::string file_name = "../filter_output/test_writeToFile.txt";
		bool file_read = clockModel->writeToFile(file_name, file_contents);

		ASSERT_TRUE(file_read == true);

		if (file_read)
		{
			remove(file_name.c_str());
		}
	}

	TEST(twoStateClockModel, writeToFile_2)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		mat file_contents = mat::Identity(10,2);
		std::string file_name = "../filter____output/test_writeToFile.txt";
		bool file_read = clockModel->writeToFile(file_name, file_contents);

		ASSERT_FALSE(file_read == true);

		if (file_read)
		{
			remove(file_name.c_str());
		}
	}
}

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	
	return RUN_ALL_TESTS();
}