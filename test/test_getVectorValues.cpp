/**
 * @file test_getVectorValues.cpp
 * @brief getVectorValues write the contents of a matrix to file and read it into a vector for other operations
 * @function twoStateClockModel::getVectorValues(mat, std::string, std::<std::vector<double>>)
 * @note test cases checks for the correct input file name, for the input matrix and the vector container
 */

#include <iostream>
#include <cstdio>
#include <gtest/gtest.h>

#include "../src/pointOneNav.h"

namespace 
{
	TEST(twoStateClockModel, getVectorValues_1)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		mat file_contents = mat::Identity(10,2);
		std::string file_name = "../filter_output/test_writeToFile.txt";
		std::vector<std::vector<double> > init_vec, values_from_file;
		bool get_vec = clockModel->getVectorValues(file_contents, file_name, values_from_file);

		ASSERT_TRUE(get_vec == true);

		if (file_name.size())
		{
			remove(file_name.c_str());
		}
	}

	TEST(twoStateClockModel, getVectorValues_2)
	{
		std::unique_ptr<twoStateClockModel> clockModel(new twoStateClockModel(false, 1));
		mat file_contents = mat::Identity(10,2);
		std::string file_name = "../filter_____output/test_writeToFile.txt";
		std::vector<std::vector<double> > init_vec, values_from_file;
		bool get_vec = clockModel->getVectorValues(file_contents, file_name, values_from_file);

		EXPECT_EQ(get_vec, false);

		if (file_name.size())
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