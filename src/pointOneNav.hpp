#ifndef POINT_ONE_NAV_HPP
#define POINT_ONE_NAV_HPP

#include "pointOneNav.h"
#include "matplotlibcpp.h"

/**
 * @file pointOneNav.hpp
 * @brief contains all the class member function definitions declared in .h file 
 */

/// class constructor, makes a check to command line args to set dt value
twoStateClockModel::twoStateClockModel(bool arg_input, float arg_val)
{
	this->arg_check = arg_input;
	if (arg_input)
	{
		this->arg_in = arg_val;
	}
	else
	{
		this->arg_in = 1.0; 
	}
}

/// twoStateClockModel dynamics function
void twoStateClockModel::f(const mat x_k, const mat w_k)
{	
	mat a = mat::Identity(2,2);
	a(0,1) = this->arg_in;		

	mat b = mat::Identity(2,2);

	this->x_kp1_k = a * x_k + b * w_k;
	this->df_dxk = a;
	this->df_dwk = b;
}

/// twoStateClockModel measurement function
void twoStateClockModel::h(const mat x_kp1_k, const mat v_kp1)
{
	this->y_kp1_k = this->x_kp1_k + v_kp1;
	this->dh_dxkp1 = mat::Identity(2,2);
}

/// Extended Kalman Filter algorithm to update state and covariance
void twoStateClockModel::simpleEKF(const mat& xk_est, const mat xcovk_est, const mat w_k, const mat clock_noise_cov, 
									const mat y_kp1, const mat v_kp1, const mat measurement_cov)
{
	/// Propogate the states
	f(xk_est, w_k);

	/// Predict the measurements
	h(this->x_kp1_k, v_kp1);
	this->zkp1 = y_kp1 - this->y_kp1_k;

	/// Propogate the covariance
	mat p_kp1_k = this->df_dxk  * xcovk_est * this->df_dxk.transpose() + this->df_dwk * clock_noise_cov * this->df_dwk.transpose();

	/// Compute filter gain and k_kp1
	skp1 = this->dh_dxkp1  * p_kp1_k * this->dh_dxkp1.transpose() + measurement_cov;
	mat k_kp1 = p_kp1_k * (this->dh_dxkp1.transpose() * skp1.inverse());

	/// Compute the a posteriori state estimate
	this->x_kp1 = this->x_kp1_k + k_kp1 * this->zkp1;

	/// Compute the a posteriori covariance estimate
	int n_x = xk_est.rows();
	mat imwh = mat::Identity(2,2) - k_kp1 * this->dh_dxkp1;

	/// Joseph form to enforce positive semi-definiteness of the covariance matrix
	xcov_kp1 = imwh * p_kp1_k * imwh.transpose() + k_kp1 * measurement_cov * k_kp1.transpose();
}

/// Return the a posteriori state estimate
mat twoStateClockModel::get_xkp1()
{
	return this->x_kp1;
}

/// Return the predicted measurements
mat twoStateClockModel::get_zkp1()
{
	return this->zkp1;
}

/// Return the filter gain
mat twoStateClockModel::get_skp1()
{
	return this->skp1;
}

/// Return the covariance matrix
mat twoStateClockModel::get_xcov_kp1()
{
	return this->xcov_kp1;
}

/// *********************
//! FILE I/O OPERATIONS |
/// *********************

/// Read in the contents of the input text file  
bool twoStateClockModel::readFromFile(const std::string file_name, std::vector<std::vector<double> >& values_from_file)
{
	std::ifstream file_in;
	std::vector<std::string> data;

	file_in.open(file_name);
	std::string line;

	/// Add lines from input file into a vector, if file is open
	if (file_in.is_open())
	{
		while (std::getline(file_in, line))
		{
			data.push_back(line);
		}
		file_in.close();
	}
	else
	{
		std::cout << "Error opening the file " << file_name << ", ";
		return false;
	}
	
	/// From the vector of strings, extract the values, convert it to type double and add into the given vector
	std::vector<std::string>::iterator it; /// vector iterator
	for (it = data.begin(); it != data.end(); it++)
	{
		std::stringstream file_ss;
		file_ss << *it;
		std::string temp_str;
		double temp_val;
		std::vector<double> value_file;

		while (!file_ss.eof())
		{
			file_ss >> temp_str;
			if (std::stringstream(temp_str) >> temp_val)
			{
				/// Just to visualize the decimal precision of the input values, as given in input files
				std::cout.setstate(std::ios_base::failbit);
				std::cout << std::setprecision(18) << temp_val << std::endl;
				std::cout.clear();
				value_file.push_back(temp_val);
			}
			temp_str.clear();
		}
		values_from_file.push_back(value_file);
	}
	return true;
}

/// Insert the data from the input file to the given input matrix, if reading in file is successful
void twoStateClockModel::getInputData(const std::string file_name, mat& input_mat)
{
	/// vector to hold the values from the file
	std::vector<std::vector<double> > values_from_file;
	
	/// Location of the filter input files
	std::stringstream file_input;
	file_input << "../filter_input/" << file_name;

	/// Read in and add to the input matrix
	if (readFromFile(file_input.str(), values_from_file))
	{
		int rows = values_from_file.size();
		int cols = values_from_file[0].size();

		mat temp(rows, cols);
		for (int i=0; i<rows; ++i)
		{
			for (int j=0; j<cols; ++j)
			{
				temp(i,j) = values_from_file[i][j];
			}
		}
		input_mat = temp;
	}
	else
	{
		std::cerr << " check the input file!" << std::endl;
	}
}

/// If reading from file is successful, insert the data from the input file to the given input vector
void twoStateClockModel::getInputData(const std::string file_name, std::vector<std::vector<double> >& values_from_file)
{
	if (readFromFile(file_name, values_from_file))
	{
		return;
	}
	else
	{
		std::cerr << " check the input file!" << std::endl;
	}
}

/// Check if all the filter inputs files are read in correctly
bool twoStateClockModel::checkInputs(struct filter_input f_in)
{
	if (f_in.initial_state_estimate.size() > 0 and f_in.initial_state_estimate_covariance.size() > 0 and 
		f_in.measurement_noise_covariance.size() > 0 and f_in.process_noise_covariance.size() > 0 and 
		f_in.measurement_history.size() > 0)
	{
		return true;
	}
	return false;
}

/// Write the contents of a given matrix to a text file
bool twoStateClockModel::writeToFile(const std::string file_name, const mat output)
{
	std::stringstream ss;
	ss << "../filter_output/" << file_name;

	std::ofstream file(ss.str());
	if (file.is_open())
	{
		/// Decimal precision as given in the filter output provided
		file << std::setprecision(16) << std::scientific << output;
		file.close();
		return true;
	}

	std::cerr << "Error writing output to file, " << std::endl;
	return false;
}

/// If writing to file is successful, the data are then read and added into the input vector container
bool twoStateClockModel::getVectorValues(const mat mat_in, const std::string file_name, std::vector<std::vector<double> >& output_values)
{
	if (writeToFile(file_name, mat_in))
	{
		/// read from the output folder
		std::stringstream ss;
		ss << "../filter_output/" << file_name;
		getInputData(ss.str(), output_values);
		return true;
	}

	std::cerr << " no values added into the vector!" << std::endl;
	return false;
}

/// Plot the values using the given input vectors with the plot title and axis labels
void twoStateClockModel::matPlot(const std::vector<double> x, const std::vector<double> y, std::string caption, const std::string x_label, const std::string y_label, const float gamma_high)
{
	/// Make sure the input vector with values to be plotted has the same number of elements as y
	if (x.size() == y.size())
	{
		/// Gamma_high value obtained from inverse chi-squared probability distribution with 2 degrees of freedom, evaluated at the value (1-alpha)=0.95
		if (gamma_high != 0.0)
		{
			auto counter = 0;
			for (auto it=x.begin(); it!=x.end(); it++)
			{
				if ((*it) > gamma_high)
				{
					counter++;
				}
			}
			auto nis = counter / 1000.0;
			auto alpha = 0.05;

			/// Caption for this plot
			std::stringstream ss;
			ss << "NIS, " << nis << " above the " << alpha << " threshold";
			caption = ss.str();

			std::vector<double> w(y.size(),gamma_high);
			matplotlibcpp::plot(y, w, "r-");
		}

		matplotlibcpp::title(caption);
		matplotlibcpp::plot(y, x);	
		matplotlibcpp::xlabel(x_label);
		matplotlibcpp::ylabel(y_label);

		/// Save plot to file in the given path
		std::stringstream ss;
		ss << "../filter_output/" << caption << ".png";
		matplotlibcpp::save(ss.str());

		/// Show plot
		matplotlibcpp::show();
		matplotlibcpp::close();
	}
	else
	{
		std::cerr << "Error plotting, vectors must be of the same length!" << std::endl;
	}
}

#endif // POINT_ONE_NAV_HPP