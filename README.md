# PointOneNav - toyproblem_cpp


## Project structure:
``` The code in this project is organized as shown:```


    .
		 ├── filter_input            # Filter input text files
		 ├── filter_output           # Filter ouputs generated after compiling and executing
		 ├── src                     # Source files
		 ├── test                    # Automated tests
		 ├── CMakeLists.txt          # High-level CMake file for the entire project
		 └── README.md 		     # A readme file for details and instructions	
		 ├── LICENSE
		 └── .gitignore
    

## Code developed platform:
```
	- Ubuntu 16.04
	- C++ 11, gcc version 6.5.0, cmake version 3.5
	- Eigen3
	- Googletest (gtest)
```

## Requirements:
```
	- C++ 11 
	- CMake version 3.5+
	- Eigen3 library
	- Matplotlib-cpp
	- Googletest  
```

### CMake [source](https://cmake.org/)
```
	- CMake version of 3.5+ is recommended

	$ sudo apt-get -y install cmake
```

### Eigen library [source](http://eigen.tuxfamily.org/)
 ```
 	- Installation: Eigen is a header only library, please refer source to use it without CMake

 	- Installing with CMake, as used in this project:

 	$ cd ~/Downloads
 	$ wget -q http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2
 	$ tar xf 3.3.4.tar.bz2
 	$ rm -rf 3.3.4.tar.bz2
 	$ cd eigen-eigen-5a0156e40feb
 	$ mkdir -p build && cd build
 	$ cmake \
			-DCMAKE_BUILD_TYPE=Release \
			-DCMAKE_INSTALL_PREFIX=/usr/local \
			../
		$ make -j4
		$ make install

		- By installing, the project is set to find Eigen library at /usr/include/eigen3 by default, please make necessary changes if not installed at this path.

		$ export CMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH:/<path-to-library>" => CMake would then be able to find it.

 ``` 
> Refer to file "INSTALL" from the tarball if you still have issues.


### Matplotlib-cpp [source](https://github.com/lava/matplotlib-cpp)
 ```
 	- We only need to have the file matplotlibcpp.h from the repo, in the same directory as the main.cpp file.
 	- It is already included in this project in "src" folder, so no further action required
 	- It uses Python, so please do the installation step
 ```

 - Installation:
 	```
 		$ sudo apt-get install python-matplotlib python2.7-dev 
 	```


### Googletest [source](https://github.com/google/googletest)
 ```
 	- A testing and mocking framework developed by google.
 ```

 - Installation:
 	```
 		$ sudo apt-get install libgtest-dev
 		$ sudo apt-get install cmake
 		$ cd /usr/src/gtest
 		$ sudo cmake CMakeLists.txt
 		$ sudo make
		$ sudo cp *.a /usr/lib
 	```


## Source files description:
```
	- main.cpp => top level file that runs the algorithm
	- pointOneNav.h => contains the high level data abstraction with declaration of function prototypes
 	- pointOneNav.hpp => contains the corresponding functions definitions
 	- matplotlibcpp.h => C++ plotting wrapper built on matplotlib
```
- The tests for the corresponding member functions named by their function name are placed under the folder "test"  

## How to build and run:
```
	$ git clone https://github.com/DeepakKarishetti/pointOneNav.git
	$ cd toyProblem_cpp
	$ mkdir build && cd build
	$ cmake ../ 
	$ make && ./bin/ekf 

	$ ./bin/<unit-test-exe> // for unit test results
```

## Outputs:
```
	- The filter outputs generated are:
		- filter_output.txt 
		- Bias_estimate.png => plot between receiver time and clock bias estimate
		- Rate_estimate.png => plot between receiver time and clock rate estimate
		- NIS_filter_residual_plot.png => plot showing the NIS filter residuals 
		
	- All the filter outputs are saved in the folder "filter_output", once the program is executed

	- "bin" folder in build contains all the executables generated, including unit tests
```

## Note:
```
	- All the input and output file paths are hard-coded, changing the file paths will break the code.
	- Running the executable has to be made from the build folder as ./bin/<exe-name> due to above reason.
	- First argument given with the executable file will be set as the dt value in the dynamics function,else 1.
	- The folder "filter_output" is initially empty and is required for running the exe file to get the output.
	- Once the code is compiled and exe run, all the outputs are saved in the folder name "filter_output".
	- All the executable files are saved in the folder "/build/bin".
	- Requires graphics or monitor window connected to display plots using matplotlib, will not work over ssh.
```

> Please let me know if you have any questions or suggestions at: ```r10.deepak@gmail.com```
