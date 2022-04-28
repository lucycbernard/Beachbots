Remember: 
    When creating a new ros node in a package
    1. Add the line "#!/usr/bin/env python" (without the quotes) to the top of the file
    2. in the terminal, in the directory of the file $ chmod +x file.python
    3. in the package's CMakeLists.txt, add the python file in the appropriate area

    When creating a new ros launch file
    1. in the package's CMakeLists.txt, add the launch file in the appropriate area

    Ensure all packages are installed on local machine
    $ sudo apt-get install libopencv-dev 
    $ sudo apt-get install libyaml-cpp-dev
    $ sudo apt-get install libbondcpp-dev
    $ sudo apt-get install libbullet-dev
    $ sudo apt-get install libboost-python-dev
    $ sudo apt-get install python-catkin-pkg

    Note on installing orocos
	$ sudo apt-get install libeigen3-dev libcppunit-dev
	Cd into orocos_kdl
	$ cd <path>/orcos_kdl
	$ mkdir build
	$ cd build
	$ cmake ..
	$ make -j$(nproc)
	$ sudo make install
	$ sudo ldconfig
	cd into python_orocos_kdl
	$ cd ../../python_orocos_kdl
	$ cd pybind11
	$ git submodule init
	$ git submodule update
