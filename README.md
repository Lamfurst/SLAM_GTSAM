# SLAM_GTSAM
This is a project that uses GTSAM to solve Graph SLAM. This program reads G2O file from this Datasets https://lucacarlone.mit.edu/datasets/. The resulted optimized values is exported to CSV file to be later plot in Matlab.

## Quickstart

In the root library folder execute:
```sh
#!bash
mkdir build
cd build
cmake ..
make
```

Go back to the root library folder execute following to build the CSV folder to save the CSV output:
```sh
#!bash
mkdir CSV
```
In the root library folder execute:
```sh
#!bash
cd build
./GTSAM_SLAM
```

The output csv is then saved to the CSV folder you just made.

Prerequisites:

- [Boost](http://www.boost.org/users/download/) >= 1.65 (Ubuntu: `sudo apt-get install libboost-all-dev`)
- [CMake](http://www.cmake.org/cmake/resources/software.html) >= 3.0 (Ubuntu: `sudo apt-get install cmake`)
- A modern compiler, i.e., at least gcc 4.7.3 on Linux.

Optional prerequisites - used automatically if findable by CMake:

- [Intel Threaded Building Blocks (TBB)](http://www.threadingbuildingblocks.org/) (Ubuntu: `sudo apt-get install libtbb-dev`)
- [Intel Math Kernel Library (MKL)](http://software.intel.com/en-us/intel-mkl) (Ubuntu: [installing using APT](https://software.intel.com/en-us/articles/installing-intel-free-libs-and-python-apt-repo))
    - See [INSTALL.md](INSTALL.md) for more installation information
    - Note that MKL may not provide a speedup in all cases. Make sure to benchmark your problem with and without MKL.

## License
This software is released under the MIT License. See the LICENSE file for more information.
