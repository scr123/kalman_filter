# Kalman Filter

Kalman Filter is a library for the Kalman (KF), Extended Kalman (EKF), and Unscented Kalman (UKF) filters.


## Release Notes
* Completed:
    * Kalman Filter Base Class
    * Extended Kalman Filter Derived Class
    * Utilities for measurement conversions
    * Unit test framework
* TODO:
    * Complete Unscented Kalman Filter Derived Class
    * Include API to support use of Jacobians for non-linear transforms:
        * **Update(const vec_t& _z, const mat_t& _H, const mat_t& _R)** - add parameter ***const vec_t& _x*** with default value being internal class state vector (added parameter will be used to pass state back when dealing with non-Cartesian measurements)
        * Provide Jacobian utility specifically for radar measurements (alternatively a symbolic differentiator can be used to calculate the Jacobian for measurment error linearization)
    * Complete tests for filters, providing examples where all final state estimates are within allowed error (involves tuning of Kalman filter with specific sensors/data set used)

## Getting Started
These instructions will help you generate the necessary documentation for using this package, and list the required dependencies.

### Documentation

The documentation for this project is Doxygen based. To generate, execute the following commands:

```
cd <path>/kalman_filter
doxygen Doxyfile
```

### Dependencies

The follwing dependencies are required, and can be installed accordingly.

```
sudo apt install doxygen
sudo apt install libgtest-dev
sudo apt install build-essential
sudo apt install python-catkin-tools
sudo apt install ros-melodic-desktop-full (Includes required Eigen3 library)

```
## Running the tests

To compile unit and pipeline tests, use the following command:
```
catkin build kalman_filter --no-deps --catkin-make-args run_tests
```

### Break down into end to end tests

The Kalman Filter test verifies basic functionality of the Kalman Filter base, and derived classes.   Test demonstrates construction of object, running of update and prediction steps, and extraction of state components. Data set used for testing is from Udacity's Self-Driving Car Nano Degree, and can be found [here](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project/tree/master/data).

```
kf_test.cpp 
```

The utils unit test validates measurment conversions.
```
utils_test.cpp
```

## Built With

* [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html) - Build tool used for compiling this project
* [Google Test](https://github.com/google/googletest) - Unit testing framework
* [ros_melodic](http://wiki.ros.org/melodic) - Open source meta-operating system


## Authors

* **Sean Crutchlow**

## License

This project is licensed under the MIT License - see the LICENSE file for details
