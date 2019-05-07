# IMU Preintegration

**Current version:** 1.0.0 

IMU Preintegration is a real-time module library. It is able to use for VIO and VI SLAM. We provide two non-linear optimizations libraries for estimating the states. We also provide the examples to run this library and show how to use this IMU Preintegration module library.

This library is not bug-free, and welcome to improve it together!


## Related Publications:

* Forster C, Carlone L, Dellaert F, et al. **On-Manifold Preintegration for Real-Time Visual--Inertial Odometry**. IEEE Transactions on Robotics, 2017, 33(1): 1-21. **[PDF](http://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)**.

* Tong Qin, Peiliang Li, Zhenfei Yang, Shaojie Shen, **VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator**,  IEEE Transactions on Robotics. [PDF](https://ieeexplore.ieee.org/document/8421746/?arnumber=8421746&source=authoralert) 


## 1. License
IMU Preintegration is released under a [GPLv3 license](https://github.com/raulmur/ORB_SLAM2/blob/master/License-gpl.txt). For a list of all code/library dependencies (and associated licenses), please see [Dependencies.md](https://github.com/raulmur/ORB_SLAM2/blob/master/Dependencies.md).


## 2. Prerequisites
We have tested the library in **Ubuntu 14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

### C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

### BLAS and LAPACK
[BLAS](http://www.netlib.org/blas) and [LAPACK](http://www.netlib.org/lapack) libraries are requiered by g2o (see below). On ubuntu:
```
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
```

### g2o (Included in Thirdparty folder)
We use modified versions of the [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. The modified libraries (which are BSD) are included in the *Thirdparty* folder.

### ceres
We also use [ceres](https://github.com/ceres-solver/ceres-solver) library to perform non-linear optimizations. 


## 3. Building IMU Preintegration library

Clone the repository:
```
git clone git@github.com:mc275/IMU_Preintegration.git 
```

We provide a script `build.sh` to build the *Thirdparty* libraries and *IMU Preintegration*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd IMU_Preintegration
chmod +x build.sh
./build.sh
```

This will create **libIMU_Preintegration.so**  at *lib* folder and the executables **main** in *bin* folder.

## 4. Run Examples

### EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```

```

```

```

## 5. Processing your own sequences
You can refer to the examples for IMU Preintegration and call this library API for your own datasets.
