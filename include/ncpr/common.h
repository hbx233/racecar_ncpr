/**
 * Global include header file 
 * author: Yuxuan Huang 
 */
#ifndef COMMON_H_
#define COMMON_H_

#include <vector>
#include <memory>
#include <iostream>
#include <string>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <termios.h>
#include <stdio.h>
#include <limits.h>
#include <functional>
using std::vector;
using std::string;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::cout;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::istringstream;
using std::chrono::high_resolution_clock;
//Third Party dependices 
//Eigen 
#include <Eigen/Core>
#include <Eigen/Dense>
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Matrix32d = Eigen::Matrix<double,3,2>;
using Matrix23d = Eigen::Matrix<double,2,3>;
using Eigen::MatrixXd;
using Eigen::VectorXd;
//Sophus
//#include <sophus/so2.h>
//#include <sophus/se2.h>
//using Sophus::SO2;
//using Sophus::SE2;
//OpenCV, just to visualize the extracted line 
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>

#define PI 3.14159265
#define ERROR_MSG(msg) cout<<"[ERROR] "<<msg<<endl
#endif