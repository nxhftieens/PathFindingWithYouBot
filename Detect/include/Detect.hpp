#ifndef DETECT_HEADER_HPP
#define DETECT_HEADER_HPP

#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "../../GridMap/include/Element.hpp"

using namespace cv;
using namespace std;

void calibrateCameraCustom(Mat& cameraMatrix, Mat& distCoeffs);
void generateChessboardImage(const int& rows, const int& cols, const int& squareSize, const string& filename);
Vec3d rotationMatrixToEulerAngles(Mat& R);

#endif // DETECT_HEADER_HPP