#ifndef MAPFROMCAM_HEADER_HPP
#define MAPFROMCAM_HEADER_HPP

#include <iostream>
#include <string>
#include <vector>
#include <filesystem>
#include <cmath>

#include <utility>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "../../GridMap/include/Element.hpp"

using namespace cv;
using namespace std;

// Define possible obstacle shapes
enum class ObstacleShape {
    Rectangle,
    Circle,
    UShape,
    None
};

// Structure to hold marker information
struct MarkerInfo {
    int id;
    double x;
    double y;
    double rotation; // Rotation angle in degrees relative to marker ID = 0
    CellType type;
    ObstacleShape shape; // Only relevant for obstacles
};

pair<int, int> rotatePoint(const int& x, const int& y, const double& angle);

void addRectangle(vector<vector<CellType>>& gridMap, 
    const int& centerX, const int& centerY, const int&  width, const int& height, 
    const double& angle, const double& resolution, const double& minX, const double& minY);

void addCircle(vector<vector<CellType>>& gridMap,
    const int& centerX, const int& centerY, const int& radius,
    const double& resolution, const double& minX, const double& minY);

void addUShape(vector<vector<CellType>>& gridMap,
    const int& centerX, const int& centerY, const int& width, const int& height,
    const double& angle, const double& resolution, const double& minX, const double& minY);

vector<vector<CellType>> mapFromImage(const string& imgPath, const string& dirForCalibration);

void calibrateCameraFromImages(const string& dir, Mat& cameraMatrix, Mat& distCoeffs);

Vec3d rotationMatrixToEulerAngles(Mat& R);

void printGridMap(const vector<vector<CellType>>& gridMap);

void showGridMap(const std::vector<std::vector<CellType>>& gridMap, int cellSize);

#endif // !MAPFROMCAM_HEADER_HPP
