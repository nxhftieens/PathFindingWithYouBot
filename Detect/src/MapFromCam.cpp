#include "../include/MapFromCam.hpp"

#define CALIB_PARAMS_FILE std::string(PROJECT_DIR) + "/Detect/calibParams.xml"

// Main function to execute the mapping and print the grid
//int main() {
//	/*if (argc < 3) {int argc, char** argv
//		cout << "Usage: " << argv[0] << " <image_path> <calibration_directory>" << endl;
//		return -1;
//	}*/
//
//	/*string imgPath = argv[1];
//	string dirForCalibration = argv[2];*/
//
//	string imgPath = "D:/SourceCode/PathFindingWithYouBot/Detect/asd.png";
//	string dirForCalibration = "D:/SourceCode/PathFindingWithYouBot/Detect/CamCalib";
//
//	vector<vector<CellType>> gridMap = mapFromImage(imgPath, dirForCalibration);
//
//	if (gridMap.empty()) {
//		cout << "Failed to generate gridMap." << endl;
//		return -1;
//	}
//
//	// Print the gridMap to console
//	printGridMap(gridMap);
//
//	// Display the gridMap using OpenCV
//	showGridMap(gridMap, 5); // You can adjust cellSize as needed
//
//	return 0;
//}

vector<vector<CellType>> mapFromImage(const string& imgPath, const string& dirForCalibration, const bool& calibrate = true)
{
	vector<vector<CellType>> gridMap;

	Mat cameraMatrix, distCoeffs;
	const float markerLength = 0.19f;

	std::cout << "Calibration file path: " << CALIB_PARAMS_FILE << std::endl;

	if (calibrate) {
		// Get camera matrix and distortion coefficients
		calibrateCameraFromImages(dirForCalibration, cameraMatrix, distCoeffs);
	}
	else {
		// Load camera matrix and distortion coefficients from file
		FileStorage fs(CALIB_PARAMS_FILE, FileStorage::READ);
		if (!fs.isOpened()) {
			cout << "Failed to open calibration file." << endl;
			return gridMap;
		}
		fs["cameraMatrix"] >> cameraMatrix;
		fs["distCoeffs"] >> distCoeffs;
		fs.release();
	}

	aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
	aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);
	aruco::ArucoDetector detector(dictionary, detectorParams);

	Mat objPoints(4, 1, CV_32FC3);
	objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0.0);
	objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0.0);
	objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0.0);
	objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0.0);

	Mat mapImage = imread(imgPath);
	if (mapImage.empty())
	{
		cout << "Failed to load image: " << imgPath << endl;
		return gridMap;
	}

	vector<int> ids;
	vector<vector<Point2f>> corners, rejected;
	detector.detectMarkers(mapImage, corners, ids, rejected);

	size_t nMarkers = corners.size();
	vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

	if (!ids.empty()) {
		// Calculate pose for each marker
		for (size_t i = 0; i < nMarkers; i++) {
			solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
		}

		int idx0 = -1;
		for (size_t i = 0; i < ids.size(); i++) {
			if (ids[i] == 0) { // Marker ID 0 is the reference
				idx0 = i;
				break;
			}
		}

		//Marker with ID = 0 is marked as the origin of the grid map, located at upper left corner

		if (idx0 >= 0) {
			// Marker ID 0 detected
			Mat R0;
			Rodrigues(rvecs[idx0], R0);
			Mat T0 = Mat::eye(4, 4, CV_64F);
			R0.copyTo(T0(Range(0, 3), Range(0, 3)));
			Mat t0 = Mat(tvecs[idx0]).reshape(1, 3);
			t0.copyTo(T0(Range(0, 3), Range(3, 4)));

			// Invert T0
			Mat R0_inv = R0.t();
			Mat t0_inv = -R0_inv * t0;
			Mat T0_inv = Mat::eye(4, 4, CV_64F);
			R0_inv.copyTo(T0_inv(Range(0, 3), Range(0, 3)));
			t0_inv.copyTo(T0_inv(Range(0, 3), Range(3, 4)));

			// Store marker positions
			vector<MarkerInfo> markers;
			map<int, MarkerInfo> boundaryMarkers;

			// Compute relative pose for each marker
			for (size_t i = 0; i < nMarkers; i++) {
				Mat Ri;
				Rodrigues(rvecs[i], Ri);
				Mat Ti = Mat::eye(4, 4, CV_64F);
				Ri.copyTo(Ti(Range(0, 3), Range(0, 3)));
				Mat ti = Mat(tvecs[i]).reshape(1, 3);
				ti.copyTo(Ti(Range(0, 3), Range(3, 4)));

				// Relative transformation
				Mat T_rel = T0_inv * Ti;
				Mat R_rel = T_rel(Range(0, 3), Range(0, 3));
				Mat t_rel = T_rel(Range(0, 3), Range(3, 4));

				double x = fabs(t_rel.at<double>(0));
				double y = fabs(t_rel.at<double>(1));

				// Convert rotation matrix to Euler angles
				Vec3d eulerAngles = rotationMatrixToEulerAngles(R_rel);

				// Calculate rotation angle in degrees around Z-axis
				double rotationAngle = eulerAngles[2] * 180.0 / CV_PI;

				// Assign CellType based on marker ID
				CellType type = CellType::Obstacle;
				ObstacleShape shape = ObstacleShape::None; // Default shape

				if (ids[i] == 4) { // Assuming marker ID 4 is the target
					type = CellType::Target;
				}
				else if (ids[i] == 5) { // Assuming marker ID 5 is the start
					type = CellType::Start;
				}
				else {
					// Assign shapes based on marker IDs or any other logic
					switch (ids[i]) {
					case 6:
						shape = ObstacleShape::Circle;
						break;
					case 7:
						shape = ObstacleShape::UShape;
						break;
					default:
						shape = ObstacleShape::Rectangle;
						break;
					}
				}
				markers.push_back(MarkerInfo{ ids[i], y, x, rotationAngle, type, shape }); //y and x are swapped to match the gridMap orientation when ID 0 is rotated right 90 degrees

				// Identify boundary markers (IDs 0,1)
				if (ids[i] >= 0 && ids[i] <= 1) {
					boundaryMarkers[ids[i]] = MarkerInfo{ ids[i], y, x, rotationAngle, CellType::Empty, ObstacleShape::None };
				}
			}

			if (markers.empty()) {
				cout << "No markers to generate gridMap." << endl;
				return gridMap;
			}

			if (boundaryMarkers.size() < 2) {
				cout << "Not all boundary markers (IDs 0-1) detected." << endl;
				return gridMap;
			}

			// Define grid boundaries using boundary markers
			// Define grid boundaries using boundary markers
			double minX = min(boundaryMarkers[0].x, boundaryMarkers[1].x);
			double maxX = max(boundaryMarkers[0].x, boundaryMarkers[1].x);
			double minY = min(boundaryMarkers[0].y, boundaryMarkers[1].y);
			double maxY = max(boundaryMarkers[0].y, boundaryMarkers[1].y);

			// Define grid resolution (e.g., 0.05 meters per cell)
			const double resolution = 0.01;

			// Calculate grid size
			int gridWidth = static_cast<int>(ceil((maxX - minX) / resolution)) + 1;
			int gridHeight = static_cast<int>(ceil((maxY - minY) / resolution)) + 1;

			// Initialize gridMap with Empty cells
			gridMap.assign(gridHeight, vector<CellType>(gridWidth, CellType::Empty));

			// Populate gridMap with boundary markers
			for (int id = 0; id <= 1; id++) {
				auto& marker = boundaryMarkers[id];
				int gridX = static_cast<int>(round((marker.x - minX) / resolution));
				int gridY = static_cast<int>(round((marker.y - minY) / resolution));

				// Ensure indices are within bounds
				gridX = clamp(gridX, 0, gridWidth - 1);
				gridY = clamp(gridY, 0, gridHeight - 1);

				gridMap[gridY][gridX] = CellType::Empty;
			}

			// Populate gridMap with markers
			for (const auto& marker : markers) {

				cout << "Marker ID: " << marker.id << ", X: " << marker.x << ", Y: " << marker.y
					<< ", Rotation: " << marker.rotation << ", Type: " << static_cast<int>(marker.type)
					<< ", Shape: " << static_cast<int>(marker.shape) << endl;
				// Skip boundary markers
				if (marker.id == 0 || marker.id == 1) continue;

				int centerX = static_cast<int>(round((marker.x - minX) / resolution));
				int centerY = static_cast<int>(round((marker.y - minY) / resolution));

				// Define shape parameters based on obstacle shape
				switch (marker.shape) {
				case ObstacleShape::Rectangle:
					addRectangle(gridMap, centerX, centerY, 30, 8, marker.rotation, resolution, minX, minY);
					break;
				case ObstacleShape::Circle:
					addCircle(gridMap, centerX, centerY, 26, resolution, minX, minY);
					break;
				case ObstacleShape::UShape:
					addUShape(gridMap, centerX, centerY, 16, 40, marker.rotation, resolution, minX, minY);
					break;
				default:
					// Default to single cell if shape is undefined
					if (centerX >= 0 && centerX < gridWidth && centerY >= 0 && centerY < gridHeight) {
						gridMap[centerY][centerX] = marker.type;
					}
					break;
				}
				/*if (centerX >= 0 && centerX < gridWidth && centerY >= 0 && centerY < gridHeight) {
					gridMap[centerY][centerX] = marker.type;
				}*/
			}

			// Enforce boundary walls
			int rows = gridMap.size();
			int cols = gridMap[0].size();
			for (int y = 0; y < rows; ++y)
			{
				for (int x = 0; x < cols; ++x)
				{
					if (x == 0 || x == cols - 1 || y == 0 || y == rows - 1)
						gridMap[y][x] = CellType::Obstacle;
				}
			}
		}
		else {
			// Marker ID 0 not detected
			cout << "Marker ID 0 not detected. Cannot compute relative positions." << endl;
		}
	}
	return gridMap;
}

void calibrateCameraFromImages(const string& dir, Mat& cameraMatrix, Mat& distCoeffs)
{
	const int chessboardWidth = 9;
	const int chessboardHeight = 6;
	const Size boardSize = Size(chessboardWidth, chessboardHeight);

	// Prepare object points (0,0,0), (1,0,0), ..., (8,5,0)
	vector<Point3f> objp;
	for (int i = 0; i < chessboardHeight; i++)
	{
		for (int j = 0; j < chessboardWidth; j++)
			objp.push_back(Point3f(j, i, 0));
	}

	vector<vector<Point3f>> objpoints;
	vector<vector<Point2f>> imgpoints;

	// Get list of image files in directory
	vector<string> imageFiles;
	for (const auto& entry : filesystem::directory_iterator(dir))
	{
		if (entry.is_regular_file())
		{
			string ext = entry.path().extension().string();
			if (ext == ".jpg" || ext == ".jpeg" || ext == ".png" || ext == ".bmp" || ext == ".tiff")
			{
				imageFiles.push_back(entry.path().string());
			}
		}
	}

	if (imageFiles.empty())
	{
		cout << "No images found in directory: " << dir << endl;
		return;
	}

	int imagesCollected = 0;
	Mat frame, gray;

	for (const auto& imgPath : imageFiles)
	{
		frame = imread(imgPath);
		if (frame.empty())
		{
			cout << "Failed to load image: " << imgPath << endl;
			continue;
		}

		cvtColor(frame, gray, COLOR_BGR2GRAY);

		vector<Point2f> corners;
		bool found = false;

		found = findChessboardCorners(gray, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH
			+ CALIB_CB_NORMALIZE_IMAGE
			+ CALIB_CB_FAST_CHECK);
		

		if (found)
		{
			TermCriteria criteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.001);
			cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), criteria);
			drawChessboardCorners(frame, boardSize, corners, found);

			objpoints.push_back(objp);
			imgpoints.push_back(corners);
			imagesCollected++;
			cout << "Processed calibration image " << imagesCollected << ": " << imgPath << endl;
		}
	}

	if (imagesCollected < 1)
    {
        cout << "Not enough calibration images." << endl;
        return;
    }

	Size frameSize = frame.size();
	cout << "Frame size: " << frameSize << endl;
	vector<Mat> rvecs, tvecs;
	try {
		calibrateCamera(objpoints, imgpoints, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs);		
	}
	catch (const cv::Exception& e) {
		cerr << "Calibration failed: " << e.what() << endl;
		return;
	}
	// Save camera matrix and distortion coefficients to file
	FileStorage fs(CALIB_PARAMS_FILE, FileStorage::WRITE);
	fs << "cameraMatrix" << cameraMatrix;
	fs << "distCoeffs" << distCoeffs;
	fs.release();
	//calibrateCamera(objpoints, imgpoints, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs);
}

// Function to convert rotation matrix to Euler angles
Vec3d rotationMatrixToEulerAngles(Mat& R)
{
	double sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

	bool singular = sy < 1e-6; // If

	double x, y, z;
	if (!singular)
	{
		x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
		y = atan2(-R.at<double>(2, 0), sy);
		z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
	}
	else
	{
		x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
		y = atan2(-R.at<double>(2, 0), sy);
		z = 0;
	}
	return Vec3d(x, y, z);
}

pair<int, int> rotatePoint(const int& x, const int& y, const double& angle)
{
	double rad = -angle * CV_PI / 180.0;
	double cosA = cos(rad);
	double sinA = sin(rad);
	double newX = x * cosA - y * sinA;
	double newY = x * sinA + y * cosA;
	return { static_cast<int>(round(newX)), static_cast<int>(round(newY)) };
}

void addRectangle(vector<vector<CellType>>& gridMap,
	const int& centerX, const int& centerY, const int& width, const int& height,
	const double& angle, const double& resolution, const double& minX, const double& minY)
{
	// Define half-dimensions
	double halfW = width / 2.0;
	double halfH = height / 2.0;

	// Define rectangle corners relative to center
	vector<pair<double, double>> corners = {
		{-halfW, -halfH},
		{ halfW, -halfH},
		{ halfW,  halfH},
		{-halfW,  halfH}
	};

	// Rotate corners based on the angle
	vector<Point> rotatedCorners;
	for (const auto& corner : corners) {
		pair<int, int> rotated = rotatePoint(static_cast<int>(round(corner.first)),
			static_cast<int>(round(corner.second)),
			angle);
		// Translate to grid coordinates
		int gridX = centerX + rotated.first;
		int gridY = centerY + rotated.second;
		rotatedCorners.emplace_back(Point(gridX, gridY));
	}

	// Create a temporary mask with the same dimensions as gridMap
	int gridHeight = gridMap.size();
	int gridWidth = gridMap[0].size();
	Mat mask = Mat::zeros(gridHeight, gridWidth, CV_8UC1);

	// Define the polygon by the rotated corners
	const Point* pts[1] = { rotatedCorners.data() };
	int npts[] = { static_cast<int>(rotatedCorners.size()) };

	// Fill the polygon on the mask
	fillPoly(mask, pts, npts, 1, Scalar(255));

	// Iterate through the mask and update gridMap
	for (int y = 0; y < gridHeight; ++y) {
		uchar* row = mask.ptr<uchar>(y);
		for (int x = 0; x < gridWidth; ++x) {
			if (row[x] == 255) {
				gridMap[y][x] = CellType::Obstacle;
			}
		}
	}
}

void addCircle(vector<vector<CellType>>& gridMap,
	const int& centerX, const int& centerY, const int& radius,
	const double& resolution, const double& minX, const double& minY)
{
	for (int dx = -radius; dx <= radius; ++dx) {
		for (int dy = -radius; dy <= radius; ++dy) {
			if (dx * dx + dy * dy <= radius * radius) {
				// Calculate new grid indices by adding offsets
				int gridX = centerX + dx;
				int gridY = centerY + dy;

				// Ensure indices are within bounds
				if (gridX >= 0 && gridX < gridMap[0].size() && gridY >= 0 && gridY < gridMap.size()) {
					gridMap[gridY][gridX] = CellType::Obstacle;
				}
			}
		}
	}
}

void addUShape(vector<vector<CellType>>& gridMap,
	const int& centerX, const int& centerY, const int& width, const int& height,
	const double& angle, const double& resolution, const double& minX, const double& minY)
{
	// Define relative positions for U-shape components
	// Bottom horizontal rectangle
	pair<int, int> bottomRelPos = { 0, height / 3 };
	// Left vertical rectangle
	pair<int, int> leftRelPos = { -width / 2, 0 };
	// Right vertical rectangle
	pair<int, int> rightRelPos = { width / 2, 0 };

	// Rotate relative positions based on the angle
	pair<int, int> bottomRotated = rotatePoint(bottomRelPos.first, bottomRelPos.second, angle);
	pair<int, int> leftRotated = rotatePoint(leftRelPos.first, leftRelPos.second, angle);
	pair<int, int> rightRotated = rotatePoint(rightRelPos.first, rightRelPos.second, angle);

	// Calculate absolute positions by adding rotated relative positions to the center
	int bottomX = centerX + bottomRotated.first;
	int bottomY = centerY + bottomRotated.second;

	int leftX = centerX + leftRotated.first;
	int leftY = centerY + leftRotated.second;

	int rightX = centerX + rightRotated.first;
	int rightY = centerY + rightRotated.second;

	// Add rectangles without further rotation (angle = 0)
	addRectangle(gridMap, bottomX, bottomY, width, height / 3, angle, resolution, minX, minY);
	addRectangle(gridMap, leftX, leftY, width / 3, height, angle, resolution, minX, minY);
	addRectangle(gridMap, rightX, rightY, width / 3, height, angle, resolution, minX, minY);
}

void printGridMap(const vector<vector<CellType>>& gridMap) {
	if (gridMap.empty()) {
		cout << "GridMap is empty." << endl;
		return;
	}
	int rows = gridMap.size();
	int cols = gridMap[0].size();

	for (int y = 0; y < rows; y++) {
		for (int x = 0; x < cols; x++) {
			switch (gridMap[y][x]) {
			case CellType::Empty:
				cout << ". ";
				break;
			case CellType::Obstacle:
				cout << "O ";
				break;
			case CellType::Target:
				cout << "T ";
				break;
			case CellType::Start:
				cout << "S ";
				break;
			default:
				cout << "? ";
				break;
			}
		}
		cout << endl;
	}
}

// Function to display the gridMap using OpenCV
void showGridMap(const std::vector<std::vector<CellType>>& gridMap, int cellSize = 10) {
	if (gridMap.empty()) {
		std::cout << "GridMap is empty. Nothing to display." << std::endl;
		return;
	}

	int gridHeight = gridMap.size();
	int gridWidth = gridMap[0].size();

	// Create a color image with white background
	cv::Mat displayImage(gridHeight * cellSize, gridWidth * cellSize, CV_8UC3, cv::Scalar(255, 255, 255));

	for (int y = 0; y < gridHeight; ++y) {
		for (int x = 0; x < gridWidth; ++x) {
			cv::Scalar color;

			// Assign colors based on CellType
			switch (gridMap[y][x]) {
			case CellType::Empty:
				color = cv::Scalar(255, 255, 255); // White
				break;
			case CellType::Obstacle:
				color = cv::Scalar(0, 0, 0); // Black
				break;
			case CellType::Target:
				color = cv::Scalar(0, 0, 255); // Red
				break;
			case CellType::Start:
				color = cv::Scalar(0, 255, 0); // Green
				break;
			default:
				color = cv::Scalar(200, 200, 200); // Grey for undefined types
				break;
			}

			// Define the top-left and bottom-right points of the cell
			cv::Point topLeft(x * cellSize, y * cellSize);
			cv::Point bottomRight((x + 1) * cellSize - 1, (y + 1) * cellSize - 1);

			// Draw the filled rectangle
			cv::rectangle(displayImage, topLeft, bottomRight, color, cv::FILLED);
		}
	}

	// Display the image in a window
	cv::imshow("Grid Map", displayImage);
	cv::waitKey(0); // Wait for a key press indefinitely
	cv::destroyAllWindows(); // Close the window after a key press
}