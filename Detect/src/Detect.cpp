#include "../include/Detect.hpp"

Mat cameraMatrix, distCoeffs;

int main()
{
	/*string imgPath;
	Mat img = imread(imgPath);*/

	// Generate and save a chessboard image
	//generateChessboardImage(20, 20, 500, "D:\\chessboard.png");

	VideoCapture cap(0, CAP_MSMF);
	if (!cap.isOpened())
	{
		cout << "Error opening video stream or file" << endl;
		return -1;
	}

	cap.set(CAP_PROP_FRAME_WIDTH, 1200);
	cap.set(CAP_PROP_FRAME_HEIGHT, 900);

	int waitTime = 10;

	bool showRejected = false;
	bool estimatePose = true;

	aruco::DetectorParameters detectorParams = aruco::DetectorParameters();
	aruco::Dictionary dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_250);
	aruco::ArucoDetector detector(dictionary, detectorParams);

	// set coordinate system
	const float markerLength = 0.19f;
	Mat objPoints(4, 1, CV_32FC3);
	objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0.0);
	objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0.0);
	objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0.0);
	objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0.0);


	// Get camera matrix and distortion coefficients
	/*Mat cameraMatrix = getCameraMatrix();
	Mat distCoeffs = getDistortionCoefficients();*/
	calibrateCameraCustom(cameraMatrix, distCoeffs);

	cout << "Camera Matrix: " << cameraMatrix << endl;
	cout << "Distortion Coefficients: " << distCoeffs << endl;


	int frameCounter = 0;
	double fps = 0.0;
	double timeStart = (double)getTickCount();
	string windowName = "Detecting";

	namedWindow(windowName, WINDOW_AUTOSIZE);

	while (cap.grab())
	{
		Mat frame, undistortedFrame, frameCopy;
		cap.retrieve(frame);
		//cout << "Frame size: " << frame.size() << endl;

		// Undistort the frame
		//undistort(frame, undistortedFrame, cameraMatrix, distCoeffs);		

		vector<int> ids;
		vector<vector<Point2f>> corners, rejected;
		detector.detectMarkers(frame, corners, ids, rejected);

		size_t nMarkers = corners.size();
		vector<Vec3d> rvecs(nMarkers), tvecs(nMarkers);

		if (estimatePose && !ids.empty()) {
			// Calculate pose for each marker
			for (size_t i = 0; i < nMarkers; i++) {
				solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
				//// Convert rotation vector to rotation matrix
				//Mat R;
				//Rodrigues(rvecs[i], R);

				//// Calculate rotation angle around Z-axis
				//double angle_z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
				//double angle_z_deg = angle_z * 180.0 / CV_PI;

				//// Print out the position and rotation angle
				//cout << "Marker ID: " << ids[i] << endl;
				//cout << "Position (x, y, z): [" << tvecs[i][0] << ", " << tvecs[i][1] << ", " << tvecs[i][2] << "] meters" << endl;
				//cout << "Rotation around Z-axis: " << angle_z_deg << " degrees" << endl << endl;
			}

			int idx0 = -1;
			for (size_t i = 0; i < ids.size(); i++) {
				if (ids[i] == 0) {
					idx0 = i;
					break;
				}
			}

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

					// Convert rotation matrix to Euler angles
					Vec3d eulerAngles = rotationMatrixToEulerAngles(R_rel);

					// Print out the relative position and rotation angles
					cout << "Marker ID: " << ids[i] << endl;
					cout << "Relative Position (x, y, z): [" << t_rel.at<double>(0) << ", " << t_rel.at<double>(1) << ", " << t_rel.at<double>(2) << "] meters" << endl;
					cout << "Rotation angles (x, y, z): [" << eulerAngles[0] * 180 / CV_PI << ", " << eulerAngles[1] * 180 / CV_PI << ", " << eulerAngles[2] * 180 / CV_PI << "] degrees" << endl << endl;
				}
			}
			else {
				// Marker ID 0 not detected
				cout << "Marker ID 0 not detected. Cannot compute relative positions." << endl;
			}

		}

		frame.copyTo(frameCopy);
		if (!ids.empty())
		{
			aruco::drawDetectedMarkers(frameCopy, corners, ids);

			if (estimatePose) {
				for (unsigned int i = 0; i < ids.size(); i++)
					cv::drawFrameAxes(frameCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], markerLength * 1.5f, 2);
			}
		}

		if (showRejected && !rejected.empty())
		{
			aruco::drawDetectedMarkers(frameCopy, rejected, noArray(), Scalar(100, 0, 255));
		}

		frameCounter++;

		if (frameCounter >= 10)
		{
			double timeEnd = (double)getTickCount();
			double timeDiff = (timeEnd - timeStart) / getTickFrequency();
			fps = frameCounter / timeDiff;

			frameCounter = 0;
			timeStart = timeEnd;

			string tiltle = windowName + " | FPS: " + to_string(int(fps));
			setWindowTitle(windowName, tiltle);
		}

		imshow(windowName, frameCopy);
		char key = (char)waitKey(waitTime);
		if (key == 27)
			break;
		else if (key == 'r')
			showRejected = !showRejected;
	}
	return 0;
}

void calibrateCameraCustom(Mat& cameraMatrix, Mat& distCoeffs)
{
	const int chessboardWidth = 6;
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

	// Capture images for calibration
	VideoCapture cap(0, CAP_MSMF);
	if (!cap.isOpened())
	{
		cout << "Error opening video stream or file" << endl;
		return;
	}

	cap.set(CAP_PROP_FRAME_WIDTH, 1200);
	cap.set(CAP_PROP_FRAME_HEIGHT, 900);

	int imagesToCollect = 30;
	int imagesCollected = 0;
	Mat frame, gray;

	while (imagesCollected < imagesToCollect)
	{
		Mat frame, gray;
		cap >> frame;
		if (frame.empty())
			break;
		cvtColor(frame, gray, COLOR_BGR2GRAY);

		vector<Point2f> corners;
		bool found = findChessboardCorners(gray, boardSize, corners);

		if (found)
		{
			TermCriteria criteria(TermCriteria::EPS | TermCriteria::COUNT, 30, 0.001);
			cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), criteria);
			drawChessboardCorners(frame, boardSize, corners, found);

			objpoints.push_back(objp);
			imgpoints.push_back(corners);
			imagesCollected++;
			cout << "Collected calibration image " << imagesCollected << "/" << imagesToCollect << endl;
			waitKey(500);
		}

		//imshow("Calibration", frame);
		char key = (char)waitKey(10);
		if (key == 27)
			break;
	}
	cap.release();
	//destroyWindow("Calibration");

	Size frameSize = cv::Size(1280, 720);
	vector<Mat> rvecs, tvecs;
	calibrateCamera(objpoints, imgpoints, frameSize, cameraMatrix, distCoeffs, rvecs, tvecs);
}

// Function to generate and save a chessboard image
void generateChessboardImage(const int& rows, const int& cols, const int& squareSize, const string& filename)
{
	int width = cols * squareSize;
	int height = rows * squareSize;
	Mat chessboard(height, width, CV_8UC1, Scalar(255));

	for (int i = 0; i < rows; ++i)
	{
		for (int j = 0; j < cols; ++j)
		{
			if ((i + j) % 2 == 0)
			{
				rectangle(chessboard, Point(j * squareSize, i * squareSize),
					Point((j + 1) * squareSize, (i + 1) * squareSize), Scalar(0), FILLED);
			}
		}
	}
	imwrite(filename, chessboard);
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