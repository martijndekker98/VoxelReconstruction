#include <cstdlib>
#include <string>
#include <iostream>
#include <random>
#include <sstream>
#include <filesystem>

#include "utilities/General.h"
#include "VoxelReconstruction.h"
#include "utilities/Background.h"
#include "utilities/Calibrate.h"
#include "controllers/Camera.h"

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/calib3d.hpp"

using namespace std;
using namespace nl_uu_science_gmt;
using namespace std::filesystem;
using namespace cv;

const string datapath = current_path().parent_path().parent_path().string() + "\\data\\";


void analyseVideo(string fileName) {
	//Camera::detExtrinsics( const string& data_path, const string& checker_vid_fname, const string& intr_filename, const string& out_fname)
	cout << "test analysing videos for extrinsics" << endl;
	Camera::detExtrinsics("J:\\computer vision\\VoxelReconstruction\\data\\" + fileName + "\\", "checkerboard.avi", "intrinsics.xml", "extrinsics.xml");

	cout << "done analysing" << endl;
}

void analyseVideos() {
	for (int i = 1; i < 5; i++)
	{
		Camera::detExtrinsics("J:\\VoxelReconstruction\\data\\cam" + to_string(i) + "\\", "checkerboard.avi", "intrinsics.xml", "extrinsics.xml");
	}
	cout << "Done analysing all videos" << endl;
}

void analysePic() {
	string path = current_path().parent_path().parent_path().string();
	string folderName = "cam1";
	string newPath = path + "\\data\\" + folderName + "\\checkerboard.avi";
	VideoCapture capture(newPath);

	//Mat source = imread("J:\\Greenshot\\checkerboard.jpg", 1);
	namedWindow("test", 1);
	/*
	Mat viewGray;
	cvtColor(source, viewGray, COLOR_BGR2GRAY);
	imshow("test", source);
	waitKey(3000);
	imshow("test", viewGray);
	waitKey(3000);*/

	Mat mask, mask_strict;
	Scalar lowerb = Scalar(0, 0, 0);
	Scalar upperb = Scalar(110, 110, 110);

	Mat view;
	capture >> view;

	Mat viewGray;
	cvtColor(view, viewGray, COLOR_BGR2GRAY);
	imshow("test", viewGray);
	waitKey(3000);

	Scalar lowerb1 = Scalar(234, 234, 234);
	Scalar upperb1 = Scalar(255, 255, 255);
	inRange(view, lowerb1, upperb1, mask_strict);
	imshow("test", mask_strict);
	waitKey(3000);

	//threshold(mask_strict, contours, 200, 255, THRESH_BINARY);
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(mask_strict, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	Mat withConts = Mat::zeros(mask_strict.rows, mask_strict.cols, CV_8UC3);
	int idx = 0;
	for (; idx >= 0; idx = hierarchy[idx][0])
	{
		Scalar color(0, 0, 255);
		drawContours(withConts, contours, idx, color, FILLED, 8, hierarchy);
	}
	imshow("test", withConts);
	waitKey(3000);
	/*
	vector<Point2f> pointbuf;
	bool found;
	Size boardSize;
	boardSize.width = 8;
	boardSize.height = 6;

	cornerSubPix(viewGray, pointbuf, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
	found = findChessboardCorners(view, boardSize, pointbuf, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
	if (found)
		drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
	else
		cout << "NOT FOUND!" << endl;
	imshow("test", view);
	waitKey(3000);*/
}

int main( int argc, char** argv) {
	// init background stuff
    Background::findOrCreateBackground();

	Calibrate::calibrate();

	cout << "Calibrating done" << endl;
	waitKey(2000);
	analyseVideos();

	/*
	vector<Camera*> cams;
	// camera init
	Camera camera1(datapath + "cam1\\", General::IntrinsicsFile, 1);
	camera1.initialize();
	
	Camera camera2(datapath + "cam2\\", General::IntrinsicsFile, 2);
	camera2.initialize();

	Camera camera3(datapath + "cam3\\", General::IntrinsicsFile, 3);
	camera3.initialize();

	Camera camera4(datapath + "cam4\\", General::IntrinsicsFile, 4);
	camera4.initialize();

	cout << "Cameras initialised" << endl;
	waitKey(3000);*/

	string data = current_path().parent_path().parent_path().string() + "\\data";
	VoxelReconstruction::showKeys();
	VoxelReconstruction vr(data + std::string(PATH_SEP), 4);
	vr.run(argc, argv);
	
	return EXIT_SUCCESS;
}
