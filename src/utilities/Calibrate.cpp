
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <filesystem>
#include "Calibrate.h"

using namespace cv;
using namespace std;
using namespace std::filesystem;

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };


// reprojection isn't perfect, but done based on estimated distortion coefficients and the cameramatrix. here we compute the error
// of reprojecting.
double Calibrate::computeReprojectionErrors(
    const vector<vector<Point3f> >& objectPoints,
    const vector<vector<Point2f> >& imagePoints,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const Mat& camMat, const Mat& distMat,
    vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); i++)
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
            camMat, distMat, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }

    return std::sqrt(totalErr / totalPoints);
}

// calculate the corners of the pattern
void Calibrate::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
    corners.resize(0);

    switch (patternType)
    {
    case CHESSBOARD:
    case CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
                corners.push_back(Point3f(float(j * squareSize),
                    float(i * squareSize), 0));
        break;

    case ASYMMETRIC_CIRCLES_GRID:
        for (int i = 0; i < boardSize.height; i++)
            for (int j = 0; j < boardSize.width; j++)
                corners.push_back(Point3f(float((2 * j + i % 2) * squareSize),
                    float(i * squareSize), 0));
        break;

    default:
        CV_Error(Error::StsBadArg, "Unknown pattern type\n");
    }
}

bool Calibrate::runCalibration(vector<vector<Point2f> > imagePoints, Size imageSize, Size boardSize, Pattern patternType, float squareSize, float aspectRatio, float grid_width, bool release_object, int flags, Mat& camMat, Mat& distMat, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, vector<Point3f>& newObjPoints, double& totalAvgErr)
{
    camMat = Mat::eye(3, 3, CV_64F);
    if (flags & CALIB_FIX_ASPECT_RATIO)
        camMat.at<double>(0, 0) = aspectRatio;
    distMat = Mat::zeros(8, 1, CV_64F);
    vector<vector<Point3f> > objectPoints(1);
    Calibrate::calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
    objectPoints[0][boardSize.width - 1].x = objectPoints[0][0].x + grid_width;
    newObjPoints = objectPoints[0];
    objectPoints.resize(imagePoints.size(), objectPoints[0]);

    double rms;
    int i;
    int iFixedPoint = -1;
    double rms_try;
    rms = calibrateCameraRO(objectPoints, imagePoints, imageSize, iFixedPoint, camMat, distMat, rvecs, tvecs, newObjPoints, flags | CALIB_FIX_K3 | CALIB_USE_LU);

    int best_i = -1;

    for (i = 0; i < (int)imagePoints.size(); i++) {
        vector<vector<Point2f>> newImagePoints(imagePoints);
        vector<vector<Point3f>> newObjectPoints(objectPoints);
        newImagePoints.erase(newImagePoints.begin() + i);
        newObjectPoints.erase(newObjectPoints.begin() + i);
        rms_try = calibrateCameraRO(newObjectPoints, newImagePoints, imageSize, iFixedPoint,
            camMat, distMat, rvecs, tvecs, newObjPoints,
            flags | CALIB_FIX_K3 | CALIB_USE_LU);
        if (rms_try < rms) {
            best_i = i;
            rms = rms_try;
        }
    }

    if (best_i >= 0) {
        imagePoints.erase(imagePoints.begin() + best_i);
        objectPoints.erase(objectPoints.begin() + best_i);
    }

    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(camMat) && checkRange(distMat);

    objectPoints.clear();
    objectPoints.resize(imagePoints.size(), newObjPoints);
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
        rvecs, tvecs, camMat, distMat, reprojErrs);

    return ok;
}


// save the found camera matrix and distortion coefficients to a file
void Calibrate::saveCameraParams(const string& filename,
    Size imageSize, Size boardSize,
    float squareSize, float aspectRatio, int flags,
    const Mat& camMat, const Mat& distMat,
    const vector<Mat>& rvecs, const vector<Mat>& tvecs,
    const vector<float>& reprojErrs,
    const vector<vector<Point2f> >& imagePoints,
    const vector<Point3f>& newObjPoints,
    double totalAvgErr)
{
    FileStorage fs(filename, FileStorage::WRITE);

    time_t tt;
    time(&tt);
    struct tm* t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    if (!rvecs.empty() || !reprojErrs.empty())
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if (flags & CALIB_FIX_ASPECT_RATIO)
        fs << "aspectRatio" << aspectRatio;

    if (flags != 0)
    {
        sprintf(buf, "flags: %s%s%s%s",
            flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << camMat;
    fs << "distortion_coefficients" << distMat;

    fs << "avg_reprojection_error" << totalAvgErr;
    if (!reprojErrs.empty())
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if (!rvecs.empty() && !tvecs.empty())
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for (int i = 0; i < (int)rvecs.size(); i++)
        {
            Mat r = bigmat(Range(i, i + 1), Range(0, 3));
            Mat t = bigmat(Range(i, i + 1), Range(3, 6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        fs << "extrinsic_parameters" << bigmat;
    }

    if (!imagePoints.empty())
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for (int i = 0; i < (int)imagePoints.size(); i++)
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }

    if (!newObjPoints.empty())
    {
        fs << "grid_points" << newObjPoints;
    }
}

// call the runCalibration and saveCameraParams functions.
bool Calibrate::runAndSave(const string& outputFilename,
    const vector<vector<Point2f> >& imagePoints,
    Size imageSize, Size boardSize, Pattern patternType, float squareSize,
    float grid_width, bool release_object,
    float aspectRatio, int flags, Mat& camMat,
    Mat& distMat, bool writeExtrinsics, bool writePoints, bool writeGrid)
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;
    vector<Point3f> newObjPoints;

    bool ok = Calibrate::runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
        aspectRatio, grid_width, release_object, flags, camMat, distMat,
        rvecs, tvecs, reprojErrs, newObjPoints, totalAvgErr);
    printf("%s. avg reprojection error = %.7f\n",
        ok ? "Calibration succeeded" : "Calibration failed",
        totalAvgErr);

    if (ok)
    {
        cout << "done" << endl;
        Calibrate::saveCameraParams(outputFilename, imageSize,
            boardSize, squareSize, aspectRatio,
            flags, camMat, distMat,
            writeExtrinsics ? rvecs : vector<Mat>(),
            writeExtrinsics ? tvecs : vector<Mat>(),
            writeExtrinsics ? reprojErrs : vector<float>(),
            writePoints ? imagePoints : vector<vector<Point2f> >(),
            writeGrid ? newObjPoints : vector<Point3f>(),
            totalAvgErr);
    }
    return ok;
}

int Calibrate::calibrate() {
    Size boardSize, imageSize;
    float squareSize, aspectRatio = 1;
    Mat camMat, distMat;
    string outputFilename = "out_camera_data.yml";
    string inputFilename = "";

    int i;
    int nframes = 30;
    bool writeExtrinsics = false, writePoints = true, writeGrid = true;
    //
    //
    // write extrinsics was true
    //
    //
    bool undistortImage = false;
    int flags = 0;
    //VideoCapture capture;
    int delay = 1000;
    clock_t prevTimestamp = 0;
    int mode = CAPTURING;
    int cameraId = 0;
    vector<vector<Point2f> > imagePoints;
    Pattern pattern = CHESSBOARD;
    bool calibrated = false;
    int winSize;

 
    boardSize.width = 8;
    boardSize.height = 6;
    squareSize = 115;
    winSize = 11;
    float grid_width = squareSize * (boardSize.width - 1);
    bool release_object = false;

    string path = "D:\\Master\\1e jaar\\Computer Vision\\Repos\\VoxelReconstruction";
    string newPath = path + "\\data\\cam4\\intrinsics.avi";
    VideoCapture capture(newPath);


    if (!capture.isOpened())
        throw "Error when reading file";

    if (squareSize <= 0)
        return fprintf(stderr, "Invalid board square width\n"), -1;
    if (nframes <= 3)
        return printf("Invalid number of images\n"), -1;
    if (aspectRatio <= 0)
        return printf("Invalid aspect ratio\n"), -1;
    if (delay <= 0)
        return printf("Invalid delay\n"), -1;
    if (boardSize.width <= 0)
        return fprintf(stderr, "Invalid board width\n"), -1;
    if (boardSize.height <= 0)
        return fprintf(stderr, "Invalid board height\n"), -1;

    namedWindow("w", 1);
    for (i = 0; ; i++) {
        Mat view, viewGray;
        imageSize = view.size();

        capture >> view;
        if (view.empty())
            break;

        if (i % 50 == 0) {
            imageSize = view.size();
            vector<Point2f> pointbuf;
            cvtColor(view, viewGray, COLOR_BGR2GRAY);

            bool found;
            Mat temp = view.clone();
            //determine if patterin in view
            switch (pattern)
            {
            case CHESSBOARD:
                found = findChessboardCorners(view, boardSize, pointbuf,
                    CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
                break;
            case CIRCLES_GRID:
                found = findCirclesGrid(view, boardSize, pointbuf);
                break;
            case ASYMMETRIC_CIRCLES_GRID:
                found = findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
                break;
            default:
                return fprintf(stderr, "Unknown pattern type\n"), -1;
            }
            //if (found)
            //    cout << "Found!" << endl;

            // if a chessboard is found, we attempt to increase the accuracy of determining the corners
            if (pattern == CHESSBOARD && found) {
                cornerSubPix(viewGray, pointbuf, Size(11, 11),
                    Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.0001));
            }
            if (mode == CAPTURING && found)//&&
              //(!capture.isOpened() || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC) )
            {
                cout << "Push pointbuf" << endl;
                imagePoints.push_back(pointbuf);
                prevTimestamp = clock();
            }
            // draw corners if the chessboard is in view and undistort is off (we dont want overlapping animations)
            if (found && undistortImage == false)
                drawChessboardCorners(view, boardSize, Mat(pointbuf), found);



            if (mode == CAPTURING && imagePoints.size() >= (unsigned)nframes) {
                Calibrate::runAndSave(outputFilename, imagePoints, imageSize, boardSize, pattern, squareSize, grid_width, release_object, aspectRatio, flags, camMat, distMat,
                    writeExtrinsics, writePoints, writeGrid);
                cout << "runAndSave!" << endl;
                mode = DETECTION;
            }
            imshow("w", view);

            //cout << "Size: " << imagePoints.size() << endl;
            
            char key = waitKey(2);
            if (key == 'q')
                break;

            }
        }
        cout << "frames: " << to_string(i) << endl;
        
}