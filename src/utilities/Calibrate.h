using namespace std;
using namespace cv;

class Calibrate
{
public:
    static enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

    static double Calibrate::computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& camMat, const Mat& distMat,
        vector<float>& perViewErrors);
    static void Calibrate::calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType);
    static void Calibrate::writeXML(string filename, Mat intr, Mat coeffs);
    static bool Calibrate::checkIntrinsics(string fileName);
    static bool Calibrate::runCalibration(vector<vector<Point2f> > imagePoints,
        Size imageSize, 
        Size boardSize, 
        Pattern patternType, 
        float squareSize, 
        float aspectRatio, 
        float grid_width, 
        bool release_object, 
        int flags, 
        Mat& camMat, Mat& distMat, vector<Mat>& rvecs, vector<Mat>& tvecs, vector<float>& reprojErrs, vector<Point3f>& newObjPoints, double& totalAvgErr);
    static void Calibrate::saveCameraParams(const string& filename,
        Size imageSize, Size boardSize,
        float squareSize, float aspectRatio, int flags,
        const Mat& camMat, const Mat& distMat,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const vector<float>& reprojErrs,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Point3f>& newObjPoints,
        double totalAvgErr);
    static bool Calibrate::runAndSave(const string& outputFilename,
        const vector<vector<Point2f> >& imagePoints,
        Size imageSize, Size boardSize, Pattern patternType, float squareSize,
        float grid_width, bool release_object,
        float aspectRatio, int flags, Mat& camMat,
        Mat& distMat, bool writeExtrinsics, bool writePoints, bool writeGrid, string folderName, string fileName);
    static int Calibrate::calibrate();
    


};
