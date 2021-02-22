#include <cstdlib>
#include <string>
#include <iostream>
#include <random>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/cvstd.hpp>
#include "utilities/General.h"
#include "VoxelReconstruction.h"

#include <filesystem>

namespace fs = std::filesystem;
using namespace cv;
using namespace std;
using namespace nl_uu_science_gmt;



int computeMedian(vector<int> elements)
{
    nth_element(elements.begin(), elements.begin() + elements.size() / 2, elements.end());

    //sort(elements.begin(),elements.end());
    return elements[elements.size() / 2];
}

cv::Mat compute_median(std::vector<cv::Mat> vec)
{
    // Note: Expects the image to be CV_8UC3
    cv::Mat medianImg(vec[0].rows, vec[0].cols, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int row = 0; row < vec[0].rows; row++)
    {
        for (int col = 0; col < vec[0].cols; col++)
        {
            std::vector<int> elements_B;
            std::vector<int> elements_G;
            std::vector<int> elements_R;

            for (int imgNumber = 0; imgNumber < vec.size(); imgNumber++)
            {
                int B = vec[imgNumber].at<cv::Vec3b>(row, col)[0];
                int G = vec[imgNumber].at<cv::Vec3b>(row, col)[1];
                int R = vec[imgNumber].at<cv::Vec3b>(row, col)[2];

                elements_B.push_back(B);
                elements_G.push_back(G);
                elements_R.push_back(R);
            }

            medianImg.at<cv::Vec3b>(row, col)[0] = computeMedian(elements_B);
            medianImg.at<cv::Vec3b>(row, col)[1] = computeMedian(elements_G);
            medianImg.at<cv::Vec3b>(row, col)[2] = computeMedian(elements_R);
        }
    }
    return medianImg;
}

int main(
		int argc, char** argv)
{
    cout << "Searching for background.png files" << endl;

    std::string path = "D:\\Master\\1e jaar\\Computer Vision\\Repos\\VoxelReconstruction\\data";
    for (const auto& entry : fs::directory_iterator(path)) {

        string data_path = entry.path().string() + "\\";

        if (!General::fexists(data_path + General::BackgroundImageFile)) {

            if (fs::is_directory(entry.path())) {

                // init filepath
                cout << "Starting background frame averaging for videofile:" << endl << data_path + General::BackgroundVideoFile << endl;
                VideoCapture cap(data_path + General::BackgroundVideoFile);

                if (!cap.isOpened())
                    cerr << "Error opening video \n";

                // Select 10 frames
                default_random_engine gen;
                uniform_int_distribution<int>distribution(0,
                    cap.get(CAP_PROP_FRAME_COUNT));

                vector<Mat> frames;
                Mat frame;
                int nFrames = 10;

                // take frames from video
                for (int i = 0; i < nFrames; i++)
                {
                    int x = distribution(gen);
                    cap.set(CAP_PROP_POS_FRAMES, x);
                    Mat frame;
                    cap >> frame;
                    if (frame.empty())
                        continue;
                    frames.push_back(frame);
                }

                // Determine the median frame from all frames
                Mat mFrame = compute_median(frames);

                // Display median frame and write to file
                imshow("median frame", mFrame);
                imwrite(data_path + General::BackgroundImageFile, mFrame);
                cout << "Showing averaged frame, press any key to continue. Automatic continue in 5 seconds" << endl;
                cv::waitKey(5);
                cv::destroyWindow("median frame");
            }
        }
    }

	//VoxelReconstruction::showKeys();
	//VoxelReconstruction vr("data" + std::string(PATH_SEP), 4);
	//vr.run(argc, argv);

	return EXIT_SUCCESS;
}
