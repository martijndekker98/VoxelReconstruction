#ifndef BACKGROUND_H_
#define BACKGROUND_H_

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <opencv2/core/cvstd.hpp>
#include <string>
using namespace std;

namespace nl_uu_science_gmt
{
	class Background
	{
	public:
		static int computeMedian(vector<int> elements);
		static cv::Mat compute_median(std::vector<cv::Mat> vec);
		static void findOrCreateBackground();
	};

}
#endif /* BACKGROUND_H_ */