#include "opencv2/tracking.hpp"
#include <iostream>

// Forward declare OpenCV types
namespace cv {
class Mat;
class KalmanFilter;
}  // namespace cv

namespace adas {
class Track{
public:
	uint32_t id; 

	int cls;

	uint16_t sources;

	float conf_01;
	
	float stateDim = 5; //x, y, vx, vy, w

	int measDim = 4; //x, y, vx, vy (assumes w dosent change)
	
	
	cv::KalmanFilter kf = cv::KalmanFilter();
	bool kf_initialized = false;

	Track ();

	Track (cv::Mat initialState);

	void init(cv::Mat initialState);
	
	//Makes prediction of the next state of object
	cv::Mat getPrediction();
	
	cv::Mat update(cv::Mat measurement, float dt);
};
}