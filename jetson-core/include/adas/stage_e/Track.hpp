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
	int stateDim = 5; //x, y, vx, vy, w
	int measDim = 4; //x, y, vx, vy (assumes w dosent change)
	
	int object_id;
	
	cv::KalmanFilter kf = cv::KalmanFilter();

	Track (cv::Mat initialState);
	
	//Makes prediction of the next state of object
	cv::Mat getPrediction();
	
	cv::Mat update(cv::Mat measurement, float dt);
};
}