#include "opencv2/tracking.hpp"
#include <iostream>

// Forward declare OpenCV types
namespace cv {
class Mat;
class KalmanFilter;
}  // namespace cv

namespace adas {
class EgoFrame{
public:
	int stateDim = 5; //x, y, vx, vy, yaw
	int measDim = 4; //x, y, vx, vy (no change in yaw)
	//int measDim = 5; //x, y, vx, vy, yaw
	
	cv::KalmanFilter kf = cv::KalmanFilter();
	bool kf_initialized = false;
	
	/**
	 * Empty constructor
	 */
	EgoFrame ();
	
	/**
	 * Constructor for EgoFrame
	 */
	EgoFrame (cv::Mat initialState);
	
	/**
	 * init function
	 * 
	 * Mat initialState: cv::Mat matrix that is the initial state of the EgoFrame
	 * initialState should be a (5,1) matrix contining x, y, vx, vy, yaw
	 * 
	 */
	void init(cv::Mat initialState);
	
	//Makes prediction of the next state of object
	cv::Mat getPrediction();
	
	cv::Mat update(cv::Mat measurement, float dt);
};
}