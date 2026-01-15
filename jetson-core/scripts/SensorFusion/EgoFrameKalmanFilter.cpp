#include "opencv2/tracking.hpp"
#include <iostream>

using namespace cv;

int stateDim = 5; //x, y, vx, vy, yaw
int measDim = 4; //x, y, vx, vy (if we don't have yaw)
//int measDim = 5; //x, y, vx, vy, yaw

class EgoFrameKalmanFilter{
public:
	KalmanFilter kf = KalmanFilter();
	
	/**
	 * Constructor for EgoFrameKalmanFilter
	 * 
	 * Mat initialState: cv::Mat matrix that is the initial state of the EgoFrame
	 * initialState should be a (5,1) matrix contining x, y, vx, vy, yaw
	 * 
	 */
	EgoFrameKalmanFilter (Mat initialState) {
		kf.init(stateDim, measDim, 0, CV_32F);
		
		float dt = 0.05; //Elapsed time
		//Updates the position based off the velocity * elapsed time
		kf.transitionMatrix = (Mat_<float>(stateDim,stateDim) <<
			1, 0, dt, 0, 0,
			0, 1, 0, dt, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0,
			0, 0, 0, 0, 1);
		
		kf.measurementMatrix = (Mat_<float>(measDim,stateDim) <<
			1, 0, 0, 0, 0,
			0, 1, 0, 0, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0);
		
		// If yaw exists
		// kf.measurementMatrix = (Mat_<float>(measDim,stateDim) <<
		// 	1, 0, 0, 0, 0,
		// 	0, 1, 0, 0, 0,
		// 	0, 0, 1, 0, 0,
		// 	0, 0, 0, 1, 0,
		// 	0, 0, 0, 0, 1);
		
		kf.statePost = initialState;
	}
	
	//Makes prediction of the next state of object
	Mat getPrediction(){
		Mat prediction = kf.predict();
		std::cout << "Predicated: " << std::endl << prediction << std::endl;
		return prediction;
	}
	
	Mat update(Mat measurement){		
		//setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
		//setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
		//setIdentity(kf.errorCovPost, Scalar::all(1));
		
		Mat estimated = kf.correct(measurement);
		std::cout << "Estimated: " << std::endl << estimated << std::endl << std::endl;
		return estimated;
	}
};
