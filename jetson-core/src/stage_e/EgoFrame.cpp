#include "opencv2/tracking.hpp"
#include <iostream>

using namespace cv;

class EgoFrame{
public:
	int stateDim = 5; //x, y, vx, vy, yaw
	int measDim = 4; //x, y, vx, vy (no change in yaw)
	//int measDim = 5; //x, y, vx, vy, yaw
	
	KalmanFilter kf = KalmanFilter();
	
	/**
	 * Empty constructor
	 */
	EgoFrame (){}
	
	/**
	 * Constructor for EgoFrame
	 */
	EgoFrame (Mat initialState) {
		init(initialState);
	}
	
	/**
	 * init function
	 * 
	 * Mat initialState: cv::Mat matrix that is the initial state of the EgoFrame
	 * initialState should be a (5,1) matrix contining x, y, vx, vy, yaw
	 * 
	 */
	void init(Mat initialState){
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
		return kf.predict();
	}
	
	Mat update(Mat measurement, float dt){		
		//setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
		//setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
		//setIdentity(kf.errorCovPost, Scalar::all(1));
		
		//Update dt for accuracy
		kf.transitionMatrix = (Mat_<float>(stateDim,stateDim) <<
			1, 0, dt, 0, 0,
			0, 1, 0, dt, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0,
			0, 0, 0, 0, 1);
		
		return kf.correct(measurement);
	}
};
