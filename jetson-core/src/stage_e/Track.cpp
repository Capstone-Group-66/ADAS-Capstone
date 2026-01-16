#include "opencv2/tracking.hpp"
#include <iostream>

using namespace cv;
	
class Track{
public:
	int stateDim = 5; //x, y, vx, vy, w
	int measDim = 4; //x, y, vx, vy (assumes w dosent change)
	
	int object_id;
	
	KalmanFilter kf = KalmanFilter();

	Track (Mat initialState) {
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
		
		kf.statePost = initialState;
	}
	
	//Makes prediction of the next state of object
	Mat getPrediction(){
		Mat prediction = kf.predict();
		return prediction;
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

		Mat estimated = kf.correct(measurement);
		return estimated;
	}
};
