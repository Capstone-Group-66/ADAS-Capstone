#include "adas/stage_e/Track.hpp"
#include "opencv2/tracking.hpp"
#include <iostream>
	
namespace adas {

	Track::Track (cv::Mat initialState) {
		kf.init(stateDim, measDim, 0, CV_32F);
		
		float dt = 0.05; //Elapsed time
		//Updates the position based off the velocity * elapsed time
		kf.transitionMatrix = (cv::Mat_<float>(stateDim,stateDim) <<
			1, 0, dt, 0, 0,
			0, 1, 0, dt, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0,
			0, 0, 0, 0, 1);
		
		kf.measurementMatrix = (cv::Mat_<float>(measDim,stateDim) <<
			1, 0, 0, 0, 0,
			0, 1, 0, 0, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0);
		
		kf.statePost = initialState;
	}
	
	//Makes prediction of the next state of object
	cv::Mat Track::getPrediction(){
		cv::Mat prediction = kf.predict();
		return prediction;
	}
	
	cv::Mat Track::update(cv::Mat measurement, float dt){		
		//setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
		//setIdentity(kf.measurementNoiseCov, Scalar::all(1e-1));
		//setIdentity(kf.errorCovPost, Scalar::all(1));
		
		//Update dt for accuracy
		kf.transitionMatrix = (cv::Mat_<float>(stateDim,stateDim) <<
			1, 0, dt, 0, 0,
			0, 1, 0, dt, 0,
			0, 0, 1, 0, 0,
			0, 0, 0, 1, 0,
			0, 0, 0, 0, 1);

		cv::Mat estimated = kf.correct(measurement);
		return estimated;
	}
}