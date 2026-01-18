#include "adas/stage_e/Track.hpp"
#include "opencv2/video/tracking.hpp"
#include <iostream>
	
namespace adas {
	Track::Track() {}

	Track::Track (cv::Mat initialState) {
		init(initialState);
	}

	void Track::init(cv::Mat initialState){
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
		
		if (initialState.rows != stateDim){
			initialState = (cv::Mat_<float>(stateDim,1) << 
			initialState.at<float>(0,0), 
			initialState.at<float>(1,0), 
			initialState.at<float>(2,0), 
			initialState.at<float>(3,0), 
			0.0f);
		}
		kf.statePost = initialState;
		kf_initialized = true;
	}
	
	//Makes prediction of the next state of object
	cv::Mat Track::getPrediction(){
		if(!kf_initialized){
			return cv::Mat();
		}
		return kf.predict();
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

		if(!kf_initialized){
			init(measurement);
			return getPrediction();
		}

		cv::Mat estimated = kf.correct(measurement);
		object_detected = true;
		return estimated;
	}
}