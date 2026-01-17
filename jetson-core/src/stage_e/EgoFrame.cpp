#include "adas/stage_e/EgoFrame.hpp"
#include "opencv2/tracking.hpp"
#include <iostream>

namespace adas {
	EgoFrame::EgoFrame (){}
	
	/**
	 * Constructor for EgoFrame
	 */
	EgoFrame::EgoFrame (cv::Mat initialState) {
		init(initialState);
	}
	
	/**
	 * init function
	 * 
	 * Mat initialState: cv::Mat matrix that is the initial state of the EgoFrame
	 * initialState should be a (5,1) matrix contining x, y, vx, vy, yaw
	 * 
	 */
	void EgoFrame::init(cv::Mat initialState){
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
		
		// If yaw exists
		// kf.measurementMatrix = (cv::Mat_<float>(measDim,stateDim) <<
		// 	1, 0, 0, 0, 0,
		// 	0, 1, 0, 0, 0,
		// 	0, 0, 1, 0, 0,
		// 	0, 0, 0, 1, 0,
		// 	0, 0, 0, 0, 1);

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
		std::cout << "[EgoFrame] EgoFrame initiated" << std::endl;
	}
	
	//Makes prediction of the next state of object
	cv::Mat EgoFrame::getPrediction(){
		return kf.predict();
	}
	
	cv::Mat EgoFrame::update(cv::Mat measurement, float dt){
		std::cout << measurement << std::endl;	
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

		if(kf_initialized){
			init(measurement);
			return getPrediction();
		}
		
		std::cout << "[EgoFrame] EgoFrame updated" << std::endl;
		return kf.correct(measurement);
	}
};
