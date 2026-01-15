#include "opencv2/tracking.hpp"
#include <iostream>
#include "ObjectKalmanFilter.cpp"
#include "EgoFrameKalmanFilter.cpp"

using namespace cv;

EgoFrameKalmanFilter egoFrame;

int stateDim = 5;

bool FCW_check(Mat ef, Mat track){
	float ef_x = ef.at<float>(0,0);
	float track_x = track.at<float>(0,0);
	
	// float ef_y = ef.at<float>(1,1);
	// float track_y = track.at<float>(1,1);

	float ef_vx = ef.at<float>(3,3);
	float track_vx = track.at<float>(3,3);	

	// float ef_vy = ef.at<float>(4,4);
	// float track_vy = track.at<float>(4,4);
	
	float stop_time = ef_vx / (0.7 * 9.81) + 2.5;

	return stop_time <= (track_x/track_vx);
}

int main(int, char**)
{
	//Setup egoframe
	Mat egoFrameInitialState = (Mat_<float>(stateDim,1) << 0, 0, 5, 0, 0); //Vehicle at 0,0 moving 5x,0y with 0 yaw
	egoFrame = EgoFrameKalmanFilter(egoFrameInitialState);

	//Setup object tracker
	Mat trackInitialState = (Mat_<float>(stateDim,1) << 5, 5, 0, 0, 10); //Object at 5,5 not moving with a width of 10
	ObjectKalmanFilter track1 = ObjectKalmanFilter(trackInitialState);
	
	int i = 0;

	int ef_x = 0;
	int ef_y = 0;
	int ef_vx = 5;
	int ef_vy = 0;

	int t_x = 0;
	int t_y = 0;
	int t_vx = 5;
	int t_vy = 0;

	while (i < 10) {
		Mat ef_prediction = egoFrame.getPrediction();
		Mat track_prediction = track1.getPrediction();

		FCW_check(ef_prediction, track_prediction);	

		//Update egoFrame and track
		Mat egoFrameMeas = (Mat_<float>(measDim, 1) << ef_x, ef_y, ef_vx, ef_vy);
		Mat trackMeas = (Mat_<float>(measDim, 1) << t_x, t_y, t_vx, t_vy);
		
		egoFrame.update(trackMeas);
		track1.update(trackMeas);
		
		//Manually change location of vehicle and objects
		i++;
		ef_x += 5;
		t_x += 5;
		t_vx -= 1; //Decrease object speed by 1 so that vehicle and object collide eventuall
	}
}
