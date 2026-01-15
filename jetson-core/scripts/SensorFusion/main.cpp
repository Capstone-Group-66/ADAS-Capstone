#include "opencv2/tracking.hpp"
#include <iostream>
#include "Track.cpp"
#include "EgoFrame.cpp"

using namespace cv;

bool FCW_check(Mat ef, Mat track){
	float ef_x = ef.at<float>(0,0);
	float track_x = track.at<float>(0,0);
	
	// float ef_y = ef.at<float>(0,1);
	// float track_y = track.at<float>(0,1);

	float ef_vx = ef.at<float>(0,2);
	float track_vx = track.at<float>(0,2);	

	// float ef_vy = ef.at<float>(0,3);
	// float track_vy = track.at<float>(0,3);
	
	// stope_time = vehicle_speed / (friction_coefficient * gravity) + reaction_time
	float stop_time = ef_vx / (0.7 * 9.81) + 2.5;
	
	float collision_time = (track_x - ef_x) / (track_vx + ef_vx);
	
	std::cout << "Prediction: " << std::endl;
	std::cout << "ef_x: " << ef_x << std::endl;
	std::cout << "ef_vx: " << ef_vx << std::endl;
	std::cout << "Stop time: " << stop_time << std::endl;
	
	std::cout << "track_x: " << track_x << std::endl;
	std::cout << "track_vx: " << track_vx << std::endl;
	std::cout << "Collision time: " << collision_time << std::endl << std::endl;
	
	return stop_time >= collision_time;
}

int main(int, char**)
{	
	float ef_x = 0;
	float ef_y = 0;
	float ef_vx = 5;
	float ef_vy = 0;
	float ef_yaw = 0;

	float t_x = 100;
	float t_y = 0;
	float t_vx = 1;
	float t_vy = 0;
	float t_w = 10;
	
	//Setup egoframe
	Mat egoFrameInitialState = (Mat_<float>(5,1) << ef_x, ef_y, ef_vx, ef_vy, ef_yaw); //Vehicle at 0,0 moving 5x,0y with 0 yaw
	EgoFrame egoFrame = EgoFrame(egoFrameInitialState);

	//Setup object tracker
	Mat trackInitialState = (Mat_<float>(5,1) << t_x, t_y, t_vx, t_vy, t_w); //Object at 5,5 not moving with a width of 10
	Track track1 = Track(trackInitialState);

	int i = 0;
	bool collision = false;
	while (!collision) {
		Mat ef_prediction = egoFrame.getPrediction();
		Mat track_prediction = track1.getPrediction();

		if(FCW_check(ef_prediction, track_prediction)){
			std::cout << "FCW alert" << std::endl;
			collision = true;
		}	

		//Update egoFrame and track
		Mat egoFrameMeas = (Mat_<float>(4, 1) << ef_x, ef_y, ef_vx, ef_vy);
		Mat trackMeas = (Mat_<float>(4, 1) << t_x, t_y, t_vx, t_vy);
		
		egoFrame.update(egoFrameMeas, 0.05);
		track1.update(trackMeas, 0.05);
		
		//Manually change location of vehicle and objects
		i++;
		ef_x += ef_vx;
		t_x += t_vx;
		//t_vx -= 1; //Decrease object speed by 1 so that vehicle and object collide eventuall
	}
}
