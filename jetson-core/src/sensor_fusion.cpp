#include "opencv2/tracking.hpp"
#include <iostream>
#include "adas/stage_e/Track.hpp"
#include "adas/stage_e/EgoFrame.hpp"
#include "adas/common/Types.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

bool FCW_check(cv::Mat ef, cv::Mat track){
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
    std::ifstream file("test_files/omnipresense_radar_log.csv");
    std::string line;
    
    if (!file.is_open()) {
        std::cerr << "Cannot open file\n";
        return 1;
    }
    
	float ef_x = 0;
	float ef_y = 0;
	float ef_vx = 0;
	float ef_vy = 0;
	float ef_yaw = 0;

	float t_y = 0;
	float t_vy = 0;
	float t_w = 0;
	
	int previous_time = 0;

	bool initial = true;
	adas::RadarTargets radarTargets;

	//Setup egoframe
	cv::Mat egoFrameInitialState = (cv::Mat_<float>(5,1) << ef_x, ef_y, ef_vx, ef_vy, ef_yaw); //Vehicle at 0,0 moving 5x,0y with 0 yaw
	adas::EgoFrame egoFrame = adas::EgoFrame(egoFrameInitialState);

    std::getline(file, line); //skip header line
	while (std::getline(file, line)) {
        std::stringstream ss(line);	
        std::string cell;

		int timestamp = 0;
		std::string type;
		std::string value;

        std::getline(ss, cell, ',');
        timestamp = std::stoi(cell);
        std::getline(ss, cell, ','); //ignore (raw hex)
        std::getline(ss, type, ','); //either mps or m
		type = type.erase(0, 2).erase(type.size()-1, 1);
        std::getline(ss, value, ','); //either distance or speed
        std::getline(ss, cell); //ignore

		if (type == "mps"){
			adas::RadarTarget radarTarget;

			radarTarget.radial_vel_mps = std::stof(value);
			std::cout << "Object Speed: " << radarTarget.radial_vel_mps << std::endl;

			std::getline(file, line);
			std::getline(ss, cell, ',');
			timestamp = std::stoi(cell);
			std::getline(ss, cell, ','); //ignore (raw hex)
			std::getline(ss, type, ','); //either mps or m
			type = type.erase(0, 2).erase(type.size()-1, 1);
			std::getline(ss, value, ','); //either distance or speed
			std::getline(ss, cell); //ignore

        	radarTarget.range_m = std::stof(value);
			std::cout << "Object Distance: " << radarTarget.range_m << std::endl;

			std::cout << "Timestamp: " << timestamp << std::endl;
			std::cout << "Type: " << type << std::endl;
			std::cout << "Range: " << radarTarget.range_m << std::endl;
			
			radarTargets.targets.push_back(radarTarget);

			if (initial) {
				//Setup object tracker
				cv::Mat trackInitialState = (cv::Mat_<float>(5,1) << radarTarget.range_m, t_y, radarTarget.radial_vel_mps, t_vy, t_w);
				adas::Track track1 = adas::Track(trackInitialState);
				initial = false;
			}
		}
		
		cv::Mat ef_prediction = egoFrame.getPrediction();
		cv::Mat track_prediction = track1.getPrediction();

		if(FCW_check(ef_prediction, track_prediction)){
			std::cout << "FCW alert" << std::endl;
		}	

		//Update egoFrame and track
		cv::Mat egoFrameMeas = (cv::Mat_<float>(4, 1) << ef_x, ef_y, ef_vx, ef_vy);
		cv::Mat trackMeas = (cv::Mat_<float>(4, 1) << t_x, t_y, t_vx, t_vy);
		
		egoFrame.update(egoFrameMeas, 0.05);
		track1.update(trackMeas, 0.05);
	}

	file.close();
	return 0;
}