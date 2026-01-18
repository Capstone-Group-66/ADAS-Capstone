#include "adas/stage_e/SensorFusion.hpp"

#ifdef __linux__
#include <unistd.h>
#endif

#include <csignal>
#include <iostream>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <thread>

namespace adas {

SensorFusion::SensorFusion( SPSCQueue<RadarTargets, 8>& frontRadarQueue,
                     SPSCQueue<ImuSample, 32>& imuQueue)
    : FrontRadarQueue(frontRadarQueue), IMUQueue(imuQueue) {}

SensorFusion::~SensorFusion() { stop(); }

void SensorFusion::start(){
    if (running_.load(std::memory_order_relaxed)) {
        return;
    }

    running_.store(true, std::memory_order_relaxed);
    thread_ = std::thread(&SensorFusion::run, this);
}

void SensorFusion::stop(){
    running_.store(false, std::memory_order_relaxed);

#ifdef __linux__
    if (fd_ >= 0) {
        close(fd_);
        fd_ = -1;
    }
#endif

    if (thread_.joinable()) {
        thread_.join();
    }
}

void SensorFusion::run(){
    std::cout << "[SensorFusion] Starting " << std::endl;

    healthy_.store(true, std::memory_order_relaxed);

    cv::Mat egoFrameInitialState = (cv::Mat_<float>(5,1) << 0, 0, 0, 0, 0); //Replace
    egoFrame = adas::EgoFrame(egoFrameInitialState);

    while (running_.load(std::memory_order_relaxed)) {
        //updateEgoFrame(); //No change at the moment
        
        updateFrontTrack();
        
        detectFCW(egoFrame.getPrediction(), frontTrack.getPrediction());

        // //Wait 50ms for the 20GHz
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void SensorFusion::updateFrontTrack(){
    std::optional<adas::RadarTargets> radarTargets;
    if (auto radarTargets = FrontRadarQueue.try_pop()) {    
        if(radarTargets.has_value()){
            //Process radar targets
            float sum_speed = 0.0f;
            float avg_speed = 0.0f;
            float final_range = 0.0f;
            float dt = 0.05;

            //Get average speed of targets and the final distance of the object
            for (adas::RadarTarget target : radarTargets->targets){
                sum_speed += target.radial_vel_mps;
            }
            avg_speed = sum_speed / radarTargets->targets.size();
            final_range = radarTargets->targets.back().range_m;

            if(frontTrack.previous_time_ns != 0){
                dt = adas::Clock::ns_to_sec(radarTargets->h.t_ingest_ns - frontTrack.previous_time_ns);
            }
            frontTrack.previous_time_ns = radarTargets->h.t_ingest_ns;
            if(avg_speed <= 0.20 && avg_speed >= -0.02 && final_range <= 2.42 && final_range >= 2.38){//TODO redo this based off actual sensor data
                if(frontTrack.object_detected){
                    frontTrack = adas::Track();
                    std::cout << "[SensorFusion] Front Track Reset" << std::endl;
                }
            }else{

                if(!frontTrack.kf_initialized){
                    frontTrack = adas::Track((cv::Mat_<float>(4,1) << final_range, 0.0f, avg_speed, 0.0f));
                    std::cout << "[SensorFusion] Front Track Initialized:" << frontTrack.getPrediction().reshape(1, 1) << std::endl; 
                }else{
                    cv::Mat estimate = frontTrack.update((cv::Mat_<float>(4,1) << final_range, 0.0f, avg_speed, 0.0f),dt);
                    std::cout << "[SensorFusion] Front Track Updated:" << estimate.reshape(1, 1) << std::endl; 
                }
                std::cout << "Avg Speed: " << avg_speed << " m/s, Final Range: " << final_range << " m" << std::endl;
            }
        }
    }
}

void SensorFusion::updateEgoFrame(){
    adas::ImuSample imuSample;
    float dt = 0.05;

    //Some random stuff that can kinda get speed and distance from imu since we don't have gps
    if (auto imuSample = IMUQueue.try_pop()) {  
        std::cout << "[SensorFusion] Updating EgoFrame with IMU" << std::endl;  
        if(imuSample.has_value()){
            //Process radar targets
            float speed_x = 0.0f;
            float distance_x = 0.0f;

            if(egoFrame.previous_time_ns != 0){
                dt = adas::Clock::ns_to_sec(imuSample->h.t_ingest_ns - egoFrame.previous_time_ns);
            }
            speed_x = egoFrame.getPrediction().at<float>(2,2) + (imuSample->acc_mps2[0] * dt); //Replace
            egoFrame.previous_time_ns = imuSample->h.t_ingest_ns;
            distance_x = egoFrame.getPrediction().at<float>(0,0) + (speed_x * dt); //Replace
            egoFrame.update((cv::Mat_<float>(4,1) << distance_x, 0.0f, speed_x, 0.0f),dt);

            std::cout << "[SensorFusion] EgoFrame updated:" << egoFrame.getPrediction().reshape(1, 1) << std::endl; 
        }
    }
}

void SensorFusion::detectFCW(cv::Mat ef, cv::Mat track){
    if (ef.empty() || track.empty()) {
        return;
    }

    if (ef.rows < 4 || ef.cols < 1 || track.rows < 4 || track.cols < 1) {
        return;
    }

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

    float collision_time = (track_x - ef_x) / (track_vx + ef_vx) ;

    if(stop_time >= collision_time && collision_time >= 0){
        std::cout << "[FCW] Send Alert" << std::endl;
        
        //TODO add ble stuff
    }else{
        //Safe
        //Probably still need to send something
    }
    
    // std::cout << "Prediction: " << std::endl;
    // std::cout << "ef_x: " << ef_x << std::endl;
    // std::cout << "ef_vx: " << ef_vx << std::endl;
    // std::cout << "Stop time: " << stop_time << std::endl;
    
    // std::cout << "track_x: " << track_x << std::endl;
    // std::cout << "track_vx: " << track_vx << std::endl;
    // std::cout << "Collision time: " << collision_time << std::endl << std::endl;
}
}