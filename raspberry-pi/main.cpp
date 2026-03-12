#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <ncnn/net.h>

using namespace cv;
//Real sizes all in m {height, width}
float real_sizes[6][2] = {
  {1.7f, 0.5f}, // 0 Person
  {1.1f, 0.5f}, // 1 Bicycle
  {1.5f, 1.8f}, // 2 Car
  {1.2f, 0.9f}, // 3 Motorcycle
  {3.3f, 2.5f}, // 4 Bus
  {3.0f, 2.4f} // 5 Truck
};

// Camera details
const Size CAMERA_SIZE =  Size(1080, 720);
const float CAMERA_FOCAL_PX = 828.7524f;
Mat camera_matrix = (Mat_<float>(3,3) << 828.7524f, 0.0f, 606.7092f, 0.0f, 829.1880f, 397.7422f, 0.0f, 0.0f, 1.0f);
Mat dist_coeffs = (Mat_<float>(1,5) << -0.4554f, 0.2687f, 0.0008772f, 0.0004815f, -0.09284f);
std::string video_path = "videos/CameraFront.mp4";
//std::string video_path = "/dev/video0";

// ncnn model files
std::string param_path = "models/yolo11n_ncnn_320_model/model.ncnn.param";
std::string bin_path = "models/yolo11n_ncnn_320_model/model.ncnn.bin";
Size model_size = Size(320, 320);

const float IOU_THRESHOLD = 0.2f;
const float CONFIDENCE_THRESHOLD = 0.35;
const float NMS_THRESHOLD = 0.2f;

// set draw = true to show image with boxes
bool draw = false;

struct Detection {
    int class_id;
    float confidence;
    Rect box;
};

struct Track {
    int class_id;
    Rect last_box;

    int missing_frames = 0;

    float long_distance_m;
    float lat_distance_m;
    float long_speed_mps = 0.0f;
    float lat_speed_mps = 0.0f;
    float ttc = -1.0f;
    
    std::chrono::steady_clock::time_point last_time;

    Track(const Detection& detection) {
        class_id = convertClassId(detection.class_id);
        last_box = detection.box;
        long_distance_m = estimateLongDistance(last_box);
        lat_distance_m = estimateLatDistance(last_box, long_distance_m);
        last_time = std::chrono::steady_clock::now();
    }

    // Matches class id from model to real sizes
    int convertClassId(int id){
        if(id == 5) return 4;
        if(id == 7) return 5;
        return id;
    }

    float estimateLongDistance(const Rect& box){
        float long_h = (CAMERA_FOCAL_PX * real_sizes[class_id][0]) / box.height;
        float long_w = (CAMERA_FOCAL_PX * real_sizes[class_id][1]) / box.width;
        return std::sqrt(long_h * long_w);
    }

    float estimateLatDistance(const Rect& box, float long_dist){
        float centroid_x = box.x + (box.width * 0.5f);
        return ((centroid_x - CAMERA_SIZE.width * 0.5f) * long_dist) / CAMERA_FOCAL_PX; 
    }

    void update(Detection& detection){
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_time).count();
        if(dt <= 0) return;

        float new_long = estimateLongDistance(detection.box);
        float new_lat= estimateLatDistance(detection.box, new_long);
        
        if(std::abs(new_long - long_distance_m) > 5) {
            return;
        }
    
        long_speed_mps = (new_long - long_distance_m) / dt;
        lat_speed_mps = (new_lat - lat_distance_m) / dt;

        long_distance_m = new_long;
        lat_distance_m = new_lat;
        last_time = now;
        last_box = detection.box;
        missing_frames = 0;

        if (long_speed_mps < -0.1f) {
            ttc = std::abs(long_distance_m / long_speed_mps);
        } else {
            ttc = -1.0f;
        }
    }
};

float calculate_iou(const cv::Rect& a, const cv::Rect& b) {
    float intersection_area = (a & b).area();
    float union_area = a.area() + b.area() - intersection_area;
    return (union_area > 0) ? (intersection_area / union_area) : 0;
}

void updateTracks(std::vector<Track>& tracks, std::vector<Detection>& detections){
    // Update tracks with detections that match best
    std::vector<bool> used_detection(detections.size(), false);
    for (auto& track : tracks) {
        float max_iou = 0.0;
        int best_id = -1;

        for (int i = 0; i < detections.size(); i++) {
            float iou = calculate_iou(track.last_box, detections[i].box);
            
            if (iou > max_iou) {
                max_iou = iou;
                best_id = i;
            }
        }

        if (max_iou > IOU_THRESHOLD) {
            track.update(detections[best_id]);
            used_detection[best_id] = true;
        } else {
            track.missing_frames++;
        }
    }

    // Remove tracks that have missed more than 5 frames
    tracks.erase(std::remove_if(tracks.begin(), tracks.end(),
        [](const Track& track) {return track.missing_frames > 5;}), tracks.end());

    // Create new tracks for detections that still remain
    tracks.reserve(tracks.size() + detections.size());
    for (int i = 0; i < detections.size(); i++) {
        if (!used_detection[i]) {
            tracks.emplace_back(detections[i]);
        }
    }
}

// Take output from inference and create list of Detections
std::vector<Detection> postProcess(ncnn::Mat& output) {
    std::vector<Detection> detections;

    // For NMS
    std::vector<Rect> boxes;
    std::vector<float> confidences;

    //person, bicycle, car, motorcycle, bus, truck
    const std::set<int> target_classes = {0, 1, 2, 3, 5, 7};

    // Create detections from boxes identified in model
    for (size_t i = 0; i < output.w; ++i) {
        float max_score = 0;
        int class_id = 0;
        
        for (int j = 4; j < 84; ++j) {
            float score = output.row(j)[i];
            if (score > max_score){
                max_score = score;
                class_id = j - 4;
            }
        }

        if (max_score > CONFIDENCE_THRESHOLD && !(target_classes.find(class_id) == target_classes.end())){
            float cx = output.row(0)[i] * CAMERA_SIZE.width / model_size.width;
            float cy = output.row(1)[i] * CAMERA_SIZE.height / model_size.height;
            float w = output.row(2)[i] * CAMERA_SIZE.width / model_size.width;
            float h = output.row(3)[i] * CAMERA_SIZE.height / model_size.height;

            Rect2f box = Rect2f(cx - w/2, cy - h/2, w, h);
            Detection det = {class_id, max_score, box};
            detections.push_back(det);
            boxes.push_back(box);
            confidences.push_back(max_score);
        }
    }

    // Remove overlapping detection
    std::vector<Detection> nmsDetections;
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, CONFIDENCE_THRESHOLD, NMS_THRESHOLD, indices);
    for (int id : indices) {
        nmsDetections.push_back(detections[id]);
    }

    return nmsDetections;
}

// Get the earliest ttc from tracks
float computeTTC(std::vector<Track>& tracks){
    float earliest_ttc = -1.0f;
    for (auto& track : tracks) {
        if (track.ttc > 0.0f){
            if (earliest_ttc == -1.0f) {
                earliest_ttc = track.ttc;
            } else if (track.ttc < earliest_ttc){
                earliest_ttc = track.ttc;
            }
        }
    }
    return earliest_ttc;
}

int main(){
    ncnn::Net net;

    net.load_param(param_path.c_str());
    net.load_model(bin_path.c_str());

    VideoCapture cap(video_path);
    if(!cap.isOpened()){
        std::cerr << "Error opening video stream" << std::endl;
        return -1;
    }

    float camera_fps = cap.get(CAP_PROP_FPS);

    const float norm_vals[3] = {1/255.0f, 1/255.0f, 1/255.0f};

    // Variables for undistorting
    Mat map1, map2;
    initUndistortRectifyMap(
        camera_matrix, dist_coeffs,
        Mat(),
        camera_matrix,
        CAMERA_SIZE,
        CV_16SC2,
        map1, map2
    );

    Mat frame;
    std::vector<Track> tracks;
    auto start = std::chrono::steady_clock::now();
    while(cap.isOpened()){
        // Skip frames to keep system real time
        auto elapsed_time = std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
        int target_frame_id = (int)(elapsed_time * camera_fps);
        int current_id  = (int)cap.get(CAP_PROP_POS_FRAMES);
        while (current_id < target_frame_id){
            if (!cap.grab()) break;
            current_id++;
        }

        if(cap.retrieve(frame)){
            if(frame.empty()) break;
            
            // Preprocess
            // Undistort image
            Mat undistorted_frame;
            remap(frame, undistorted_frame, map1, map2, INTER_LINEAR);

            ncnn::Mat input = ncnn::Mat::from_pixels_resize(undistorted_frame.data, ncnn::Mat::PIXEL_BGR, undistorted_frame.cols, undistorted_frame.rows, model_size.width, model_size.height);
            input.substract_mean_normalize(0, norm_vals);
            ncnn::Extractor ex = net.create_extractor();
            ex.input("in0", input);

            // Inference
            ncnn::Mat output;
            ex.extract("out0", output);

            // Postprocess
            std::vector<Detection> detections = postProcess(output);
            updateTracks(tracks, detections);

            float ttc = computeTTC(tracks);
            if(ttc < 2.5 && ttc > 0) {
                std::cout << "1,2" << std::endl; //true,on //TODO
            } else {
                std::cout << "0,2" << std::endl; //false,on //TODO
            }
            
            // Draw tracks
            if (draw){
                for(const auto& track : tracks){
                    rectangle(undistorted_frame, track.last_box, Scalar(0, 255, 0), 2);
                    putText(undistorted_frame, std::to_string(track.class_id), Point(track.last_box.x, track.last_box.y - 5), FONT_HERSHEY_SIMPLEX, 0.5f, Scalar(0, 255, 0), 1);
                }

                imshow("YOLO11n ncnn", undistorted_frame);
                
                if(waitKey(1) == 27){
                    break;
                }
            }
        }
    }
    cap.release();
    if(draw){
        destroyAllWindows(); 
    }  
    return 0;
}