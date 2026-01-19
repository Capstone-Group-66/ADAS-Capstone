#include <iostream>
#include <opencv2/core.hpp>

int main() {
    cv::Mat m = cv::Mat::eye(3, 3, CV_32F);
    std::cout << "OpenCV OK, matrix(0,0) = " << m.at<float>(0, 0) << "\n";
    return 0;
}
