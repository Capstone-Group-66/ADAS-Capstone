#include "ObjectKalmanFilter.cpp"
#include "opencv2/tracking.hpp"
#include <iostream>

using namespace cv;

int main(int, char **) {
    Mat initialState = (Mat_<float>(stateDim, 1) << 5, 5, 0, 0, 10);
    ObjectKalmanFilter kf = ObjectKalmanFilter(initialState);

    int i = 0;
    int x_position = 0;
    int y_position = 0;
    while (i < 10) {
        kf.getPrediction();

        Mat measurement = (Mat_<float>(measDim, 1) << x_position, y_position, 5, 3);

        kf.update(measurement);

        i++;
        x_position += 5;
        y_position += 3;
    }
}
