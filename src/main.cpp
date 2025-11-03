#include <iostream>
#include "config.hpp"
#include "garage.hpp"

int main()
{
    cv::Mat frame;
    auto cap = cv::VideoCapture("../img/camera_record1.mp4");
    Garage::initGarage();
    while (true)
    {
        auto ret = cap.read(frame);
        if (ret != true) break;
        Garage::Update(frame);
        
    }
}