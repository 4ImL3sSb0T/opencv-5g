#include <iostream>
#include "config.hpp"
#include "garage.hpp"

int main()
{
    cv::Mat frame;
    auto cap = cv::VideoCapture("../img/WIN_20251103_21_26_41_Pro.mp4");
    // auto cap = cv::VideoCapture(1);
    Garage::initGarage();
    
    while (true)
    {
        auto ret = cap.read(frame);
        if (ret != true) break;
        Garage::Update(frame);
        
    }
}