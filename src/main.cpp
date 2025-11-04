#include <iostream>
#include "config.hpp"
#include "garage.hpp"

int main()
{
    cv::Mat frame;
    if (!Config::load_config("../config/config.json"))
    {
        std::cout << "Error loading config.json" << std::endl;
        return -1;
    }
    auto config = Config::get_config();
    std::string path;
    try {
        path = config["vision"]["path"].get<std::string>();
    } catch (std::exception& e) {
        std::cout << "Error loading value" << std::endl;
        return -1;
    }
    auto cap = cv::VideoCapture(path);
    // auto cap = cv::VideoCapture(1);
    Garage::initGarage();
    
    while (true)
    {
        auto ret = cap.read(frame);
        if (ret != true) break;
        Garage::Update(frame);
        
    }
}