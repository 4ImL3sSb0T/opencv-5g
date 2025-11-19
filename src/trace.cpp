#include <iostream>
#include <thread>
#include <string>
#include <pigpio.h>
#include "config.hpp"
#include "cone_detector.hpp"
#include "image_Q.hpp"
#include "garage.hpp"

// ========================== PWM Configuration ==========================
constexpr int MOTOR_PWM_PIN = 13;
constexpr int SERVO_PWM_PIN = 12;
constexpr int GIMBAL_HORIZONTAL_PIN = 22; // 云台水平舵机
constexpr int GIMBAL_VERTICAL_PIN = 23; // 云台垂直舵机

constexpr float PWM_RANGE = 40000.0;
constexpr float PWM_FREQUENCY = 200.0;
constexpr float PWM_DUTY_CYCLE_UNLOCK = 10000.0;
constexpr int SERVO_PWM_FREQUENCY = 50;
constexpr int SERVO_PWM_RANGE = 1000;

// ========================== Control Parameters ==========================
constexpr int MOTOR_MAX_SPEED = 8000;
int SERVO_CENTER_POSITION = 74;
int SERVO_MAX_OFFSET = 10;

// PID Parameters
float PID_KP = 1.0;
float PID_KD = 1.2;
int PID_OUTPUT_MAX = 15;
int PID_ERROR_OFFSET = 19;
int PID_OUTPUT_OFFSET = 80;

// Gimbal
int GIMBAL_HORIZONTAL_CENTER = 74;
int GIMBAL_VERTICAL_CENTER = 69;

// Motor
int MOTOR_TARGET_SPEED = 1000;

enum class CAR_STATE
{
    BEFORE_OBSTACLE,
    BEFORE_ZEBRA,
    BEFORE_GUIDED,
    BEFORE_GARAGE,
    STOP
};

auto current_state = CAR_STATE::BEFORE_OBSTACLE;

// ========================== Global Variables ==========================
int g_stopFlag = 0;
int BANMATIME = 0;

int yuyinflag = 0;
int hdflage = 0;
int times;
int stop_banmaxiankaojin = 0;
int y = 0;

// ========================== Motor Control ==========================
void setMotorSpeed(int speed) {
    // Limit speed range
    speed = -speed;
    if (speed > MOTOR_MAX_SPEED){
        speed = MOTOR_MAX_SPEED;
    }
    if (speed < -MOTOR_MAX_SPEED){
        speed = -MOTOR_MAX_SPEED;
    }

    // Set PWM based on direction
    if (speed < 0){
        // Reverse
        speed = -speed;
        gpioPWM(MOTOR_PWM_PIN, speed + PWM_DUTY_CYCLE_UNLOCK);
    } else{
        // Forward
        gpioPWM(MOTOR_PWM_PIN, PWM_DUTY_CYCLE_UNLOCK - speed);
    }
}

// ========================== Servo Control ==========================
void setServoPosition(int position) {
    // Limit servo position range
    if (position >= SERVO_CENTER_POSITION + SERVO_MAX_OFFSET){
        position = SERVO_CENTER_POSITION + SERVO_MAX_OFFSET;
    }
    if (position <= SERVO_CENTER_POSITION - SERVO_MAX_OFFSET){
        position = SERVO_CENTER_POSITION - SERVO_MAX_OFFSET;
    }

    gpioPWM(SERVO_PWM_PIN, position);
}

// ========================== PWM Initialization ==========================
void initializePWM() {
    // Initialize GPIO
    if (gpioInitialise() < 0){
        std::cout << "GPIO initialization failed for PWM" << std::endl;
        return;
    }
    std::cout << "GPIO PWM initialized successfully" << std::endl;

    // Configure servo PWM
    gpioSetMode(SERVO_PWM_PIN, PI_OUTPUT);
    gpioSetPWMfrequency(SERVO_PWM_PIN, SERVO_PWM_FREQUENCY);
    gpioSetPWMrange(SERVO_PWM_PIN, SERVO_PWM_RANGE);
    gpioPWM(SERVO_PWM_PIN, 70);

    // Configure motor PWM
    gpioSetMode(MOTOR_PWM_PIN, PI_OUTPUT);
    gpioSetPWMfrequency(MOTOR_PWM_PIN, PWM_FREQUENCY);
    gpioSetPWMrange(MOTOR_PWM_PIN, PWM_RANGE);
    gpioPWM(MOTOR_PWM_PIN, PWM_DUTY_CYCLE_UNLOCK);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// ========================== Gimbal Initialization ==========================
void initializeGimbal() {
    // Initialize horizontal servo
    if (gpioInitialise() < 0){
        std::cout << "GPIO initialization failed for gimbal" << std::endl;
        return;
    }
    std::cout << "GPIO gimbal initialized successfully" << std::endl;

    gpioSetMode(GIMBAL_HORIZONTAL_PIN, PI_OUTPUT);
    gpioSetPWMfrequency(GIMBAL_HORIZONTAL_PIN, SERVO_PWM_FREQUENCY);
    gpioSetPWMrange(GIMBAL_HORIZONTAL_PIN, SERVO_PWM_RANGE);
    gpioPWM(GIMBAL_HORIZONTAL_PIN, GIMBAL_HORIZONTAL_CENTER); // Horizontal: larger value = left, smaller = right

    // Initialize vertical servo
    if (gpioInitialise() < 0){
        std::cout << "GPIO initialization failed for gimbal" << std::endl;
        return;
    }
    std::cout << "GPIO gimbal initialized successfully" << std::endl;

    gpioSetMode(GIMBAL_VERTICAL_PIN, PI_OUTPUT);
    gpioSetPWMfrequency(GIMBAL_VERTICAL_PIN, SERVO_PWM_FREQUENCY);
    gpioSetPWMrange(GIMBAL_VERTICAL_PIN, SERVO_PWM_RANGE);
    gpioPWM(GIMBAL_VERTICAL_PIN, GIMBAL_VERTICAL_CENTER); // Vertical: larger value = up, smaller = down
}
void banmachuli(void) {
    if (stopbanma == 1) {
        times = BANMATIME;
        stopbanma = 2;
    }
    if (stopbanma == 2) // 停车保持打角正常
    {
        if (stop_banmaxiankaojin == 0) {
            //  gpioPWM(13,10200);
            //  gpioPWM(13,14200);
            MOTOR_TARGET_SPEED = 2000;
            setServoPosition(70);
            std::this_thread::sleep_for(
                std::chrono::milliseconds(2500)); // 延时 1000 毫

            times = BANMATIME;
            stop_banmaxiankaojin = 1;
        }
        if (banmaxian_Y < 45) {
            MOTOR_TARGET_SPEED = 1200;
            times = BANMATIME;
            std::cout << "times =" << BANMATIME << std::endl;
        }

        else
            MOTOR_TARGET_SPEED = 2000;
        std::cout << "times =" << BANMATIME << std::endl;
        std::cout << "stopbanma =" << stopbanma << std::endl;
        std::cout << "banmaxian_Y =" << banmaxian_Y << std::endl;
        std::cout << "stop_banmaxiankaojin =" << stop_banmaxiankaojin << std::endl;

        if (BANMATIME - times > 1) {
            stopbanma = 200;
            MOTOR_TARGET_SPEED = 0;
            y = BANMATIME;
            stop_banmaxiankaojin = 0;
            hdflage = 1;
            yuyinflag = 2; // 2是允许
            std::cout << "现在允许语音了" << std::endl;
        }
    }
}
// ========================== PID Control ==========================
void pidControl(int error) {
    static int lastError = 0;

    // Apply error offset
    error = error - PID_ERROR_OFFSET;
    std::cout << "Tracking error: " << error << std::endl;
    // Calculate PID output
    if (MOTOR_TARGET_SPEED < 1500) {
        PID_KP = 1.2;
        PID_KD = 1.2;
    } else if (MOTOR_TARGET_SPEED == 2000) {
        PID_KP = 1.0;
        PID_KD = 1.2;
    } else if (MOTOR_TARGET_SPEED == 2500) {
        PID_KP = 0.85;
        PID_KD = 1.2;
    } else if (MOTOR_TARGET_SPEED == 3700) {
        PID_KP = 0.59;
        PID_KD = 0.9;
    }
    int output = -(PID_KP * error + PID_KD * (error - lastError));

    // Limit output range
    if (output < -PID_OUTPUT_MAX){
        output = -PID_OUTPUT_MAX;
    }
    if (output > PID_OUTPUT_MAX){
        output = PID_OUTPUT_MAX;
    }

    // Apply to servo (positive = left, negative = right)
    setServoPosition(output + PID_OUTPUT_OFFSET);

    lastError = error;
}

// ========================== Main Run Function ==========================
int run() {
    // 加载配置文件
    const auto ret = Config::load_config("../config/config.json");
    std::cout << "Config load status: " << ret << std::endl;
    if (!ret){
        std::cout << "Failed to load configuration file!" << std::endl;
        return -1;
    }
    auto config = Config::get_config();
    try{
        PID_KP = config["pid"]["kp"].get<float>();
        PID_KD = config["pid"]["kd"].get<float>();
        PID_OUTPUT_MAX = config["pid"]["output_limit"].get<int>();
        PID_ERROR_OFFSET = config["pid"]["error_offset"].get<int>();
        PID_OUTPUT_OFFSET = config["pid"]["output_offset"].get<int>();

        SERVO_CENTER_POSITION = config["servo"]["center_position"].get<int>();
        SERVO_MAX_OFFSET = config["servo"]["max_offset"].get<int>();

        GIMBAL_HORIZONTAL_CENTER = config["gimbal"]["horizontal_center"].get<int>();
        GIMBAL_VERTICAL_CENTER = config["gimbal"]["vertical_center"].get<int>();

        MOTOR_TARGET_SPEED = config["motor"]["target_speed"].get<int>();

        std::cout << "PID_KP " << PID_KP << std::endl;
        std::cout << "PID_KD " << PID_KD << std::endl;
        std::cout << "PID_OUTPUT_MAX " << PID_OUTPUT_MAX << std::endl;
        std::cout << "PID_ERROR_OFFSET " << PID_ERROR_OFFSET << std::endl;
        std::cout << "PID_OUTPUT_OFFSET " << PID_OUTPUT_OFFSET << std::endl;
        std::cout << "SERVO_CENTER_POSITION " << SERVO_CENTER_POSITION << std::endl;
        std::cout << "SERVO_MAX_OFFSET " << SERVO_MAX_OFFSET << std::endl;
        std::cout << "GIMBAL_HORIZONTAL_CENTER " << GIMBAL_HORIZONTAL_CENTER << std::endl;
        std::cout << "GIMBAL_VERTICAL_CENTER " << GIMBAL_VERTICAL_CENTER << std::endl;
        std::cout << "MOTOR_TARGET_SPEED " << MOTOR_TARGET_SPEED << std::endl;
    } catch (std::exception& e){
        std::cout << "Config Error :" << e.what() << std::endl;
    }


    // Initialize hardware
    gpioTerminate();
    initializeGimbal();
    initializePWM();

    // Initialize camera
    cv::VideoCapture capture;
#ifdef _DEBUG
    const auto videoPath = config["vision"]["path"].get<std::string>();
    capture.open(videoPath);
#else
    capture.open(0);
#endif

    if (!capture.isOpened()){
        std::cout << "Failed to open video capture!" << std::endl;
        return -1;
    }

    // Wait for camera to stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    cv::Mat frame;

    Garage::initGarage();
    const double cone_min_area = config["vision"]["cone_detection"]["min_area"].get<double>();
    const double cone_max_area = config["vision"]["cone_detection"]["max_area"].get<double>();
    const double tracking_distance = config["vision"]["cone_detection"]["tracking_distance_threshold"].get<double>();
    const int max_disappeared = config["vision"]["cone_detection"]["max_disappeared_frames"].get<int>();

    // 初始化锥桶检测器
    ConeDetector::initConeDetector(Garage::yellow_low, Garage::yellow_high, cone_min_area, cone_max_area);
    ConeDetector::detection_params.tracking_distance_threshold = tracking_distance;
    ConeDetector::detection_params.max_disappeared_frames = max_disappeared;

    ConeDetector::initRedConeDetector(
        cv::Scalar(0, 100, 100),    // HSV 下限
        cv::Scalar(10, 255, 255)    // HSV 上限
    );

    while (true){
        static auto start = std::chrono::steady_clock::now();
        // Capture frame
        if (!capture.read(frame)){
            std::cout << "Failed to read frame from video capture!" << std::endl;
            break;
        }

#ifdef _DEBUG
        auto draw_frame = frame.clone();
#endif

        // Process image and get tracking error
        const int trackingError = TUxiang_Init(frame);
        // std::cout << "Tracking error: " << trackingError - PID_ERROR_OFFSET << std::endl;

        // Set motor speed
        switch (current_state){
        case CAR_STATE::BEFORE_OBSTACLE:
            {
                // 这个过程正常循迹但是开启避障功能
                const auto error = TUxiang_Init(frame);
                pidControl(error);
                break;
            }
        case CAR_STATE::BEFORE_ZEBRA:
            {
                const auto error = TUxiang_Init(frame);
                pidControl(error);
                banmachuli();
                break;
            }
        case CAR_STATE::BEFORE_GUIDED:
            {
                // 开启引导区功能
                auto cones = ConeDetector::detectCones(frame);
                auto red_cones = ConeDetector::detectRedCones(frame);
                ConeDetector::drawDetectedRedCones(draw_frame);
                ConeDetector::drawDetectedCones(draw_frame);
                if (!cones.empty()) {
                    // 进入引导区
                    // 还得判断左右
                    setMotorSpeed(1200);
                    setServoPosition(70);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    setServoPosition(70 + 15);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    setServoPosition(70 - 15);
                    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
                    current_state = CAR_STATE::BEFORE_GARAGE;
                }
                break;
            }
        case CAR_STATE::BEFORE_GARAGE:
            {
                // 开启车库检测
                break;
            }
        case CAR_STATE::STOP:
            {
                // 完赛!停车!
                setMotorSpeed(0);
                break;
            }
        default:
            break;
        }

        // 测试功能

        const auto error = ConeDetector::getError();
        // pidControl(trackingError);
        pidControl(error);
        // setMotorSpeed(MOTOR_TARGET_SPEED);

#ifdef _DEBUG
        cv::imshow("draw", draw_frame);
        if (cv::waitKey(30) == 27) break;
#endif
    }

    // Cleanup
    capture.release();
    cv::destroyAllWindows();

    return 0;
}

// ========================== Main Entry Point ==========================
int main() {
    std::thread mainThread(run);
    mainThread.join();
    return 0;
}
