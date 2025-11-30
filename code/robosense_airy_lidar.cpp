#include "robosense_airy_lidar.h"
#include <iostream>

bool RoboSenseAiryLidar::startListener(const std::string& ip) {
    (void)ip; // Airy игнорирует IP, шлёт на broadcast

    robosense::lidar::DriverParam param;
    param.input_type = robosense::lidar::InputType::RAW_PACKET;
    param.input_param.msop_port = 6699;
    param.input_param.difop_port = 7788;
    param.lidar_type = robosense::lidar::LidarType::RS_AIRY;

    driver_ = std::make_shared<robosense::lidar::Driver>();

    if (!driver_->init(param)) {
        std::cerr << "[Airy] Init failed!" << std::endl;
        return false;
    }

    driver_->regPointCloudCallback([this](const robosense::lidar::PointCloudConstPtr& cloud) {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto& p : cloud->points) {
            LivoxExtendRawPoint lp{};
            lp.x = p.x;
            lp.y = p.y;
            lp.z = p.z;
            lp.reflectivity = static_cast<uint8_t>(p.intensity);
            lp.tag = 0;
            lp.offset_time = 0;
            lp.line = static_cast<uint8_t>(p.ring); // ring в line — бонус для SLAM
            points_buffer_.push_back(lp);
        }
        received_packets_ += cloud->points.size();
    });

    driver_->regImuPacketCallback([this](const robosense::lidar::ImuPacket& imu) {
        std::lock_guard<std::mutex> lock(mutex_);
        LivoxImuPoint ip{};
        ip.gyro_x = imu.gyro_x;
        ip.gyro_y = imu.gyro_y;
        ip.gyro_z = imu.gyro_z;
        ip.acc_x = imu.acc_x;
        ip.acc_y = imu.acc_y;
        ip.acc_z = imu.acc_z;
        imu_buffer_.push_back(ip);
    });

    driver_->start();

    std::cout << "[RoboSense Airy] Started — waiting for data..." << std::endl;
    return true;
}

std::pair<LivoxPointsBufferPtr, LivoxIMUBufferPtr> RoboSenseAiryLidar::retrieveData() {
    std::lock_guard<std::mutex> lock(mutex_);
    auto points = std::make_shared<std::vector<LivoxExtendRawPoint>>(std::move(points_buffer_));
    auto imu = std::make_shared<std::vector<LivoxImuPoint>>(std::move(imu_buffer_));
    points_buffer_.clear();
    imu_buffer_.clear();
    return {points, imu};
}