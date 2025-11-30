#pragma once

#include <rslidar_sdk.hpp>
#include <vector>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include <livox_def.h>  // для LivoxExtendRawPoint
#include <livox_sdk2.h> // для LivoxImuPoint

using LivoxPointsBufferPtr = std::shared_ptr<std::vector<LivoxExtendRawPoint>>;
using LivoxIMUBufferPtr = std::shared_ptr<std::vector<LivoxImuPoint>>;

class RoboSenseAiryLidar {
public:
    RoboSenseAiryLidar() = default;

    bool startListener(const std::string& ip = "192.168.1.5");

    bool isSynced() const { return received_packets_ > 10; }

    std::pair<LivoxPointsBufferPtr, LivoxIMUBufferPtr> retrieveData();

    std::unordered_map<uint32_t, std::string> getSerialNumberToLidarIdMapping() const {
        return {{0, "RoboSense-Airy"}};
    }

    nlohmann::json produceStatus() const {
        nlohmann::json j;
        j["model"] = "RoboSense Airy";
        j["packets"] = received_packets_;
        return j;
    }

    void initializeDuration() {}

private:
    std::shared_ptr<robosense::lidar::Driver> driver_;
    std::vector<LivoxExtendRawPoint> points_buffer_;
    std::vector<LivoxImuPoint> imu_buffer_;
    std::mutex mutex_;
    std::atomic<int> received_packets_{0};
};