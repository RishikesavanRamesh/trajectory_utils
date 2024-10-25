#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <array>
#include "trajectory_utils/Save.h"  // Ensure this matches your generated header
#include <fstream> 

#include <nlohmann/json.hpp> // Include JSON library
#include <chrono> // For timestamping
#include <cmath> // For math functions

const int BUFFER_SIZE = 10000;

class OdomBuffer {
public:
    OdomBuffer(const std::string& basePath) : index(0), count(0), base_file_path(basePath) {
        buffer.fill(nav_msgs::Odometry());
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // Update currentOdom with the latest message
        currentOdom = *msg;  // Store the latest odometry data
    }

    void timerCallback(const ros::TimerEvent&) {
        // Sample currentOdom and store in buffer at 2 Hz
        if (count < BUFFER_SIZE) {
            buffer[index] = currentOdom;  // Store a copy of currentOdom
            count++;  // Increase the count until it reaches BUFFER_SIZE
        } else {
            buffer[index] = currentOdom;  // Replace oldest entry
        }
        index = (index + 1) % BUFFER_SIZE;  // Update index circularly
    }


bool saveTrajectory(trajectory_utils::Save::Request &req,
                    trajectory_utils::Save::Response &res) {
    ROS_INFO("Request to save trajectory received.");
    ROS_INFO("Filename: [%s], Duration: [%d]", req.filename.c_str(), req.duration);

    // Determine the full path for the file
    std::string file_path = base_file_path + req.filename + ".trajectory";  
    
    // Open the file in write mode
    std::ofstream file(file_path);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s. Check if the directory exists and permissions are set.", file_path.c_str());
        res.success = false;
        return true;  // Return true to indicate the service call was handled
    }

    // Create a JSON array to hold the points
    nlohmann::json jsonArray = nlohmann::json::array();

    // Print the odometry values in the buffer, limited by the duration
    ROS_INFO("Odometry values in buffer (last %d entries):", req.duration);
    int entries_to_print = std::min(static_cast<int>(req.duration), count);  // Ensure we don't exceed the count

    for (int i = 0; i < entries_to_print; ++i) {
        int idx = (index - 1 - i + BUFFER_SIZE) % BUFFER_SIZE; // Access buffer backwards
        const nav_msgs::Odometry& odom = buffer[idx];

        // Create a JSON object for the current entry
        nlohmann::json jsonEntry;
        jsonEntry["x"] = odom.pose.pose.position.x;
        jsonEntry["y"] = odom.pose.pose.position.y;
        jsonEntry["timestamp"] = odom.header.stamp.toSec(); // Add timestamp in seconds

        // Add the JSON object to the array
        jsonArray.push_back(jsonEntry);

        // Optional: Print to console for verification
        ROS_INFO("Buffer[%d]: x: [%f], y: [%f], timestamp: [%f]", 
                 idx, odom.pose.pose.position.x, odom.pose.pose.position.y, jsonEntry["timestamp"].get<double>());
    }

    // Write the JSON array to the file
    file << jsonArray.dump(4);  // Pretty print with 4 spaces for indentation

    // Close the file
    file.close();

    // Indicate success
    res.success = true;
    ROS_INFO("Successfully saved trajectory to %s", file_path.c_str());
    return true;  // Return true to indicate the service call was handled
}



private:
    std::array<nav_msgs::Odometry, BUFFER_SIZE> buffer;
    nav_msgs::Odometry currentOdom; // Declare currentOdom here
    int index; 
    int count; 
    std::string base_file_path;
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_saver");
    ros::NodeHandle nh("~");  // Use private namespace

    // Default path
    std::string base_file_path = "/saved_trajectories/";

    // Get the parameter; if it doesn't exist, the default value will be used
    nh.param<std::string>("base_file_path", base_file_path, base_file_path);


    if (base_file_path.back() != '/') {
        base_file_path += '/';  // Ensure it ends with a slash
    }

    ROS_INFO("Base file path: %s", base_file_path.c_str());

    OdomBuffer odomBuffer(base_file_path);
    ros::Subscriber sub = nh.subscribe("/odom", 1000, &OdomBuffer::odomCallback, &odomBuffer);

    // Create a timer to sample odometry data at 2 Hz
    ros::Timer timer = nh.createTimer(ros::Duration(0.5), &OdomBuffer::timerCallback, &odomBuffer);

    ros::ServiceServer service = nh.advertiseService(
        "save_trajectory",
        &OdomBuffer::saveTrajectory,
        &odomBuffer
    );

    ros::spin();  // Keep the node running
    return 0;
}
