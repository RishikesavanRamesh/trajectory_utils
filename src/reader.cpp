#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>


void publishMarkers(ros::Publisher& vis_pub, const std::vector<std::pair<float, float>>& points, const std::string& frame_id) {
    visualization_msgs::MarkerArray markerArray;

    for (size_t i = 0; i < points.size(); ++i) {
        const auto& point = points[i];

        visualization_msgs::Marker marker;
        marker.header.frame_id = frame_id; 
        marker.header.stamp = ros::Time::now();
        marker.ns = "trajectory_markers";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point.first;
        marker.pose.position.y = point.second;
        marker.pose.position.z = 0.1; // Assuming z=0 for 2D
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1; // Adjust size as needed
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0; // Fully opaque
        marker.color.r = 0.0; // Red
        marker.color.g = 1.0; // Green
        marker.color.b = 0.0; // Blue

        markerArray.markers.push_back(marker);
    }

    vis_pub.publish(markerArray);
}

std::vector<std::pair<float, float>> readPointsFromFile(const std::string& file_path) {
    std::ifstream file(file_path);
    std::vector<std::pair<float, float>> points;

    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return points; // Return empty vector
    }

    // Parse the JSON content
    nlohmann::json jsonData;
    try {
        file >> jsonData;
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to parse JSON: %s", e.what());
        return points; // Return empty vector
    }

    // Extract points from the JSON array
    for (const auto& entry : jsonData) {
        float x = entry["x"].get<float>();
        float y = entry["y"].get<float>();
        points.emplace_back(x, y);
    }

    return points;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_reader");
    ros::NodeHandle nh("~");

    // Read parameters with defaults
    std::string frame_id;
    std::string topic;
    nh.param("frame_id", frame_id, std::string("/map"));
    nh.param("topic", topic, std::string("/trajectory"));



    ROS_INFO("Frame ID: %s", frame_id.c_str());
    ROS_INFO("Topic: %s", topic.c_str());

    // Publisher for the MarkerArray
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>(topic, 2);  

    if (argc != 2) {
        ROS_ERROR("Usage: rosrun trajectory_utils trajectory_marker_publisher <file_path>");
        return 1;
    }

    std::string file_path = argv[1];
    auto points = readPointsFromFile(file_path);

    ros::Rate loop_rate(10);
    while (ros::ok()) {
        publishMarkers(vis_pub, points, frame_id);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
