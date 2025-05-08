#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <derived_object_msgs/ObjectArray.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <shape_msgs/SolidPrimitive.h>
#include <queue>
#include <vector>
#include <iostream>
#include <utility>

using Point = pcl::PointXYZ;
using Pair = std::pair<float, Point>; // (distance, point)

class PointCloudProcessor
{
public:
    PointCloudProcessor()
    {
        ros::NodeHandle nh;
        pointcloud_sub_ = nh.subscribe("/occupied_point", 1, &PointCloudProcessor::pointCloudCallback, this);
        pose_sub_ = nh.subscribe("/vicon/dingo2", 1, &PointCloudProcessor::poseCallback, this);
        object_pub_ = nh.advertise<derived_object_msgs::ObjectArray>("/dingo2/objects", 1);
        filtered_points_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 1);
    }

private:
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher object_pub_;
    ros::Publisher filtered_points_pub_;
    geometry_msgs::PoseStamped current_pose_;

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        current_pose_ = *msg;
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
    {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*cloud_msg, cloud);
        derived_object_msgs::ObjectArray object_array;
        object_array.header = cloud_msg->header;

        pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
        std::vector<Point> filtered_points;

        shape_msgs::SolidPrimitive sphere;
        sphere.type = shape_msgs::SolidPrimitive::SPHERE;
        sphere.dimensions.resize(1);
        sphere.dimensions[0] = 0.075;

        int count = 0;
        int max_count = 100;
        std::vector<float> distances;

        for (const auto &point : cloud.points)
        {
            double dx = point.x - current_pose_.pose.position.x;
            double dy = point.y - current_pose_.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (point.z > 0.15 && distance <= 3.0)
            {
                distances.push_back(distance);
                filtered_points.push_back(point);
                count++;
            }
        }

        std::vector<Point> closest_points;
        if (distances.size() > max_count)
        {
            closest_points = getClosestPoints(filtered_points, distances, max_count);
        }
        else
        {
            closest_points = filtered_points;
        }
        
        int object_id = 0;
        for (const auto &point : closest_points)
        {
            derived_object_msgs::Object obj;
            obj.pose.position.x = point.x;
            obj.pose.position.y = point.y;
            obj.pose.position.z = point.z;
            obj.shape = sphere;
            obj.id = object_id;
            object_array.objects.push_back(obj);
            filtered_cloud.push_back(point);

            object_id ++;
        }

        int num_objects = object_array.objects.size();

        while (num_objects < max_count)
        {
            derived_object_msgs::Object obj;
            obj.pose.position.x = 100;
            obj.pose.position.y = 100;
            obj.pose.position.z = 100;
            obj.shape = sphere;
            obj.id = object_id;
            object_array.objects.push_back(obj);
            num_objects++;
            object_id ++;
        }

        object_pub_.publish(object_array);

        sensor_msgs::PointCloud2 filtered_cloud_msg;
        pcl::toROSMsg(filtered_cloud, filtered_cloud_msg);
        filtered_cloud_msg.header = cloud_msg->header;
        filtered_points_pub_.publish(filtered_cloud_msg);
        ROS_INFO_THROTTLE(1, "Publishing %d obstacles. Valid: %d.", num_objects, count);
    }

    struct Compare {
        bool operator()(const Pair& a, const Pair& b) {
            return a.first < b.first; // Max heap: larger distances come first
        }
    };

    std::vector<Point> getClosestPoints(const std::vector<Point>& points, const std::vector<float>& distances, int num_points=100) {
        std::priority_queue<Pair, std::vector<Pair>, Compare> maxHeap;
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (maxHeap.size() < num_points) {
                maxHeap.emplace(distances[i], points[i]);
            } else if (distances[i] < maxHeap.top().first) {
                maxHeap.pop();
                maxHeap.emplace(distances[i], points[i]);
            }
        }
        // Extract the closest 100 points
        std::vector<Point> closestPoints;
        while (!maxHeap.empty()) {
            closestPoints.push_back(maxHeap.top().second);
            maxHeap.pop();
        }
        
        return closestPoints;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_processor");
    PointCloudProcessor processor;

    ROS_INFO("Obstacle info publisher initialized.");
    
    ros::spin();
    return 0;
}
