#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/point_cloud_conversion.h>
// #include <sensor_msgs/Point_field.hpp>

#include <goal_getter/goal_3d.h>
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

#include <string>
#include <vector>

using namespace std;

ros::Publisher goal_getter_pub;
sensor_msgs::PointCloud2 curr_pointcloud2;
// sensor_msgs::Image dp_;
std::vector<darknet_ros_msgs::BoundingBox> bboxes;
bool pc_received_ = false;
bool yolo = false;

void calculate_goal(sensor_msgs::PointCloud2& cloud_pc2, sensor_msgs::PointCloud& cloud_pc, 
  goal_getter::goal_3d * goal_msg) {
    goal_msg->header.stamp = cloud_pc2.header.stamp;
    goal_msg->header.frame_id = cloud_pc2.header.frame_id;

    int total_bbox = 0;

    for(auto box:bboxes){
      if(box.probability < 0.2){
        continue;
      }
      
      int center_x = (box.xmax + box.xmin) / 2;
      int center_y = (box.ymax + box.ymin) / 2;

      cout << center_x << " " << center_y << endl;
      int pc_index = (center_y * 640) + center_x;  // not right
      cout << pc_index << endl;
      cout << cloud_pc.points.size() << endl;
      geometry_msgs::Point32 center_point = cloud_pc.points[pc_index];

      cout << center_point.x << " " << center_point.y << " " << center_point.z << endl;
      if (std::isnan(center_point.x) || std::isnan(center_point.y) ||std::isnan(center_point.z)) {
        continue;
      }
      
      goal_msg->x = (goal_msg->x *(total_bbox) + center_point.x)/ (total_bbox+1);
      goal_msg->y = (goal_msg->y *(total_bbox) + center_point.y)/ (total_bbox+1);
      goal_msg->z = (goal_msg->z *(total_bbox) + center_point.z)/ (total_bbox+1);

      total_bbox ++;
    }
}

void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& pc_msg)
{
  // if(yolo){
  curr_pointcloud2 = *pc_msg;
  pc_received_ = true;
  // }
}

// void depthCallback(const sensor_msgs::Image::ConstPtr& depth_msg)
// {
//   dp_ = *depth_msg;
// }

void yoloCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& bboxes_msg){
  bboxes= bboxes_msg->bounding_boxes;
  cout << "success on bbox" << endl;
  // geometry_msgs::msg::TransformStamped transform;
  sensor_msgs::PointCloud cloud_pc;
  goal_getter::goal_3d goal_msg;


  // Todo: transform from working frame to local frame
  // try {
  //   transform = tfBuffer_.lookupTransform(working_frame_, point_cloud_.header.frame_id,
  //       point_cloud_.header.stamp, tf2::durationFromSec(2.0));
  // } catch (tf2::TransformException & ex) {
  //   RCLCPP_ERROR(this->get_logger(), "Transform error of sensor data: %s, %s\n",
  //     ex.what(), "quitting callback");
  //   return;
  // }
  // tf2::doTransform<sensor_msgs::msg::PointCloud2>(point_cloud_, local_pointcloud, transform);
  sensor_msgs::convertPointCloud2ToPointCloud(curr_pointcloud2, cloud_pc);
  cout << "success on converting" << endl;
  cout << curr_pointcloud2.width << " " << curr_pointcloud2.height << endl;
  calculate_goal(curr_pointcloud2, cloud_pc, &goal_msg);
  if (!std::isnan(goal_msg.x))
    goal_getter_pub.publish(goal_msg);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "goal_getter");
  ros::NodeHandle n;

  //Add your code here
  goal_getter_pub = n.advertise<goal_getter::goal_3d>("/goal_getter/goal_3d", 1000);
  ros::Subscriber yolo_sub =n.subscribe("/darknet_ros/bounding_boxes", 1000, yoloCallback);
  ros::Subscriber pc_sub =n.subscribe("/camera/depth/color/points", 1000, pcCallback);
  // ros::Subscriber depth_sub =n.subscribe("/camera/depth/image_rect_raw", 1000, depthCallback);

  ros::Rate loop_rate(20);

  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}