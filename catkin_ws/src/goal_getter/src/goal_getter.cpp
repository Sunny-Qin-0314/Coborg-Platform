#include <ros/ros.h>
#include <gb_visual_detection_3d_msgs/BoundingBox3d.h>
#include <gb_visual_detection_3d_msgs/BoundingBoxes3d.h>
#include <goal_getter/goal_msg.h>
#include <vector>

#define HZ 5

class GoalGetter
{
public:
  GoalGetter()
  {
    darknet3d_sub_ = nh_.subscribe(
      "/darknet_ros_3d/bounding_boxes", 1, &GoalGetter::goalgetterCallback, this);
    goal_pub_ = nh_.advertise<goal_getter::goal_msg>("/goal", 1);
  }

  void
  step()
  {
    double center_x = 0, center_y =0, center_z = 0;

    for(auto bbx : bboxes_)
    {
      // Calculate certer point in 3D coordinates:

      center_x += (bbx.xmin + bbx.xmax) / 2.0;
      center_y += (bbx.ymin + bbx.ymax) / 2.0;
      center_z += (bbx.zmin + bbx.zmax) / 2.0;

      ROS_INFO("%s: (x,y,z) corresponding to d435 camera_link (%lf, %lf, %lf)\n", bbx.Class.c_str(), (bbx.xmin + bbx.xmax) / 2.0,(bbx.ymin + bbx.ymax) / 2.0,(bbx.zmin + bbx.zmax) / 2.0);

    }
    ROS_INFO("---------------------------");
    goal_getter::goal_msg goal_msg;
    goal_msg.Class = "Avg on " + std::to_string(bboxes_.size()) + " hands";
    goal_msg.x = center_x/bboxes_.size();
    goal_msg.y = center_y/bboxes_.size();
    goal_msg.z = center_z/bboxes_.size();
    goal_msg.normal_x = normal_x;
    goal_msg.normal_y = normal_y;
    goal_msg.normal_z = normal_z;
    goal_pub_.publish(goal_msg);
  }

private:
  void
  goalgetterCallback(const gb_visual_detection_3d_msgs::BoundingBoxes3d::ConstPtr & msg)
  {
    // This callback, only save the 3D bounding boxes in 'bboxes_' variable:
    
    bboxes_ = msg->bounding_boxes;
    normal_x = msg -> normal_x;
    normal_y = msg -> normal_y;
    normal_z = msg -> normal_z;
  }

  ros::NodeHandle nh_;
  ros::Subscriber darknet3d_sub_;
  ros::Publisher goal_pub_;
  std::vector<gb_visual_detection_3d_msgs::BoundingBox3d> bboxes_;
  float normal_x, normal_y, normal_z;

};

int
main(int argc, char ** argv)
{
  ros::init(argc, argv, "goal_getter");

  GoalGetter goal_getter;

  // Configure the loop frequency (Hertzios):

  ros::Rate loop_rate(HZ);

  while (ros::ok())
  {
    ros::spinOnce();
    goal_getter.step();
    loop_rate.sleep();
  }

  return 0;
}
