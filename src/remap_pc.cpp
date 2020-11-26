#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class TransformPointCloud
{
private:
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  void callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
public:
  TransformPointCloud(ros::NodeHandle &node);
};

TransformPointCloud::TransformPointCloud(ros::NodeHandle &node) : nh_(node), tfListener_(this->tfBuffer_)
{
  pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/depth_registered/points_tf", 1);
  sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/depth_registered/points", 1, &TransformPointCloud::callback, this);
}

void TransformPointCloud::callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    sensor_msgs::PointCloud2::Ptr msg_tf (new sensor_msgs::PointCloud2);

    geometry_msgs::TransformStamped transformStamped;
      try{
        transformStamped = tfBuffer_.lookupTransform("base_link", "camera_rgb_optical_frame",
                                ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      }

      tf2::doTransform(*msg, *msg_tf, transformStamped);

      pub_.publish(msg_tf);
  }

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;

  TransformPointCloud tf_pc(node);

  ros::spin();
  return 0;
};
