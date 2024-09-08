#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

double dest_marker_pose_x, dest_marker_pose_y, dest_marker_pose_z;
double dest_marker_orient_x, dest_marker_orient_y, dest_marker_orient_z, dest_marker_orient_w;

void done_cb(std_msgs::Bool goal_reached) {
  ROS_INFO("Reahced GOALL");
  // ROS_INFO(goal_reached.data);

  if (goal_reached.data) {
    marker.pose.position.x = dest_marker_pose_x;
    marker.pose.position.y = dest_marker_pose_y;
    marker.pose.position.z = dest_marker_pose_z;
    marker.pose.orientation.x = dest_marker_orient_x;
    marker.pose.orientation.y = dest_marker_orient_y;
    marker.pose.orientation.z = dest_marker_orient_z;
    marker.pose.orientation.w = dest_marker_orient_w;
    marker.action = visualization_msgs::Marker::ADD;
  } else {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  
  marker_pub.publish(marker);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("/goal_reached", 10, done_cb);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  double init_marker_pose_x, init_marker_pose_y, init_marker_pose_z;
  double init_marker_orient_x, init_marker_orient_y, init_marker_orient_z, init_marker_orient_w;
  n.getParam("/init_marker_pose_x", init_marker_pose_x);
  n.getParam("/init_marker_pose_y", init_marker_pose_y);
  n.getParam("/init_marker_pose_z", init_marker_pose_z);
  n.getParam("/init_marker_orient_x", init_marker_orient_x);
  n.getParam("/init_marker_orient_y", init_marker_orient_y);
  n.getParam("/init_marker_orient_z", init_marker_orient_z);
  n.getParam("/init_marker_orient_w", init_marker_orient_w);

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = init_marker_pose_x;
  marker.pose.position.y = init_marker_pose_y;
  marker.pose.position.z = init_marker_pose_z;
  marker.pose.orientation.x = init_marker_orient_x;
  marker.pose.orientation.y = init_marker_orient_y;
  marker.pose.orientation.z = init_marker_orient_z;
  marker.pose.orientation.w = init_marker_orient_w;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  marker_pub.publish(marker);

  ros::spin();
  // ros::Duration(5.0).sleep();
  return 0;
}