#include <ros/ros.h>
#include <ros/master.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>

class AddMarkers 
{
  public:
    AddMarkers()
    {
      ROS_INFO("Running Add Markers Node.");
      marker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      
      n_.getParam("/add_markers/init_marker_pose_x", init_marker_pose_x_);
      n_.getParam("/add_markers/init_marker_pose_y", init_marker_pose_y_);
      n_.getParam("/add_markers/init_marker_pose_z", init_marker_pose_z_);
      n_.getParam("/add_markers/init_marker_orient_x", init_marker_orient_x_);
      n_.getParam("/add_markers/init_marker_orient_y", init_marker_orient_y_);
      n_.getParam("/add_markers/init_marker_orient_z", init_marker_orient_z_);
      n_.getParam("/add_markers/init_marker_orient_w", init_marker_orient_w_);

      n_.getParam("/add_markers/dest_marker_pose_x", dest_marker_pose_x_);
      n_.getParam("/add_markers/dest_marker_pose_y", dest_marker_pose_y_);
      n_.getParam("/add_markers/dest_marker_pose_z", dest_marker_pose_z_);
      n_.getParam("/add_markers/dest_marker_orient_x", dest_marker_orient_x_);
      n_.getParam("/add_markers/dest_marker_orient_y", dest_marker_orient_y_);
      n_.getParam("/add_markers/dest_marker_orient_z", dest_marker_orient_z_);
      n_.getParam("/add_markers/dest_marker_orient_w", dest_marker_orient_w_);


      initialize_marker();

      // Publish the marker
      while (marker_pub_.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }

      marker_pub_.publish(marker_);

      n_.getParam("/add_markers/is_standalone", is_standalone_);

      if (is_standalone_ != "t") {
        // Keep trying to subcribe to goal_reached topic  (i.e wait for pick_objects to be ready)
        while (ros::ok()) {
          if (isPickObjectsRunning()) {
            ROS_INFO("pick_objects node ready");
            sub_ = n_.subscribe("/goal_reached", 10, &AddMarkers::goal_reached_cb, this);

            ROS_INFO("Callback attached");
            break;
          } else {
            ROS_INFO("pick_objects node not ready");
          }

          ros::Duration(1.0).sleep();
        }
      }

      if (is_standalone_ == "t") {
        ros::Duration(5.0).sleep();

        ROS_INFO("Removing marker.");
        remove_marker();

        ros::Duration(5.0).sleep();

        move_to_dest();
      }
    }

    void initialize_marker()
    {
      ROS_INFO("Initializing Marker");
      uint32_t shape = visualization_msgs::Marker::CUBE;

      marker_.header.frame_id = "map";
      marker_.header.stamp = ros::Time::now();

      // Set the namespace and id for this marker_.  This serves to create a unique ID
      // Any marker_ sent with the same namespace and id will overwrite the old one
      marker_.ns = "basic_shapes";
      marker_.id = 0;

      // Set the marker_ type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
      marker_.type = shape;

      // Set the marker_ action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker_.action = visualization_msgs::Marker::ADD;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      marker_.pose.position.x = init_marker_pose_x_;
      marker_.pose.position.y = init_marker_pose_y_;
      marker_.pose.position.z = init_marker_pose_z_;
      marker_.pose.orientation.x = init_marker_orient_x_;
      marker_.pose.orientation.y = init_marker_orient_y_;
      marker_.pose.orientation.z = init_marker_orient_z_;
      marker_.pose.orientation.w = init_marker_orient_w_;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
      marker_.scale.x = 0.5;
      marker_.scale.y = 0.5;
      marker_.scale.z = 0.5;

      // Set the color -- be sure to set alpha to something non-zero!
      marker_.color.r = 0.0f;
      marker_.color.g = 1.0f;
      marker_.color.b = 0.0f;
      marker_.color.a = 1.0;

      marker_.lifetime = ros::Duration();
    }

    void move_to_dest() 
    {
      ROS_INFO("Moving Marker to Dest");

      marker_.pose.position.x = dest_marker_pose_x_;
      marker_.pose.position.y = dest_marker_pose_y_;
      marker_.pose.position.z = dest_marker_pose_z_;
      marker_.pose.orientation.x = dest_marker_orient_x_;
      marker_.pose.orientation.y = dest_marker_orient_y_;
      marker_.pose.orientation.z = dest_marker_orient_z_;
      marker_.pose.orientation.w = dest_marker_orient_w_;
      marker_.action = visualization_msgs::Marker::ADD;

      marker_pub_.publish(marker_);
    }

    void remove_marker() 
    {
      ROS_INFO("Deleting Marker");
      marker_.action = visualization_msgs::Marker::DELETE;
      marker_pub_.publish(marker_);
    }

    void goal_reached_cb(std_msgs::Bool goal_reached)
    {
      ROS_INFO("Reahced GOALL");
      // ROS_INFO(goal_reached.data);

      if (goal_reached.data) {
        move_to_dest();
      } else {
        remove_marker();
      }
    }

    bool isPickObjectsRunning() {
      std::vector<std::string> nodes;
      ros::master::getNodes(nodes);

      for (std::vector<std::string>::iterator node = nodes.begin(); node != nodes.end(); ++node) {
        if (*node == "/pick_objects") {
          return true;
        }
      }

      return false;
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher marker_pub_;
    ros::Subscriber sub_;

    std::string is_standalone_;
    visualization_msgs::Marker marker_;

    double init_marker_pose_x_, init_marker_pose_y_, init_marker_pose_z_;
    double init_marker_orient_x_, init_marker_orient_y_, init_marker_orient_z_, init_marker_orient_w_;
    
    double dest_marker_pose_x_, dest_marker_pose_y_, dest_marker_pose_z_;
    double dest_marker_orient_x_, dest_marker_orient_y_, dest_marker_orient_z_, dest_marker_orient_w_;
};

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);

  AddMarkers AMObject;

  ros::spin();

  return 0;
}