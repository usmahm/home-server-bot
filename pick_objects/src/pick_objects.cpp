#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class PickObject
{
  public:
    PickObject() : ac_("move_base", true)
    {
      pub_ = n_.advertise<std_msgs::Bool>("/goal_reached", 10);

      n_.getParam("/pick_objects/init_marker_pose_x", init_marker_pose_x_);
      n_.getParam("/pick_objects/init_marker_pose_y", init_marker_pose_y_);
      n_.getParam("/pick_objects/init_marker_pose_z", init_marker_pose_z_);
      n_.getParam("/pick_objects/init_marker_orient_x", init_marker_orient_x_);
      n_.getParam("/pick_objects/init_marker_orient_y", init_marker_orient_y_);
      n_.getParam("/pick_objects/init_marker_orient_z", init_marker_orient_z_);
      n_.getParam("/pick_objects/init_marker_orient_w", init_marker_orient_w_);

      n_.getParam("/pick_objects/dest_marker_pose_x", dest_marker_pose_x_);
      n_.getParam("/pick_objects/dest_marker_pose_y", dest_marker_pose_y_);
      n_.getParam("/pick_objects/dest_marker_pose_z", dest_marker_pose_z_);
      n_.getParam("/pick_objects/dest_marker_orient_x", dest_marker_orient_x_);
      n_.getParam("/pick_objects/dest_marker_orient_y", dest_marker_orient_y_);
      n_.getParam("/pick_objects/dest_marker_orient_z", dest_marker_orient_z_);
      n_.getParam("/pick_objects/dest_marker_orient_w", dest_marker_orient_w_);

      // Wait 5 sec for move_base action server to come up
      while(!ac_.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }

      move_to_first_goal();

        // Wait an infinite time for the results
      ac_.waitForResult();
      
      // Check if the robot reached its goal_1
      if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)  {
        std_msgs::Bool msg;
        msg.data = false;
        pub_.publish(msg);
        
        ROS_INFO("Hooray, the base moved 1 meter forward");
      }
      else
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      
      ros::Duration(5.0).sleep();

      move_to_second_goal();

      ac_.waitForResult();
  
      // Check if the robot reached its goal_2
      if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        std_msgs::Bool msg;
        msg.data = true;
        pub_.publish(msg);
        ROS_INFO("Hooray, the base moved 3 meter forward");
      }
      else
        ROS_INFO("The base failed to move forward 3 meter for some reason");
      
    }

    void move_to_first_goal()
    {
      move_base_msgs::MoveBaseGoal goal_1;

      // set up the frame parameters
      goal_1.target_pose.header.frame_id = "map";
      goal_1.target_pose.header.stamp = ros::Time::now();
      
      // Define a position and orientation for the robot to reach
      goal_1.target_pose.pose.position.x = init_marker_pose_x_;
      goal_1.target_pose.pose.position.y = init_marker_pose_y_;
      goal_1.target_pose.pose.position.z = init_marker_pose_z_;
      goal_1.target_pose.pose.orientation.x = init_marker_orient_x_;
      goal_1.target_pose.pose.orientation.y = init_marker_orient_y_;
      goal_1.target_pose.pose.orientation.z = init_marker_orient_z_;
      goal_1.target_pose.pose.orientation.w = init_marker_orient_w_;

      // Send the goal_1 position and orientation for the robot to reach
      ROS_INFO("Sending goal_1");
      ac_.sendGoal(goal_1);    
    }

    void move_to_second_goal()
    {
      move_base_msgs::MoveBaseGoal goal_2;

      goal_2.target_pose.header.frame_id = "map";
      goal_2.target_pose.header.stamp = ros::Time::now();
      
      // Define a position and orientation for the robot to reach
      // goal_2.target_pose.pose.position.x = 3.0;
      // goal_2.target_pose.pose.orientation.w = 2.0;
      goal_2.target_pose.pose.position.x = dest_marker_pose_x_;
      goal_2.target_pose.pose.position.y = dest_marker_pose_y_;
      goal_2.target_pose.pose.position.z = dest_marker_pose_z_;
      goal_2.target_pose.pose.orientation.x = dest_marker_orient_x_;
      goal_2.target_pose.pose.orientation.y = dest_marker_orient_y_;
      goal_2.target_pose.pose.orientation.z = dest_marker_orient_z_;
      goal_2.target_pose.pose.orientation.w = dest_marker_orient_w_;

      // Send the goal_2 position and orientation for the robot to reach
      ROS_INFO("Sending goal_2");
      ac_.sendGoal(goal_2);
    }

  private:
    ros::NodeHandle n_;
    ros::Publisher pub_;

    double init_marker_pose_x_, init_marker_pose_y_, init_marker_pose_z_;
    double init_marker_orient_x_, init_marker_orient_y_, init_marker_orient_z_, init_marker_orient_w_;

    double dest_marker_pose_x_, dest_marker_pose_y_, dest_marker_pose_z_;
    double dest_marker_orient_x_, dest_marker_orient_y_, dest_marker_orient_z_, dest_marker_orient_w_;

    MoveBaseClient ac_;
};

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  
  PickObject POObject;
  
  ros::Rate r(1);

  ros::spin();

  return 0;
}