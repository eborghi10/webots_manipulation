#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometric_shapes/solid_primitive_dims.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_msgs/PickupAction.h>
#include <moveit_msgs/PickupActionGoal.h>

// TF

#include <tf/transform_datatypes.h>

// ActionLib

#include <actionlib/client/simple_action_client.h>


/**
 * ANOTHER EXAMPLE: https://github.com/CentralLabFacilities/katana_move_group_plugins/blob/kinetic/katana_manipulation/move_group_katana_pick_place_capability/src/katana_pick_place_action_capability.cpp
 *
 * GRASP DATA: https://github.com/AaronMR/Learning_ROS_for_Robotics_Programming_2nd_edition/blob/indigo-devel/chapter10_tutorials/rosbook_arm_pick_and_place/config/rosbook_arm_grasp_data.yaml
 *
 * PYTHON EXAMPLE: https://github.com/AaronMR/Learning_ROS_for_Robotics_Programming_2nd_edition/blob/indigo-devel/chapter10_tutorials/rosbook_arm_pick_and_place/scripts/pick_and_place.py#L164
 */

typedef actionlib::SimpleActionClient<moveit_msgs::PickupAction> PickUpClient;

void pick(moveit::planning_interface::MoveGroupInterface &group)
{
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "base_link";
  p.pose.position.x = 0.24616;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.163348;
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.699065;
  p.pose.orientation.z = 0.0;
  p.pose.orientation.w = 0.715059;
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.z = -1.0;
  g.pre_grasp_approach.direction.header.frame_id = "base_link";
  g.pre_grasp_approach.min_distance = 0.1;
  g.pre_grasp_approach.desired_distance = 0.2;

  g.post_grasp_retreat.direction.header.frame_id = "base_link";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.2;

  g.pre_grasp_posture.joint_names.resize(3);
  g.pre_grasp_posture.joint_names[0] = "finger_1_joint_1";
  g.pre_grasp_posture.joint_names[1] = "finger_2_joint_1";
  g.pre_grasp_posture.joint_names[2] = "finger_middle_joint_1";
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(3);
  g.pre_grasp_posture.points[0].positions[0] = 0.0495;
  g.pre_grasp_posture.points[0].positions[1] = 0.0495;
  g.pre_grasp_posture.points[0].positions[2] = 0.0495;

  g.grasp_posture.joint_names.resize(3);
  g.grasp_posture.joint_names[0] = "finger_1_joint_1";
  g.grasp_posture.joint_names[1] = "finger_2_joint_1";
  g.grasp_posture.joint_names[2] = "finger_middle_joint_1";
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(3);
  g.grasp_posture.points[0].positions[0] = 1.2218;
  g.grasp_posture.points[0].positions[1] = 1.2218;
  g.grasp_posture.points[0].positions[2] = 1.2218;

  grasps.push_back(g);
  group.pick("box1", grasps);
}


int main(int argc, char **argv)
{
  ros::init (argc, argv, "pick_box");
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // We will use the :planning_scene_interface:`PlanningSceneInterface
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;


  ros::NodeHandle nh;
  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

  ros::Duration sleep_time(10.0);

  ros::WallDuration(1.0).sleep();

  moveit::planning_interface::MoveGroupInterface group("manipulator");
  //moveit::planning_interface::MoveGroupInterface eff("end_effector");
  group.setPlanningTime(45.0);

  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  moveit_msgs::AttachedCollisionObject aco;

	/* The id of the object is used to identify it. */
	collision_object.id = "box1";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.1;
	primitive.dimensions[1] = 0.1;
	primitive.dimensions[2] = 0.1;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  0.25;
	box_pose.position.y = 0.0;
	box_pose.position.z =  0.05;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

        //std::vector<std::basic_string<char> > touch_link ;
        //std::basic_string<char> s = "link5" ;
        //touch_link.push_back(s) ;

        //aco.object = collision_object ;
        //aco.touch_links = touch_link ;

	// Now, let's add the collision object into the world
	  ROS_INFO("Add an object into the world");
	  planning_scene_interface.addCollisionObjects(collision_objects);
          //pub_co.publish(co) ;

	  /* Sleep so we have time to see the object in RViz */
	  sleep(2.0);

  // wait a bit for ros things to initialize
  ros::WallDuration(1.0).sleep();

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  //pickup_goal.planning_options.planning_scene_diff.is_diff = True ;
  //pickup_goal.planning_options.planning_scene_diff.robot_state.is_diff = True ;

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  //target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,-1.5707,1.5707) ;
  target_pose1.position.x = 0.191799;
  target_pose1.position.y = 0.0;
  target_pose1.position.z = 0.189095;

  target_pose1.orientation.x = 0.0;
  target_pose1.orientation.y = 0.699065;
  target_pose1.orientation.z = 0.0;
  target_pose1.orientation.w = 0.715059;

  group.setPoseTarget(target_pose1);


  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  group.move() ;

  sleep(5.0) ;

  geometry_msgs::Pose target_pose2;
  //target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0,-1.5707,1.5707) ;
  target_pose2.position.x = 0.24616;
  target_pose2.position.y = 0.0;
  target_pose2.position.z = 0.163348;

  target_pose2.orientation.x = 0.0;
  target_pose2.orientation.y = 0.699065;
  target_pose2.orientation.z = 0.0;
  target_pose2.orientation.w = 0.715059;

  group.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;
  bool success2 = group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS;

  ROS_INFO("Visualizing plan 2 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  group.move() ;

  sleep(5.0) ;

  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//

  PickUpClient pickup_client("pickup",true) ;
  //pickup_client.waitForServer();

  moveit_msgs::PickupGoal pickup_goal ;


  pickup_goal.target_name  = "box1";        // The name of the object to pick up
  pickup_goal.group_name   = "manipulator"; // Name of group for pickup
  pickup_goal.end_effector = "endeffector"; // Name of eef for pickup

  //*****//
  std::vector<moveit_msgs::Grasp> grasps;

  geometry_msgs::PoseStamped p;
  p.header.frame_id = "pose";
  p.pose.position.x = 0.24616;
  p.pose.position.y = 0.0;
  p.pose.position.z = 0.163348;
  p.pose.orientation.x = 0.0;
  p.pose.orientation.y = 0.699065;
  p.pose.orientation.z = 0.0;
  p.pose.orientation.w = 0.715059;
  moveit_msgs::Grasp g;
  g.grasp_pose = p;

  g.pre_grasp_approach.direction.vector.z = -1.0;
  g.pre_grasp_approach.direction.header.frame_id = "base_link";
  g.pre_grasp_approach.min_distance = 0.1;
  g.pre_grasp_approach.desired_distance = 0.2;

  g.post_grasp_retreat.direction.header.frame_id = "base_link";
  g.post_grasp_retreat.direction.vector.z = 1.0;
  g.post_grasp_retreat.min_distance = 0.1;
  g.post_grasp_retreat.desired_distance = 0.2;

  g.pre_grasp_posture.joint_names.resize(1, "wrist_3_joint");
  g.pre_grasp_posture.points.resize(1);
  g.pre_grasp_posture.points[0].positions.resize(1);
  g.pre_grasp_posture.points[0].positions[0] = 0;

  g.grasp_posture.joint_names.resize(1, "wrist_3_joint");
  g.grasp_posture.points.resize(1);
  g.grasp_posture.points[0].positions.resize(1);
  g.grasp_posture.points[0].positions[0] = 0;

  grasps.push_back(g);
  //***********//

  pickup_goal.possible_grasps = grasps ;
  pickup_goal.allow_gripper_support_collision = true ;

  std::vector<std::basic_string<char> > touch_link ;
  std::basic_string<char> s = "wrist_3_link" ;
  touch_link.push_back(s) ;

  pickup_goal.attached_object_touch_links = touch_link ;
  pickup_goal.allowed_planning_time = 30.0 ;
  pickup_goal.planning_options.planning_scene_diff.is_diff = true ;
  pickup_goal.planning_options.planning_scene_diff.robot_state.is_diff = true ;

  pickup_client.sendGoal(pickup_goal);
  pickup_client.waitForResult(ros::Duration(10.0));
  if (pickup_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
       printf("Yay! The dishes are now clean");
  printf("Current State: %s\n", pickup_client.getState().toString().c_str());

  //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^//


  //pick(group) ;


  std::vector<double> RPY, JointValues ;
  RPY = group.getCurrentRPY() ;
  JointValues = group.getCurrentJointValues() ;

  geometry_msgs::PoseStamped eff_pose ;
  eff_pose = group.getCurrentPose() ;

  std::stringstream ss;
  std::cout.precision(5);
  for (std::size_t i = 0; i < RPY.size() ; ++i)
  {
    ss << std::fixed << RPY[i] << "\t ";
  }

  // ROS_INFO_STREAM("RPY are: " << ss.str() ) ;


  for (std::size_t i = 0; i < JointValues.size() ; ++i)
  {
    ss << std::fixed << JointValues[i] << "\t ";
  }

  // ROS_INFO_STREAM("Joint Values are: " << ss.str() ) ;

  ss << std::fixed << eff_pose << "\t" ;
  // ROS_INFO_STREAM("Pose is: " << ss.str() ) ;

  ros::waitForShutdown();
  return 0;
}
