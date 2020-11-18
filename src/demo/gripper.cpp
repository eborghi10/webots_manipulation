#include<ros/ros.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>
#include<moveit/move_group_interface/move_group_interface.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(3);
  posture.joint_names[0] = "finger_1_joint_1";
  posture.joint_names[1] = "finger_2_joint_1";
  posture.joint_names[2] = "finger_middle_joint_1";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(3);
  posture.points[0].positions[0] = 0.05;
  posture.points[0].positions[1] = 0.05;
  posture.points[0].positions[2] = 0.05;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(3);
  posture.joint_names[0] = "finger_1_joint_1";
  posture.joint_names[1] = "finger_2_joint_1";
  posture.joint_names[2] = "finger_middle_joint_1";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(3);
  posture.points[0].positions[0] = 1.22;
  posture.points[0].positions[1] = 1.22;
  posture.points[0].positions[2] = 1.22;
  posture.points[0].time_from_start = ros::Duration(0.5);
  // END_SUB_TUTORIAL
}

void AddObjectsToScene(
  const moveit::planning_interface::MoveGroupInterface& group,
  const moveit::planning_interface::PlanningSceneInterface& psi)
{
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  moveit_msgs::AttachedCollisionObject aco;

	/* The id of the object is used to identify it. */
	collision_object.id = "object";

	/* Define a box to add to the world. */
	shape_msgs::SolidPrimitive primitive;
	primitive.type = primitive.BOX;
	primitive.dimensions.resize(3);
	primitive.dimensions[0] = 0.07;
	primitive.dimensions[1] = 0.07;
	primitive.dimensions[2] = 0.07;

	/* A pose for the box (specified relative to frame_id) */
	geometry_msgs::Pose box_pose;
	box_pose.orientation.w = 1.0;
	box_pose.position.x =  0.57;
	box_pose.position.y = -0.03;
	box_pose.position.z =  0.25;

	collision_object.primitives.push_back(primitive);
	collision_object.primitive_poses.push_back(box_pose);
	collision_object.operation = collision_object.ADD;

	std::vector<moveit_msgs::CollisionObject> collision_objects;
	collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");
  psi.addCollisionObjects(collision_objects);
}

int main(int argc,char** argv)
{
  ros::init(argc,argv,"ur5e_gripper_demo");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface psi;
  moveit::planning_interface::MoveGroupInterface group("manipulator");
  group.setPlanningTime(45.0);

  const std::string reference_frame="base_link";
  group.setPoseReferenceFrame(reference_frame);

  group.allowReplanning(true);

  AddObjectsToScene(group, psi);

  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8. |br|
  // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
  // of the cube). |br|
  // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
  // extra padding)
  grasps[0].grasp_pose.header.frame_id = "base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = 0.415;
  grasps[0].grasp_pose.pose.position.y = 0;
  grasps[0].grasp_pose.pose.position.z = 0.9-0.5;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  // move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  group.pick("object", grasps);
  return 0;
}
