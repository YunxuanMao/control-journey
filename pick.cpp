#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <boost/scoped_ptr.hpp>
// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void AddCollisionObjects(std::vector<moveit_msgs::CollisionObject> &collision_objects, std::vector<moveit_msgs::AttachedCollisionObject> &attached_objects, ros::NodeHandle node_handle, moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
    // add object
    /*
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        ros::WallDuration sleep_time(0.5);
        sleep_time.sleep();
    }*/
    collision_objects.resize(4);
    attached_objects.resize(1);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    collision_objects[2].header.frame_id = "panda_link0";
    collision_objects[2].id = "box";

    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.1;
    collision_objects[2].primitives[0].dimensions[1] = 0.1;
    collision_objects[2].primitives[0].dimensions[2] = 0.1;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.3;
    collision_objects[2].primitive_poses[0].position.y = 0.3;
    collision_objects[2].primitive_poses[0].position.z = 0.7;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[3].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    attached_objects[0].link_name = "panda_leftfinger";
    attached_objects[0].touch_links = std::vector<std::string>{"panda_hand", "panda_leftfinger", "panda_rightfinger"};
    attached_objects[0].object.header.frame_id = "panda_link0";
    attached_objects[0].object.id = "object";

    /* Define the primitive and its dimensions. */
    attached_objects[0].object.primitives.resize(1);
    attached_objects[0].object.primitives[0].type = attached_objects[0].object.primitives[0].BOX;
    attached_objects[0].object.primitives[0].dimensions.resize(3);
    attached_objects[0].object.primitives[0].dimensions[0] = 0.02;
    attached_objects[0].object.primitives[0].dimensions[1] = 0.02;
    attached_objects[0].object.primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    attached_objects[0].object.primitive_poses.resize(1);
    attached_objects[0].object.primitive_poses[0].position.x = 0.5;
    attached_objects[0].object.primitive_poses[0].position.y = 0;
    attached_objects[0].object.primitive_poses[0].position.z = 0.5;
    // END_SUB_TUTORIAL

    attached_objects[0].object.operation = attached_objects[0].object.ADD;

    collision_objects.push_back(attached_objects[0].object);

    
    planning_scene_interface.applyCollisionObjects(collision_objects);
    //planning_scene_interface.applyAttachedCollisionObjects(attached_objects);
    //sleep_time.sleep();
    /*
    ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 10);
    ros::WallDuration sleep_t(0.5);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1)
    {
        sleep_t.sleep();
    }*/
}

int main(int argc, char **argv)
{
    const std::string node_name = "pick";
    // ros initialization
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // robot model& robot state initialization
    const std::string PLANNING_GROUP = "panda_arm";
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(
        new robot_model_loader::RobotModelLoader("robot_description"));
    robot_model::RobotModelPtr robot_model = robot_model_loader->getModel();
    /* Create a RobotState and JointModelGroup to keep track of the current robot pose and planning group*/
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));
    const robot_state::JointModelGroup *joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    //planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr psm(
        new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

    psm->requestPlanningSceneState("get_planning_scene");

    // Using the :moveit_core:`RobotModel`, we can construct a7
    // :planning_scene:`PlanningScene` that maintains the state of
    // the world (including the robot).
    planning_scene::PlanningScenePtr planning_scene = psm->getPlanningScene();

    psm->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);
    psm->startStateMonitor();
    psm->startSceneMonitor();
    /*
  while (!psm->getStateMonitor()->haveCompleteState() && ros::ok())
  {
    ROS_INFO_STREAM_THROTTLE_NAMED(1, node_name, "Waiting for complete state from topic ");
  }
*/

    // add collision
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    std::vector<moveit_msgs::AttachedCollisionObject> attached_objects;
    AddCollisionObjects(collision_objects, attached_objects, node_handle, planning_scene_interface);
    //planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    // load planner
    boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;

    planner_plugin_name = "ompl_interface/OMPLPlanner";

    if (!node_handle.getParam("planning_plugin", planner_plugin_name))
        ROS_FATAL_STREAM("Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>("moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException &ex)
    {
        ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
            ROS_FATAL_STREAM("Could not initialize planner instance");
        ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
    }
    catch (pluginlib::PluginlibException &ex)
    {
        const std::vector<std::string> &classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (std::size_t i = 0; i < classes.size(); ++i)
            ss << classes[i] << " ";
        ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                             << "Available plugins: " << ss.str());
    }

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.loadRobotStatePub("/display_robot_state");
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers(); // clear all old markers
    visual_tools.trigger();


    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    req.group_name = PLANNING_GROUP;
    req.planner_id = "RRT";

    // target pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    pose.pose.orientation = tf2::toMsg(orientation);
    pose.pose.position.x = 0;
    pose.pose.position.y = 0.5;
    pose.pose.position.z = 0.5;

    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);

    moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

    req.goal_constraints.push_back(pose_goal);

    planning_interface::PlanningContextPtr context =
        planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        ROS_ERROR("Could not compute plan successfully");
        return 0;
    }
    // Visualize the result
    // ^^^^^^^^^^^^^^^^^^^^
    ros::Publisher display_publisher =
        node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
    moveit_msgs::DisplayTrajectory display_trajectory;

    /* Visualize the trajectory */
    moveit_msgs::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher.publish(display_trajectory);

    /* Set the state in the planning scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    visual_tools.publishAxisLabeled(pose.pose, "goal_1");
    visual_tools.trigger();

    //ros::waitForShutdown();
}