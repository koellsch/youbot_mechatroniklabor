#ifndef OBERSTUFENLABOR_MECHATRONIK_ROBOT_EXERCISE04_GRIPPER_TEST_H
#define OBERSTUFENLABOR_MECHATRONIK_ROBOT_EXERCISE04_GRIPPER_TEST_H

#include <ros/ros.h>
#include <oberstufenlabor_mechatronik_vision/Circle.h>
#include <oberstufenlabor_mechatronik_vision/SetColor.h>
#include <actionlib/server/simple_action_server.h>
#include "oberstufenlabor_mechatronik_robot/CollectBallsAction.h"
#include <luh_youbot_manipulation_api/manipulation_api.h>
#include <luh_youbot_poses/youbot_poses.h>

class exercise04_gripper_test : public ros::NodeHandle
{
public:
    exercise04_gripper_test();

    bool gripRoutine();

protected:

    struct CargoSlot
    {
        luh_youbot_kinematics::JointPosition above_pose;
        luh_youbot_kinematics::CylindricPosition pose;
        bool is_free;
        std::string ball_color;
    };

    /*************************************************************************************
         (A) Aufg:	Definieren Sie die für Ihre Statemachine benötigten States.

    **************************************************************************************/

    enum Substate
    {
        /// --->

        /// <---
    }substate_;

    ros::Publisher cmd_vel_publisher_;

    manipulation_api::YoubotArm arm_;
    manipulation_api::YoubotBase base_;
    manipulation_api::YoubotGripper gripper_;

    std::vector<CargoSlot> cargo_;
    youbot_poses::PoseMap poses_;

    CargoSlot* current_cargo_slot_;

    // parameters
    double turn_angle_;
    std::string search_pose_1_;
    std::string search_pose_2_;
    std::string grip_pose_;
    double forward_distance_;
    double angle_align_tolerance_;
    double angle_align_velocity_factor_;
    int max_lost_ball_count_;
    double target_point_x_;
    double target_point_y_;
    double position_align_tolerance_;
    double position_align_velocity_factor_;
    double grip_width_;
    std::string pose_file_name_;
    double above_pose_height_diff_;
    double above_pose_radius_diff_;
    double max_turn_angle_;
    double hold_duration_;
    bool arm_action_is_done_;
    double image_ratio_;
    luh_youbot_kinematics::JointVelocity low_velocity_;
    luh_youbot_kinematics::JointVelocity high_velocity_;

    manipulation_api::Pose2D start_pose_;

    manipulation_api::Pose2D getPoseDiff(manipulation_api::Pose2D current_pose, manipulation_api::Pose2D start_pose);
};

#endif // OBERSTUFENLABOR_MECHATRONIK_ROBOT_EXERCISE04_GRIPPER_TEST_H
