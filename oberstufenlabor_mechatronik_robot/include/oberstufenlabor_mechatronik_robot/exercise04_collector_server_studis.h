#ifndef OBERSTUFENLABOR_MECHATRONIK_ROBOT_EXERCISE04_COLLECTOR_SERVER_H
#define OBERSTUFENLABOR_MECHATRONIK_ROBOT_EXERCISE04_COLLECTOR_SERVER_H

#include <ros/ros.h>
#include <oberstufenlabor_mechatronik_vision/Circle.h>
#include <oberstufenlabor_mechatronik_vision/SetColor.h>
#include <actionlib/server/simple_action_server.h>
#include "oberstufenlabor_mechatronik_robot/CollectBallsAction.h"
#include <luh_youbot_manipulation_api/manipulation_api.h>
#include <luh_youbot_poses/youbot_poses.h>

class exercise04_collector_server : public ros::NodeHandle
{
public:
    typedef actionlib::SimpleActionServer<oberstufenlabor_mechatronik_robot::CollectBallsAction> CollectBallsServer;

    exercise04_collector_server();
    ~exercise04_collector_server();

protected:

    struct CargoSlot
    {
        luh_youbot_kinematics::JointPosition above_pose;
        luh_youbot_kinematics::CylindricPosition pose;
        bool is_free;
        std::string ball_color;
    };

    enum State
    {
        START,
        ARM_TO_SEARCH_1,
        ARM_TO_SEARCH_2,
        ARM_TO_SEARCH_1_TURN,
        BASE_TURN,
        BASE_TO_START,
        BASE_ALIGN_ANGLE,
        BASE_ALIGN_POSITION,
        GRIP_ROUTINE,
        DONE
    }state_;

    /*************************************************************************************
         (A) Aufg:	Implementieren Sie die von Ihnen erstellten States für die Statemachine
                    aus der Datei "exercise04_gripper_test.h" an dieser Stelle mittels
                    Copy & Paste.

    **************************************************************************************/

    enum Substate
    {
        GRIP_START
        /// --->

        /// <---
    }substate_;

    ros::Subscriber circle_subscriber_;
    ros::ServiceClient color_client_;
    CollectBallsServer* collect_balls_server_;
    ros::Publisher cmd_vel_publisher_;

    ros::Timer timer_;

    manipulation_api::YoubotArm arm_;
    manipulation_api::YoubotBase base_;
    manipulation_api::YoubotGripper gripper_;

    std::string goal_color_;
    int goal_number_;
    int current_number_;

    oberstufenlabor_mechatronik_vision::Circle detected_circle_;

    int lost_ball_count_;

    std::vector<CargoSlot> cargo_;
    youbot_poses::PoseMap poses_;

    CargoSlot* current_cargo_slot_;

    double turned_angle_;

    ros::Time hold_start_time_;

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

    CargoSlot* getFreeCargoSlot();
    bool gripRoutine();
    bool ballWasFound();
    void publishFeedback();
    void endAction();
    void circleCallback(const oberstufenlabor_mechatronik_vision::Circle::ConstPtr &circle);
    void timerCallback(const ros::TimerEvent &evt);

    void collectBallsCallback();
    void preemptCallback();

    manipulation_api::Pose2D getPoseDiff(manipulation_api::Pose2D current_pose, manipulation_api::Pose2D start_pose);
};

#endif // OBERSTUFENLABOR_MECHATRONIK_ROBOT_EXERCISE04_COLLECTOR_SERVER_H
