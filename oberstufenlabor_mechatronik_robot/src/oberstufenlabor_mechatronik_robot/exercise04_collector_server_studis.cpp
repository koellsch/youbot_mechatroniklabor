/* *****************************************************************
 *
 * Oberstufenlabor Mechatronik I Aufgabe 3 - Roboterregelung
 * Implementierung der Statemachine für den Greifprozess
 *
 * Copyright (c) 2015,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 ******************************************************************/

#include <oberstufenlabor_mechatronik_robot/exercise04_collector_server_studis.h>
#include "ros/package.h"
#include "yaml-cpp/yaml.h"

namespace yapi = manipulation_api;
namespace ykin = luh_youbot_kinematics;

//########## CONSTRUCTOR ###############################################################################################
exercise04_collector_server::exercise04_collector_server(): ros::NodeHandle(),
    collect_balls_server_(NULL)
{
    // === PARAMETERS ===
    turn_angle_ = M_PI / 4;
    search_pose_1_ = "SEARCH_LOW";
    search_pose_2_ = "SEARCH_HIGH";
    grip_pose_ = "GRIP_POSE";
    forward_distance_ = 0.2;
    angle_align_tolerance_ = 0.1; // in relation to image height
    angle_align_velocity_factor_ = 1.0;
    max_lost_ball_count_ = 3;
    target_point_x_ = 0.0;
    target_point_y_ = 0.0;
    position_align_tolerance_ = 0.02;
    position_align_velocity_factor_ = 1.0;
    grip_width_ = 0.027;
    pose_file_name_ = ros::package::getPath("oberstufenlabor_mechatronik_robot");
    pose_file_name_.append("/poses/poses.yaml");
    above_pose_height_diff_ = 0.03;
    above_pose_radius_diff_ = 0.01;
    max_turn_angle_ = M_PI / 2;
    hold_duration_ = 1.0;
    double interpolation_velocity_factor = 0.3;
    bool all_params_loaded =
            ros::param::get("oberstufenlabor_mechatronik_robot/turn_angle", turn_angle_) &&
            ros::param::get("oberstufenlabor_mechatronik_robot/search_pose_1", search_pose_1_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/search_pose_2", search_pose_2_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/grip_pose", grip_pose_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/forward_distance", forward_distance_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/angle_align_tolerance", angle_align_tolerance_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/angle_align_velocity_factor", angle_align_velocity_factor_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/max_lost_ball_count", max_lost_ball_count_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/position_align_tolerance", position_align_tolerance_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/position_align_velocity_factor", position_align_velocity_factor_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/poses_file", pose_file_name_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/above_pose_height_diff", above_pose_height_diff_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/above_pose_radius_diff", above_pose_radius_diff_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/max_turn_angle", max_turn_angle_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/hold_duration", hold_duration_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/image_ratio", image_ratio_)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/interpolation_velocity_factor", interpolation_velocity_factor);


    if(!all_params_loaded)
        ROS_WARN("Not all parameters could be loaded.");

    std::string target_point_file = ros::package::getPath("oberstufenlabor_mechatronik_robot");
    target_point_file.append("/cfg/calibration_params.yaml");
    YAML::Node doc = YAML::LoadFile(target_point_file);
    if(doc.IsNull())
    {
        ROS_ERROR("target_point.yaml could not be opened.");
    }
    else
    {
        target_point_x_ = doc["x"].as<double>();
        target_point_y_ = doc["y"].as<double>();
        grip_width_ = doc["grip_width"].as<double>();
    }

    // === LOAD POSES ===
    poses_ = youbot_poses::read(pose_file_name_);
    cargo_.resize(9);
    for(uint i=0; i<cargo_.size(); i++)
    {
        std::stringstream ss;
        ss << "CARGO_" << i+1;
        ykin::JointPosition cargo_jnt_pos = poses_[ss.str()];
        ykin::CylindricPosition cargo_cyl_pos = cargo_jnt_pos.toCylindric();
        ykin::CylindricPosition rel_pos;
        rel_pos.setZ(above_pose_height_diff_);
        rel_pos.setR(above_pose_radius_diff_);
        ykin::CylindricPosition above_cyl_pos = cargo_cyl_pos + rel_pos;
        ykin::JointPosition above_jnt_pos = above_cyl_pos.toJointspace(cargo_jnt_pos);

        cargo_[i].is_free = true;
        cargo_[i].pose = cargo_cyl_pos;
        cargo_[i].above_pose = above_jnt_pos;
        cargo_[i].ball_color = "";
    }

    // === ROS COMMUNICATION ===
    circle_subscriber_ = subscribe("oberstufenlabor_mechatronik_vision/circle", 1, &exercise03_collector_server::circleCallback, this);
    color_client_ = serviceClient<oberstufenlabor_mechatronik_vision::SetColor>("oberstufenlabor_mechatronik_vision/set_color");
    cmd_vel_publisher_ = advertise<geometry_msgs::Twist>("cmd_vel", 1);
    timer_ = createTimer(ros::Duration(0.1), &exercise03_collector_server::timerCallback, this, false, false);

    // === START ACTION SERVER ===
    collect_balls_server_ = new CollectBallsServer(*this, "oberstufenlabor_mechatronik_robot/collect_balls", false);
    collect_balls_server_->registerGoalCallback(boost::bind(&exercise03_collector_server::collectBallsCallback, this));
    collect_balls_server_->registerPreemptCallback(boost::bind(&exercise03_collector_server::preemptCallback, this));
    collect_balls_server_->start();

    // === INITIALISE YOUBOT ===
    arm_.init(*this);
    base_.init(*this);
    gripper_.init(*this);

    // === OTHER INITIALISATION ===
    lost_ball_count_ = 0;
    turned_angle_ = 0;
    low_velocity_.setQ1(ykin::MAX_JNT_VELOCITIES[0]);
    low_velocity_.setQ2(ykin::MAX_JNT_VELOCITIES[1]);
    low_velocity_.setQ3(ykin::MAX_JNT_VELOCITIES[2]);
    low_velocity_.setQ4(ykin::MAX_JNT_VELOCITIES[3]);
    low_velocity_.setQ5(ykin::MAX_JNT_VELOCITIES[4]);

    high_velocity_ = low_velocity_;

    low_velocity_ *= interpolation_velocity_factor;

    ROS_INFO("Exercise03 Collector Server is initialised.");
}



//########## DESTRUCTOR ################################################################################################
exercise04_collector_server::~exercise04_collector_server()
{
    delete collect_balls_server_;
}

//########## COLLECT BALLS CALLBACK ####################################################################################
void exercise04_collector_server::collectBallsCallback()
{
    // === GET GOAL ===
    boost::shared_ptr<const oberstufenlabor_mechatronik_robot::CollectBallsGoal> goal = collect_balls_server_->acceptNewGoal();

    goal_number_ = goal->number;
    goal_color_ = goal->color;
    current_number_ = 0;
    detected_circle_.was_found = false;
    state_ = START;
    turned_angle_ = 0;

    gripper_.open();
    gripper_.waitForCurrentAction();

    oberstufenlabor_mechatronik_vision::SetColor srv;
    srv.request.name = goal_color_;
    srv.request.is_group_name = false;
    color_client_.call(srv);

    timer_.start();
}

//########## PREEMPT CALLBACK ##########################################################################################
void exercise04_collector_server::preemptCallback()
{
    timer_.stop();

    if(collect_balls_server_->isActive())
        collect_balls_server_->setPreempted();
}

//########## CIRCLE CALLBACK ###########################################################################################
void exercise04_collector_server::circleCallback(const oberstufenlabor_mechatronik_vision::Circle::ConstPtr &circle)
{
    if(circle->was_found)
        lost_ball_count_ = 0;
    else
        lost_ball_count_++;

    if(circle->was_found || lost_ball_count_ > max_lost_ball_count_)
        detected_circle_ = *circle;
}

//########## TIMER CALLBACK ############################################################################################
void exercise04_collector_server::timerCallback(const ros::TimerEvent &evt)
{
    // === STATE MACHINE ===
    switch(state_)
    {
    //==================================================================================================================
    case START: // initial state
    {
        current_cargo_slot_ = getFreeCargoSlot();

        if(goal_number_ == 0)
        {
            state_ = DONE;
        }
        else if(current_cargo_slot_ == NULL)
        {
            ROS_WARN("Cargo is full. Aborting...");
            state_ = DONE;
        }
        else
        {
            ROS_INFO("Starting in first search pose...");
            arm_.moveToPose(search_pose_1_);
            state_ = ARM_TO_SEARCH_1;
        }
        break;
    }
    //==================================================================================================================
    case ARM_TO_SEARCH_1: // move arm to first search pose and search balls
    {
        if(arm_.isBusy())
            break;

        if(!arm_action_is_done_)
        {
            arm_action_is_done_ = true;
            hold_start_time_ = ros::Time::now();
        }

        if(ballWasFound())
        {
            ROS_INFO("Ball was found. Turning towards ball...");
            arm_action_is_done_ = false;
            state_ = BASE_ALIGN_ANGLE;
        }
        else if((ros::Time::now() - hold_start_time_).toSec() > hold_duration_)
        {
            ROS_INFO("Nothing found. Moving to second search pose...");
            arm_.moveToPose(search_pose_2_);
            arm_action_is_done_ = false;
            state_ = ARM_TO_SEARCH_2;
        }

        break;
    }
    //==================================================================================================================
    case ARM_TO_SEARCH_2: // move arm to second search pose and search balls
    {
        if(arm_.isBusy())
            break;


        if(!arm_action_is_done_)
        {
            arm_action_is_done_ = true;
            hold_start_time_ = ros::Time::now();
        }

        if(ballWasFound())
        {
            ROS_INFO("Ball was found. Turning towards ball...");
            arm_action_is_done_ = false;
            state_ = BASE_ALIGN_ANGLE;
        }
        else if((ros::Time::now() - hold_start_time_).toSec() > hold_duration_)
        {
            ROS_INFO("Moving arm back to first search pose...");
            arm_.moveToPose(search_pose_1_);
            arm_action_is_done_ = false;
            state_ = ARM_TO_SEARCH_1_TURN;
        }

        break;
    }
    //==================================================================================================================
    case ARM_TO_SEARCH_1_TURN: // move arm to first search pose before turn
    {
        if(!arm_.isBusy())
        {
            ROS_INFO("Turning base...");
            base_.move(0, 0, turn_angle_);
            state_ = BASE_TURN;
        }

        break;
    }
    //==================================================================================================================
    case BASE_TURN: // turn the base and search balls
    {
        if(turned_angle_ > max_turn_angle_)
        {
            ROS_INFO("Nothing found. Giving up...");
            state_ = DONE;
        }
        else if(ballWasFound())
        {
            ROS_INFO("Ball was found. Stopping...");
            base_.abortCurrentAction();
            turned_angle_ += turn_angle_; // TODO: calculate exact angle
            state_ = ARM_TO_SEARCH_1;
        }
        else if(!base_.isBusy())
        {
            ROS_INFO("Still not found. Continuing search...");
            turned_angle_ += turn_angle_;
            state_ = ARM_TO_SEARCH_1;
        }
        break;
    }
    //==================================================================================================================
    case BASE_ALIGN_ANGLE: // align the base to face toward the ball (from first search pose)
    {
        geometry_msgs::Twist vel;
        if(fabs(detected_circle_.x) < angle_align_tolerance_)
        {
            vel.angular.z = 0;
            ROS_INFO("Facing towards ball. Now aligning position...");
            start_pose_ = base_.getCurrentPose();
            arm_.setMaxVelocity(low_velocity_);
            arm_.moveToPose(search_pose_1_, yapi::MotionMode::INTERPOLATE);
            state_ = BASE_ALIGN_POSITION;
        }
        else if(detected_circle_.was_found)
        {
            vel.angular.z = -angle_align_velocity_factor_ * detected_circle_.x;
        }
        else if(lost_ball_count_ > max_lost_ball_count_)
        {
            ROS_INFO("Lost ball. Continuing search...");
            state_ = ARM_TO_SEARCH_1;
            break;
        }
        cmd_vel_publisher_.publish(vel);
        break;
    }
    //==================================================================================================================
    case BASE_TO_START: // move base back to start position (in case the tracked ball was lost)
    {
        if(!base_.isBusy())
        {
            ROS_INFO("Back at start. Continuing search...");            
            state_ = ARM_TO_SEARCH_1;
        }

        break;
    }
    //==================================================================================================================
    case BASE_ALIGN_POSITION: // align the position of the base towards the ball
    {
        geometry_msgs::Twist vel;
        double diff_x = target_point_x_ - detected_circle_.x;
        double diff_y = target_point_y_ - detected_circle_.y;

        if(!arm_.isBusy() && fabs(diff_x) < position_align_tolerance_ && fabs(diff_y) < position_align_tolerance_)
        {            
            vel.linear.x = 0;
            vel.linear.y = 0;
            arm_.setMaxVelocity(high_velocity_);
            ROS_INFO("Position aligned. Starting grip routine...");
            substate_ = GRIP_START;
            state_ = GRIP_ROUTINE;
        }
        else if(detected_circle_.was_found)
        {
            vel.linear.x = position_align_velocity_factor_ * diff_y;
            vel.linear.y = position_align_velocity_factor_ * diff_x;
        }
        else if(lost_ball_count_ > max_lost_ball_count_)
        {
            ROS_INFO("Lost ball. Moving back to start...");
            yapi::Pose2D current_pose = base_.getCurrentPose();
            base_.move(getPoseDiff(current_pose, start_pose_));

            arm_.moveToPose(search_pose_1_);
            arm_.setMaxVelocity(high_velocity_);
            state_ = BASE_TO_START;
            break;
        }
        cmd_vel_publisher_.publish(vel);
        break;
    }
    //==================================================================================================================
    case GRIP_ROUTINE: // sub-statemachine for grip movement
    {
        if(!gripRoutine())
            break;

        publishFeedback();

        current_cargo_slot_->is_free = false;
        current_cargo_slot_->ball_color = goal_color_;
        current_number_++;
        current_cargo_slot_ = getFreeCargoSlot();

        if(current_cargo_slot_ == NULL)
        {
            ROS_WARN("Cargo is full. Aborting...");
            state_ = DONE;
        }
        else if(current_number_ >= goal_number_)
        {
            ROS_INFO("Finished.");
            state_ = DONE;
        }
        else
        {
            ROS_INFO("%d balls left. Continuing search...", goal_number_ - current_number_);

            yapi::Pose2D current_pose = base_.getCurrentPose();
            base_.move(getPoseDiff(current_pose, start_pose_));

            arm_.moveToPose(search_pose_1_);
            state_ = BASE_TO_START;
        }
        break;
    }
    //==================================================================================================================
    case DONE: // done :)
    {
        endAction();
        break;
    }
    }
}

//########## GRIP ROUTINE ##############################################################################################
bool exercise04_collector_server::gripRoutine()
{
    /*************************************************************************************
         (A) Aufg:	Implementieren Sie die von Ihnen erstellte Statemachine für den
                    Greifprozess mittels Copy & Paste.

    **************************************************************************************/

    switch(substate_)
    {
    ///<---

    ///--->
    }
    return false;
}
//############## GET FREE CARGO SLOT ###################################################################################
exercise04_collector_server::CargoSlot* exercise04_collector_server::getFreeCargoSlot()
{
    for(uint i=cargo_.size()-1; i>=0; i--)
    {
        if(cargo_[i].is_free)
            return &cargo_[i];
    }
    return NULL;
}

//############## BALL WAS FOUND ########################################################################################
bool exercise04_collector_server::ballWasFound()
{
    if(!detected_circle_.was_found)
        return false;

    double height = 0.5;
    double width = image_ratio_ * height;

    double radius = (detected_circle_.x * detected_circle_.x)/(width * width) +
            (detected_circle_.y * detected_circle_.y) / (height * height);

    return radius < 1.0;
}

//############## END ACTION ############################################################################################
void exercise04_collector_server::endAction()
{
    timer_.stop();
    oberstufenlabor_mechatronik_robot::CollectBallsResult result;
    result.number = current_number_;

    oberstufenlabor_mechatronik_vision::SetColor srv;
    srv.request.name = "none";
    color_client_.call(srv);

    arm_.moveToPose("TRANSPORT");
    yapi::Pose2D current_pose = base_.getCurrentPose();
    base_.move(getPoseDiff(current_pose, start_pose_));
    arm_.waitForCurrentAction();
    base_.waitForCurrentAction();

    if(current_number_ == goal_number_)
        collect_balls_server_->setSucceeded(result);
    else
        collect_balls_server_->setAborted(result);
}

//############## PUBLISH FEEDBACK ######################################################################################
void exercise04_collector_server::publishFeedback()
{
    oberstufenlabor_mechatronik_robot::CollectBallsFeedback feedback;
    feedback.number = current_number_;
    collect_balls_server_->publishFeedback(feedback);
}

//############## GET POSE DIFF #########################################################################################
manipulation_api::Pose2D exercise04_collector_server::getPoseDiff(manipulation_api::Pose2D current_pose,
                                                      manipulation_api::Pose2D start_pose)
{
    double dx = start_pose.x - current_pose.x;
    double dy = start_pose.y - current_pose.y;
    double dt = start_pose.theta - current_pose.theta;

    yapi::Pose2D diff;
    diff.x = dx * cos(current_pose.theta) + dy * sin(current_pose.theta);
    diff.y = dy * cos(current_pose.theta) - dx * sin(current_pose.theta);

    while(dt > M_PI)
        dt -= 2*M_PI;
    while(dt < -M_PI)
        dt += 2*M_PI;

    diff.theta = dt;

    return diff;
}

//############## MAIN ##################################################################################################
int main(int argc, char* argv[])
{
    // === ROS INITIALISATION ===
    ros::init(argc, argv, "exercise04_collector_server");

    exercise04_collector_server server;

    ros::spin();
}
