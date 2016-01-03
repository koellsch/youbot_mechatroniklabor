/* *****************************************************************
 *
 * Oberstufenlabor Mechatronik I Aufgabe 3 - Roboterregelung
 * Erstellung einer Statemachine für den Greifprozess
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

#include <oberstufenlabor_mechatronik_robot/exercise04_gripper_test.h>

namespace yapi = manipulation_api;
namespace ykin = luh_youbot_kinematics;

//########## CONSTRUCTOR ###############################################################################################
exercise04_gripper_test::exercise04_gripper_test(): ros::NodeHandle()
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
    current_cargo_slot_ = &cargo_[0];

    // === ROS COMMUNICATION ===
    cmd_vel_publisher_ = advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // === INITIALISE YOUBOT ===
    arm_.init(*this);
    base_.init(*this);
    gripper_.init(*this);

    // === OTHER INITIALISATION ===
    low_velocity_.setQ1(ykin::MAX_JNT_VELOCITIES[0]);
    low_velocity_.setQ2(ykin::MAX_JNT_VELOCITIES[1]);
    low_velocity_.setQ3(ykin::MAX_JNT_VELOCITIES[2]);
    low_velocity_.setQ4(ykin::MAX_JNT_VELOCITIES[3]);
    low_velocity_.setQ5(ykin::MAX_JNT_VELOCITIES[4]);

    high_velocity_ = low_velocity_;

    low_velocity_ *= interpolation_velocity_factor;

    arm_.moveToPose("HOME");
    gripper_.open();
    gripper_.waitForCurrentAction();

    substate_ = GRIP_START;

    ROS_INFO("Gripper Test is initialized.");
}

//########## GRIP ROUTINE ##############################################################################################
bool exercise04_gripper_test::gripRoutine()
{
    /*************************************************************************************
         (A) Aufg:	An dieser Stelle soll eine Statemachine für den Greif- und Ablegeprozess
                    eines Balls erstellt werden. Die dafür benötigten States können dafür in der
                    Datei "exercise03_gripper_test.h" definiert werden.

         Hinweise:  - Der Zugriff auf den Arm und den Gripper erfolgt über die Objekte
                      "arm_" und "gripper_". Für den Arm stehen die Posen "GRIP" und "GRIP_ABOVE"
                      zur Verfügung. Auf die Posen für den Cargo Slot kann über das Objekt
                      "current_cargo_slot_" mit "above_pose" und "pose" zugegriffen werden.
                    - Für die Überprüfung, ob sich der Arm / Gripper in Bewegung befindet, enthalten
                      beide Objekte jeweils die Funktion "isBusy()".
                    - Der Gripper kann über die Funktion "setWidth()" geschlossen werden. Die
                      Variable "grip_width_" enthält die benötigte Breite. Die Funktion "open()"
                      öffnet den Gripper.

    **************************************************************************************/
    switch(substate_)
    {
    ///<---

    ///--->
    }
    return false;
}

//############## MAIN ##################################################################################################
int main(int argc, char* argv[])
{
    // === ROS INITIALISATION ===
    ros::init(argc, argv, "exercise03_gripper_test");

    exercise04_gripper_test gripper_test;
    while(!gripper_test.gripRoutine() && ros::ok())
        ros::spinOnce();
    ros::shutdown();
}
