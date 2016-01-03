#include <ros/ros.h>
#include <oberstufenlabor_mechatronik_vision/Circle.h>
#include <oberstufenlabor_mechatronik_vision/SetColor.h>
#include <luh_youbot_manipulation_api/youbot_arm.h>
#include <luh_youbot_manipulation_api/youbot_gripper.h>
#include <std_msgs/Float32.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <fstream>

namespace yapi = manipulation_api;

oberstufenlabor_mechatronik_vision::Circle g_circle;

void circleCallback(const oberstufenlabor_mechatronik_vision::Circle::ConstPtr &circle)
{
    g_circle = *circle;
}

//############## MAIN ##################################################################################################
int main(int argc, char* argv[])
{
    // === ROS INITIALISATION ===
    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle node;
    ros::Publisher gripper_pub = node.advertise<std_msgs::Float32>("arm_1/gripper_command", 1);
    ros::Subscriber circle_sub = node.subscribe("oberstufenlabor_mechatronik_vision/circle", 1, circleCallback);
    ros::ServiceClient color_client = node.serviceClient<oberstufenlabor_mechatronik_vision::SetColor>("oberstufenlabor_mechatronik_vision/set_color");

    // === PARAMETERS ===
    std::string search_pose, grip_pose;
    double max_gripper_width;
    bool all_params_loaded =
            ros::param::get("oberstufenlabor_mechatronik_robot/search_pose_1", search_pose)&&
            ros::param::get("oberstufenlabor_mechatronik_robot/grip_pose", grip_pose)&&
            ros::param::get("module_gripper/max_gripper_width", max_gripper_width);

    if(!all_params_loaded)
        ROS_WARN("Not all parameters could be loaded.");

    g_circle.was_found = false;

    // === INITIALISE YOUBOT ===
    yapi::YoubotArm arm;
    yapi::YoubotGripper gripper;
    arm.init(node);
    gripper.init(node);

    // === START ===
    std::cout << "Moving arm to initial position..." << std::endl;
    arm.moveToPose("ARM_UP");
    arm.waitForCurrentAction(10);
    gripper.open();
    gripper.waitForCurrentAction();

    // === CALIBRATE GRIPPER ===
    int diff = 1; // mm
    std_msgs::Float32 gripper_msg;
    gripper_msg.data= max_gripper_width;

    std::cout << "Hold a ball between the gripper fingers and press [Enter] repeatedly until the ball is fixed, then type 'q' to continue." << std::endl;

    while (ros::ok() && getchar() != 'q')
    {
        gripper_msg.data -= (diff * 0.001);
        gripper_pub.publish(gripper_msg);
        std::cout << "Gripper width set to " << (int)(gripper_msg.data * 1000) << " mm: ";
    }

    // === CALIBRATE TARGET POINT ===
    std::cout << "Caution! The arm will move to grip pose now. Make sure there is enough free room in front of the robot." << std::endl;

    std::cout << "Continue? [y/n]: ";
    std::string ans;
    std::cin >> ans;
    if(ans[0] != 'y')
    {
        ROS_INFO("Aborting...");
        arm.moveToPose("HOME");
        arm.waitForCurrentAction();
        return 0;
    }

    arm.moveToPose(grip_pose);
    arm.waitForCurrentAction();

    gripper.setWidth(gripper_msg.data + 0.005);
    gripper.waitForCurrentAction();

    std::cout << "Place a ball between the gripper fingers, then press [Enter]." << std::endl;
    getchar();
    getchar();

    arm.moveToPose(search_pose);
    arm.waitForCurrentAction();

    oberstufenlabor_mechatronik_vision::SetColor srv;
    srv.request.name = "foreground";
    srv.request.is_group_name = true;
    while(!color_client.call(srv))
    {
        std::cout << "Error: Service call to ball_finder failed. Try again? [y/n]: ";
        std::cin >> ans;
        if(ans[0] != 'y')
        {
            ROS_INFO("Aborting...");
            arm.moveToPose("HOME");
            arm.waitForCurrentAction();
            return 0;
        }
    }

    std::cout << "Waiting to detect ball..." << std::endl;

    ros::Rate r = ros::Rate(10);

    // wait for circle message
    oberstufenlabor_mechatronik_vision::Circle detected_circle;
    detected_circle.was_found = false;
    int n = 0;
    int n_max = 30;
    while(ros::ok() && n<n_max)
    {
        ros::spinOnce();
        if(g_circle.was_found)
        {
            detected_circle.x += g_circle.x;
            detected_circle.y += g_circle.y;
            n++;
        }
        r.sleep();
    }

    if(!ros::ok())
        return 0;

    detected_circle.x /= n;
    detected_circle.y /= n;

    std::cout << "Ball detected at x = " << detected_circle.x << "; y = " << detected_circle.y << std::endl;

    circle_sub.shutdown();
    srv.request.name = "none";
    srv.request.is_group_name = false;
    color_client.call(srv);

    // === SAVE RESULT TO FILE ===
    std::string filename = ros::package::getPath("oberstufenlabor_mechatronik_robot");
    filename.append("/cfg/calibration_params.yaml");

    YAML::Node result;
    result["x"] = detected_circle.x;
    result["y"] = detected_circle.y;
    result["grip_width"] = gripper_msg.data;

    std::ofstream fout(filename.c_str());
    fout << result;
    fout.close();

    std::cout << "Parameters saved to: " << filename << std::endl;

    // == MOVE TO HOME POSE ===
    arm.moveToPose("HOME");
    arm.waitForCurrentAction();

    ROS_INFO("Done.");

    return 0;
}
