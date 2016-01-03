#include <ros/ros.h>
#include "oberstufenlabor_mechatronik_robot/CollectBallsAction.h"
#include <actionlib/client/simple_action_client.h>

//############## MAIN ##################################################################################################
int main(int argc, char* argv[])
{
    std::string color;
    int number;

    if(argc != 3)
    {
        std::cout << "Usage: exercise04_collector_node color number" << std::endl;
        return 0;
    }
    else
    {
        color = argv[1];
        number = atoi(argv[2]);

        if(number == 1)
            std::cout << "Picking " << number << " " << color << " ball..." << std::endl;
        else
            std::cout << "Picking " << number << " " << color << " balls..." << std::endl;
    }


    // === ROS INITIALISATION ===
    ros::init(argc, argv, "exercise04_collector_node");
    ros::NodeHandle node;

    actionlib::SimpleActionClient<oberstufenlabor_mechatronik_robot::CollectBallsAction> ac("oberstufenlabor_mechatronik_robot/collect_balls", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    oberstufenlabor_mechatronik_robot::CollectBallsGoal goal;
    goal.color = color;
    goal.number = number;
    ac.sendGoal(goal);

    //wait for the action to return
    ac.waitForResult();

    std::cout << "Done." << std::endl;
    ros::shutdown();
}
