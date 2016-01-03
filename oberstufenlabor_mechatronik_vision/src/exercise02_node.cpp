#include <oberstufenlabor_mechatronik_vision/exercise02_library.h>

//############## MAIN ##################################################################################################
int main(int argc, char* argv[])
{
    // === ROS INITIALISATION ===
    ros::init(argc, argv, "exercise02_node");

    exercise02_library exercise02;

    exercise02.start();
}
