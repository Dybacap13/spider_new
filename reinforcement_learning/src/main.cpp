#include <iostream>
#include <math.h>
#include <reinforcement_learning.h>



int main(int argc, char **argv)
{

    ros::init(argc, argv, "reinforcement_learning");
    ros::NodeHandle nh;
     //ros::AsyncSpinner spinner( 2 ); // Using 2 threads
     //spinner.start();
    ReinforcementLearning reinforcement_learning(nh);
    ros::Duration(1.0).sleep();
    reinforcement_learning.algoritm();
    ros::spin();

   return 0;
}
