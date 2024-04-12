#include <ros/ros.h>
#include <hexapod_msgs/MoveFeet.h>





int main(int argc, char **argv) {
    ros::init(argc, argv, "test_tripod_gait");
    ros::NodeHandle nh;
    ros::Publisher legs_pub = nh.advertise<hexapod_msgs::MoveFeet>("/move_legs/legs", 1000);


    hexapod_msgs::MoveFeet left;
    hexapod_msgs::MoveFeet right;
    left.legs = {false, true, false, true, false, true};
    left.cmd_vel = 0.5;

    right.legs = {true, false, true, false, true, false};
    right.cmd_vel = 0.5;


    // left.legs = {true, true, false, false, true, true};
    // left.cmd_vel = 0.5;

    // right.legs = {false, true, true, true, true, false};
    // right.cmd_vel = 0.5;





    while(ros::ok()){

        legs_pub.publish(left);
        ros::Duration(1.0).sleep();
        legs_pub.publish(right);
       ros::Duration(1.0).sleep();

       ros::spinOnce();
   }


    return 0;

}
