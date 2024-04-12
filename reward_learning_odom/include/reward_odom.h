
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <hexapod_msgs/RewardOdom.h>
#include <gazebo_msgs/ModelStates.h>

class RewardOdometry {
public:
  RewardOdometry(ros::NodeHandle nh_);
  std::string calculatorRewardOdometry(int count);

private:
  ros::NodeHandle nh;
  ros::Subscriber odometry_sub;
  ros::Publisher reward_odometry_pub;
  ros::ServiceServer service_;


  void odometryCallback(gazebo_msgs::ModelStates odometry_msg);
  bool init_service(hexapod_msgs::RewardOdom::Request &req,
                      hexapod_msgs::RewardOdom::Response &res);


  gazebo_msgs::ModelStates odometry;
  gazebo_msgs::ModelStates odometry_last;

  std::vector<double> THRESHOLD_GYROSCOPE;
  double THRESHOLD_COORDINATES;
  double TIME_ITERATION;
  double reward_gyroscope = 0.0;
  double reward_odometry = 0.0;
  double reward = 0.0;
  bool odometry_bool = false;
  bool odometry_last_bool = false;
  double current_time = -1.0;

  std::string result_str;
  int time_t = 100;






};

RewardOdometry::RewardOdometry(ros::NodeHandle nh_) : nh(nh_) {
  odometry_sub =
      nh_.subscribe("/gazebo/model_states", 5, &RewardOdometry::odometryCallback, this);

  reward_odometry_pub = nh_.advertise<std_msgs::Float32>("/reward/reward_odometry", 1000);

  service_ = nh_.advertiseService("/calculator_reward_odometry",
                                        &RewardOdometry::init_service, this);

  ros::param::get("THRESHOLD_GYROSCOPE", THRESHOLD_GYROSCOPE);
  ros::param::get("THRESHOLD_COORDINATES", THRESHOLD_COORDINATES);
  ros::param::get("TIME_ITERATION", TIME_ITERATION);


  ROS_INFO ("Reward Odom ready");

}

bool RewardOdometry::init_service(hexapod_msgs::RewardOdom::Request &req,
                  hexapod_msgs::RewardOdom::Response &res){

    current_time = req.current_time;
    int count = req.count_legs;

    std::string result_reward;

    result_reward = calculatorRewardOdometry(count);
    std::cout << "______________________ "<<std::endl;

    res.reward_odometry = reward_odometry;
    res.result = result_reward;
    return true;
}


void RewardOdometry::odometryCallback(gazebo_msgs::ModelStates odometry_msg) {
  odometry = odometry_msg;
  odometry_bool = true;

}

std::string RewardOdometry::calculatorRewardOdometry(int count) {


    if(!odometry_last_bool) {
        odometry_last_bool = true;
        odometry_last = odometry;
        odometry_last.pose[1].position.x = 0.0;
        odometry_last.pose[1].position.y = 0.0;
        std::cout << "wait" << std::endl;
        return "wait";
    }



    std::cout << "current_time = " <<current_time<<std::endl;
    // расчёт одометрии
    double distance =
      sqrt(pow((odometry.pose[1].position.x - odometry_last.pose[1].position.x), 2) +
           pow((odometry.pose[1].position.y - odometry_last.pose[1].position.y), 2));

    std::cout << "odometry.pose[1].position.x = "<< odometry.pose[1].position.x <<std::endl;
    std::cout << "odometry.pose[1].position.y = "<< odometry.pose[1].position.y <<std::endl;
    std::cout << "odometry_last.pose[1].position.x = "<< odometry_last.pose[1].position.x <<std::endl;
    std::cout << "odometry_last.pose[1].position.y = "<< odometry_last.pose[1].position.y <<std::endl;
    std::cout << "distance = "<< distance <<std::endl;


        if (distance < THRESHOLD_COORDINATES || count == 6)
            reward_odometry = -2.0 * (current_time/(TIME_ITERATION / 2));
         else
            reward_odometry =  2.0 * (current_time/(TIME_ITERATION / 2));



    std_msgs::Float32 msg;
    msg.data = reward_odometry;
    reward_odometry_pub.publish(msg);

    std::cout << "reward_odometry = " <<reward_odometry<<std::endl;



  odometry_last = odometry;

  return "success";

}
