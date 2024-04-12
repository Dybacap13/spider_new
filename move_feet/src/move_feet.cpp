#include <move_feet.h>

    // RR -> 0
    // RM -> 1
    // RF -> 2
    // LR -> 3
    // LM -> 4
    // LF -> 5


MoveFeet::MoveFeet(ros::NodeHandle nh_): nh(nh_) {


    ros::param::get( "FEMUR_ANGLE", FEMUR_ANGLE );
    ros::param::get( "FEMUR_AXIS", FEMUR_AXIS );
    ros::param::get( "COXA_AXIS", COXA_AXIS );
    ros::param::get( "INTERPOLATION_COEFFICIENT", INTERPOLATION_COEFFICIENT );
    ros::param::get( "DELTA", DELTA );
    ros::param::get( "NUMBER_OF_LEGS", NUMBER_OF_LEGS );

    //legs_sub = nh_.subscribe("/move_legs/legs", 1,  &MoveFeet::legsCallback, this);
    move_feet_mode_sub = nh_.subscribe("/move_legs/mode",  1, &MoveFeet::moveFeetModeCallback, this);
    //reverse_position_sub = nh_.subscribe("/move_legs/reverse_position",  1, &MoveFeet::reversePositionCallback, this);


    joint_states_sub = nh_.subscribe("/joints_to_gazebo", 1, &MoveFeet::jointStatesCallback, this);
    joint_states_pub = nh_.advertise<sensor_msgs::JointState>("/joints_to_gazebo", 1000);


    client_gyro = nh_.serviceClient<hexapod_msgs::RewardGyro>(
             "/calculator_reward_gyroscope");
    client_odom = nh_.serviceClient<hexapod_msgs::RewardOdom>(
             "/calculator_reward_odometry");




    service_ = nh_.advertiseService("/move_feet_learning",
                                          &MoveFeet::init_service, this);
    ROS_INFO("MoveFeet ready");


}

bool MoveFeet::init_service(hexapod_msgs::MoveFeetLearning::Request &req,
                  hexapod_msgs::MoveFeetLearning::Response &res){

    int count = 0;
    hexapod_msgs::MoveFeet reverce;

    // получаем вектор ног и скорость
    auto move_feet = req;
    cmd_vel = move_feet.legs.cmd_vel;
    std::cout<<"________________"<<std::endl;
    std::cout << "current_time = " <<move_feet.current_time<<std::endl;


    // запрос на получение награды от ГИРОСКОПА № 1 !!!
    hexapod_msgs::RewardGyro srv_gyro_1;
    srv_gyro_1.request.legs = move_feet.legs;
    srv_gyro_1.request.current_time = move_feet.current_time;





    // здесь мы считаем ноги и создаём инвентированную команду
    for (auto number_leg = 0; number_leg < move_feet.legs.legs.size(); number_leg ++){
        last_command[number_leg] = move_feet.legs.legs[number_leg];

        if (last_command[number_leg]) {
           count ++;
           reverce.legs[number_leg] = false;

        } else reverce.legs[number_leg] = true;

        std::cout << last_command[number_leg] << "   ";
    }
    std::cout <<  " "<< std::endl;;


    std::cout <<  "count_true = "<< count << std::endl;;

    std::vector<bool>reverse_command = {!last_command[0], !last_command[1], !last_command[2], !last_command[3], !last_command[4],!last_command[5]};

    //НАЧИНАЕМ



    // if (count == 0){


    //     res.reward_general = 0.0;
    //     res.reward_odometry = 0.0;
    //     res.reward_gyroscope = 0.0;
    //     res.result = "zero";
    //     return true;

    // }


    // 1 Поднимаем ноги
    interpolationOfAngles(current_state, upLegs(last_command));
    ros::Duration(3.0).sleep();

    // отправляем запрос ГИРОСКОПУ НОМЕР 1


    if (!client_gyro.call(srv_gyro_1)){
        std::cout << "Failed to call service /calculator_reward_gyroscope" << std::endl;
        interpolationOfAngles(current_state, downLegs(last_command));
        res.result = "error";
        res.reward_general = 0.0;
        res.reward_odometry = 0.0;
        res.reward_gyroscope = 0.0;
        return true; }




    std::cout << "reward_gyroscope_1 = " <<srv_gyro_1.response.reward_gyroscope<<std::endl;

    //Награда от одометрии 1
    hexapod_msgs::RewardOdom srv_odom1;
    srv_odom1.request.count_legs = count;
    srv_odom1.request.current_time = move_feet.current_time;

    if (!client_odom.call(srv_odom1)){
        std::cout << "Failed to call service /calculator_reward_odometry" << std::endl;
        res.result = "error";
        res.reward_general = 0.0;
        res.reward_odometry = 0.0;
        res.reward_gyroscope = 0.0;
        return true; }


    if (srv_odom1.response.result == "wait") {
        ROS_INFO ("INIT SERVER COMPLETE");
        client_odom.call(srv_odom1);
    }

    // получаем награду одометрии
    reward_odometry = srv_odom1.response.reward_odometry;

     if (count == 0){
             res.reward_general = srv_odom1.response.reward_odometry;
             res.reward_odometry = srv_odom1.response.reward_odometry;
             res.reward_gyroscope = 0.0;
             res.result = "zero";
             return true;
     }


    // Если мы отклонились на первом этапе
    if (srv_gyro_1.response.reward_gyroscope <= 2.0 ){

        //сщхраняем награду ( пусть будет_
        reward_gyroscope = srv_gyro_1.response.reward_gyroscope;
        interpolationOfAngles(current_state, downLegs(last_command));

        // отправляем награду обучению
        res.reward_general = srv_gyro_1.response.reward_gyroscope + srv_odom1.response.reward_odometry;
        res.reward_odometry = srv_odom1.response.reward_odometry;
        res.reward_gyroscope = srv_gyro_1.response.reward_gyroscope;
        res.result = srv_gyro_1.response.result;


        return true;
    }


    std::cout <<"HORM" << std::endl;
    //все норм на ПЕРВОМ этапе мы устояли

    //двигаем лапки
    interpolationOfAngles(current_state, moveLegs(last_command));
    //ros::Duration(0.25).sleep();


    interpolationOfAngles(current_state, downLegs(last_command));
    //ros::Duration(0.3).sleep();

    // теперь поднимаем ОБРАТНЫЕ лапки
    interpolationOfAngles(current_state, upLegs(reverse_command));
    ros::Duration(3.0).sleep();

    //отправляем запрос на получение награды ГИРОСКОПА - 2
    hexapod_msgs::RewardGyro srv_gyro_2;
    srv_gyro_2.request.legs = reverce;
    srv_gyro_2.request.current_time = move_feet.current_time;

    if (!client_gyro.call(srv_gyro_2)){
        std::cout << "Failed to call service /calculator_reward_gyroscope" << std::endl;
        interpolationOfAngles(current_state, downLegs(reverse_command));
        res.result = "error";
        res.reward_general = 0.0;
        res.reward_odometry = 0.0;
        res.reward_gyroscope = 0.0;
        return true; }





    //Награда от одометрии 2
    hexapod_msgs::RewardOdom srv_odom2;
    srv_odom2.request.count_legs = count;
    srv_odom2.request.current_time = move_feet.current_time;

    if (!client_odom.call(srv_odom2)){
        std::cout << "Failed to call service /calculator_reward_odometry" << std::endl;
        res.result = "error";
        res.reward_general = 0.0;
        res.reward_odometry = 0.0;
        res.reward_gyroscope = 0.0;
        return true; }


    if (srv_odom2.response.result == "wait") {
        ROS_INFO ("INIT SERVER COMPLETE");
        client_odom.call(srv_odom2);
    }

    // получаем награду одометрии
    reward_odometry = srv_odom2.response.reward_odometry;



    // Если мы отклонились на ВТОРОМ этапе
    if (srv_gyro_2.response.reward_gyroscope <= 2.0 ){

        //сщхраняем награду ( пусть будет_
        reward_gyroscope = srv_gyro_2.response.reward_gyroscope;

        interpolationOfAngles(current_state, downLegs(reverse_command));
         interpolationOfAngles(current_state, reverseTrueLegs(last_command));

        // отправляем награду обучению
        res.reward_general = srv_gyro_2.response.reward_gyroscope + srv_odom2.response.reward_odometry;
        res.reward_odometry = srv_odom2.response.reward_odometry;;
        res.reward_gyroscope = srv_gyro_2.response.reward_gyroscope;
        res.result = srv_gyro_2.response.result;
        return true;
    }
    std::cout <<"HORM 2" << std::endl;

    //Все норм мы устояли на втором этапе!!!
    //двигаем обратно лапки

    interpolationOfAngles(current_state, reverseTrueLegs(last_command));
    std::cout <<" Duration" << std::endl;
    //ros::Duration(3.0).sleep();

    interpolationOfAngles(current_state, downLegs(reverse_command));


    //ЗДЕСЬ МЫ ПРОСИМ НАГРАДУ У ОДОМЕТРИИ !!
    hexapod_msgs::RewardOdom srv_odom;
    srv_odom.request.count_legs = count;
    srv_odom.request.current_time = move_feet.current_time;

    if (!client_odom.call(srv_odom)){
        std::cout << "Failed to call service /calculator_reward_odometry" << std::endl;
        res.result = "error";
        res.reward_general = 0.0;
        res.reward_odometry = 0.0;
        res.reward_gyroscope = 0.0;
        return true; }


    if (srv_odom.response.result == "wait") {
        ROS_INFO ("INIT SERVER COMPLETE");
        client_odom.call(srv_odom);
    }

    // получаем награду одометрии
    reward_odometry = srv_odom.response.reward_odometry;


    // сохраняем
    // std::cout<<"PACCHET"<<std::endl;
    // std::cout<<"srv_odom.response.reward_odometry"<<srv_odom.response.reward_odometry<<std::endl;
    // std::cout<<"srv_gyro_2.response.reward_gyroscope"<<srv_gyro_2.response.reward_gyroscope<<std::endl;
    //std::cout<<"srv_odom.response.reward_odometry + srv_gyro_2.response.reward_gyroscope"<<srv_odom.response.reward_odometry + srv_gyro_2.response.reward_gyroscope<<std::endl;
    reward = srv_odom.response.reward_odometry + srv_gyro_2.response.reward_gyroscope;
    reward_odometry = srv_odom.response.reward_odometry;
    reward_gyroscope = srv_gyro_2.response.reward_gyroscope;

// //std::cout<<"PACCHET 2 "<<std::endl;
// std::cout << "reward_gyroscope = " <<reward_gyroscope<<std::endl;
// std::cout << "reward_odometry = " <<reward_odometry<<std::endl;
// std::cout << "reward = " <<reward<<std::endl;
    res.reward_general = reward;
    res.reward_odometry = reward_odometry;
    res.reward_gyroscope = reward_gyroscope;

    res.result = srv_gyro_2.response.result;

//std::cout<<"PACCHET 3 "<<std::endl;
    std::cout << "reward_gyroscope = " <<reward_gyroscope<<std::endl;
    std::cout << "reward_odometry = " <<reward_odometry<<std::endl;
    std::cout << "reward = " <<reward<<std::endl;
    return true;
}





void MoveFeet::moveFeetModeCallback(std_msgs::Bool msg){
    if (!current_state_bool) {
        ROS_WARN ("Wait while the current angles are calculated");
        return;
    }
    move_feet_mode = msg.data;
}


void MoveFeet::jointStatesCallback(sensor_msgs::JointState msg){
    current_state = msg;
    current_state_bool = true;


}


void MoveFeet::legsCallback (hexapod_msgs::MoveFeet move_feet){

    // hexapod_msgs::Reward srv;
    // srv.request.request = true;
    // cmd_vel = move_feet.cmd_vel;
    // ROS_INFO("1");

    // for (auto number_leg = 0; number_leg < move_feet.legs.size(); number_leg ++){
    //     last_command[number_leg] = move_feet.legs[number_leg];
    // }

    // std::vector<bool>reverse_command = {!last_command[0], !last_command[1], !last_command[2], !last_command[3], !last_command[4],!last_command[5]};

    // interpolationOfAngles(current_state, upLegs(last_command));
    // ros::Duration(0.25).sleep();

    // if (!client_.call(srv)){
    //     std::cout << "Failed to call service /calculator_reward" << std::endl;
    //     interpolationOfAngles(current_state, downLegs(last_command));
    //     return; }

    // if (srv.response.result == "wait") {
    //     ROS_INFO ("INIT SERVER COMPLETE");
    //     client_.call(srv);
    // }

    // //отклонились

    // if (srv.response.reward_general < reward_gyroscope ){
    //     reward_gyroscope = srv.response.reward_general;
    //     interpolationOfAngles(current_state, downLegs(last_command));
    //     return;
    // }


    // //все норм
    // interpolationOfAngles(current_state, moveLegs(last_command));
    // ros::Duration(0.25).sleep();
    // interpolationOfAngles(current_state, downLegs(last_command));
    // //ros::Duration(0.3).sleep();
    // interpolationOfAngles(current_state, reverseTrueLegsAndUpFalseLegs(last_command));
    // ros::Duration(0.25).sleep();
    // interpolationOfAngles(current_state, downLegs(reverse_command));
    // reward_gyroscope = srv.response.reward_general;


}


void MoveFeet::interpolationOfAngles(sensor_msgs::JointState current, sensor_msgs::JointState target){

    sensor_msgs::JointState interpolation_angles_pub;
    interpolation_angles_pub = current;

    while (ros::ok()) {
        if (comparisonJointStates(target,interpolation_angles_pub )) // цикл прекратится, когда старт. значение = желаемому
            break;

        for (int i = 0; i < current_state.name.size(); i++){
            interpolation_angles_pub.position[i] = target.position[i] * INTERPOLATION_COEFFICIENT + interpolation_angles_pub.position[i] * (1 - INTERPOLATION_COEFFICIENT);
            //std::cout <<interpolation_angles_pub.name [i] <<" = " << interpolation_angles_pub.position [i] <<std::endl;
        }

        jointStatesPublisher(interpolation_angles_pub);

        current_state = interpolation_angles_pub;

        //std::cout <<"_________________"  <<std::endl;
        //std::cout <<""  <<std::endl;
        //std::cout <<""  <<std::endl;
    }

}



bool MoveFeet::comparisonJointStates(sensor_msgs::JointState first, sensor_msgs::JointState second){
    if (first.name.size() != second.name.size()) {
        ROS_ERROR("ERROR IN comparisonJointStates ");
        //ros::Duration(1.0).sleep();
        return false;
    }

    for (int i = 0; i < first.name.size(); i++ ){
        if (abs(first.position[i] - second.position[i]) > DELTA) {
            return false;
        }
    }
    return true;
}



void MoveFeet::jointStatesPublisher(sensor_msgs::JointState msg_pub){
    joint_states_pub.publish(msg_pub);
    ros::Duration(0.1).sleep();
}




void MoveFeet::reversePositionCallback(std_msgs::Bool msg){
    if (!last_state_bool) {
        ROS_ERROR("NO LAST STATE");
        return;
    }
    if(msg.data) reversePosition();
}


void MoveFeet::reversePosition(){
    //ros::Duration(1.0).sleep();
    // sensor_msgs::JointState target_state;
    // sensor_msgs::JointState down_leg;
    // target_state = last_state;
    // down_leg = last_state;

    //  for (auto number_leg = 0; number_leg < 6; number_leg ++) {


    //     if (last_command[number_leg])
    //          target_state.position[number_leg * 3 + 1] = current_state.position[number_leg * 3 + 1]  + (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
    //  }

    // interpolationOfAngles(current_state, target_state);

}



sensor_msgs::JointState MoveFeet::upLegs(std::vector<bool> command){
    sensor_msgs::JointState target_state = current_state;
    std::cout <<"1) upLegs" <<std::endl;

    for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg ++) {


        if (!command[number_leg]) {
            continue;
        }


        // здесь код для ног true
        // поднимаем
        target_state.position[number_leg * 3 + 1] = current_state.position[number_leg * 3 + 1]  + (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
       // std::cout <<target_state.name [number_leg * 3+1] <<" = " << target_state.position [number_leg * 3+1] <<std::endl;

     }
    //ros::Duration(2.0).sleep();

    return target_state;


}

sensor_msgs::JointState MoveFeet::moveLegs(std::vector<bool> command){
    sensor_msgs::JointState target_state = current_state;
    std::cout <<"2) moveLegs" <<std::endl;

    for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg ++) {


        if (!command[number_leg]) {
            continue;
        }



        // здесь двигаем coxa
        target_state.position[number_leg * 3] = current_state.position[number_leg * 3]  + (cmd_vel * COXA_AXIS[number_leg]);

      //  std::cout <<target_state.name [number_leg * 3] <<" = " << target_state.position [number_leg * 3] <<std::endl;
    }

    //ros::Duration(2.0).sleep();

    return target_state;


}

sensor_msgs::JointState MoveFeet::downLegs(std::vector<bool> command){
    sensor_msgs::JointState down_leg = current_state;
    std::cout <<"3) downLegs" <<std::endl;
    for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg ++) {
        if (command[number_leg]){
            down_leg.position[number_leg * 3 + 1] = current_state.position[number_leg * 3 + 1]  - (FEMUR_ANGLE * FEMUR_AXIS[number_leg]);
        }
        //std::cout <<down_leg.name [number_leg * 3 + 1] <<" = " << down_leg.position [number_leg * 3 + 1] <<std::endl;

    }
    //ros::Duration(2.0).sleep();

    return down_leg;

}

sensor_msgs::JointState MoveFeet::reverseTrueLegs(std::vector<bool> command){
    sensor_msgs::JointState target_state = current_state;
    std::cout <<"4) reverseTrueLegs" <<std::endl;
      for (auto number_leg = 0; number_leg < NUMBER_OF_LEGS; number_leg ++) {
           if (command[number_leg]){
               target_state.position[number_leg * 3 ] = current_state.position[number_leg * 3]  - (cmd_vel * COXA_AXIS[number_leg]);
           }

      }

 // ros::Duration(2.0).sleep();
        return target_state;
}


