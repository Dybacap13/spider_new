## Вдохновители
<a href=https://github.com/HumaRobotics/phantomx_gazebo/tree/master>PhantomX</a>
<br> </n>
<a href=https://github.com/KevinOchs/hexapod_ros>Golem</a>


## На данный момент сделано
        
1) Расчёт награды

    считает награду на показаниях гироскопа и координаты ( движения робота)
2) Алгоритм обучения

    нейронная сеть из шесть нейронов, каждый нейрон - одна нога робота
        
#### Все результаты испытаний расположены /reinforcement_learning/statistics

## Инструкция 

*Запускаем gazebo*

     roslaunch spider_gazebo srider_gazebo.launch 
     
*Запускаем ноду, которая будет публиковать в контроллеры*

     rosrun hexapod_control_gazebo hexapod_control_gazebo 
     
*Запускаем ноду, которая считает обратную кинематику и генерирует походку* 

     rosrun hexapod_controller hexapod_controller 
    
*Запускаем ноду, которая считает награду*
    
    rosrun reward_learning calculator_reward 

*Запускаем ноду, которая генерирует движение ногами на основе обучения*
    
    rosrun move_feet move_feet 
    
*Публикуем в топики*

     /state(std_msgs/Bool) - true
     /move_legs/mode(std_msgs/Bool) - true
 
*Запускаем ноду, которая начёт алгоритм обучения*

    rosrun reinforcement_learning reinforcement_learning 
    
    


##  Расчёт награды

**_reward_learning_**

Пакет, где это будет происходить

*Гироскоп*

    /spider/gyroscope_data(sensor_msgs/Imu) -  читает данные с гироскопа в газебо
    
*Одометрия* 

    /gazebo/model_states(gazebo_msgs/ModelStates) - читает координаты робота, относительно нулевой точки

*Награда* 

    /reward(std_msgs/Float) - публикуется расчитанная награда
    

## Как двигать ногой в обучении - инструкция

**_move_feet_**

Пакет, где это будет происходить, расположение ног следующее

     RR -> 0
     RM -> 1
     RF -> 2
     LR -> 3
     LM -> 4
     LF -> 5
     
*Включаем режим - публикуем `true` в топик*

    /move_legs/mode
    Тип сообщения: std_msgs/Bool 
    
*Вызываем сервис и отправляем запрос какими ногами двигать, скорость*

    /move_feet_learning


##  Nodes

**_hexapod_controller_**

Главная нода, отвечает за расчёт обратной кинематики, генерирование походки .Пока используются следующие топики:

*Subscribed Topics*

     cmd_vel (geometry_msgs/Twist)  - задание скорости движения гексапода
     state (std_msgs::Bool)  - true -робот встанет
     
*Published Topics*

    joint_states_to_gazebo (sensor_msgs::JointState) - публикует рассчитанное положение суставов.
    
**_hexapod_controller_gazebo_**

Отвечает за контроль в Gazebo

*Subscribed Topics*

     joint_states_to_gazebo (sensor_msgs::JointState) - считывает рассчитанное положение суставов и публикует в Gazebo

*Published Topics*

     /spider/j_c1_lf_position_controller/command
     /spider/j_c1_lm_position_controller/command
     /spider/j_c1_lr_position_controller/command
     /spider/j_c1_rf_position_controller/command
     /spider/j_c1_rm_position_controller/command
     /spider/j_c1_rr_position_controller/command
     /spider/j_thigh_lf_position_controller/command
     /spider/j_thigh_lm_position_controller/command
     /spider/j_thigh_lr_position_controller/command
     /spider/j_thigh_rf_position_controller/command
     /spider/j_thigh_rm_position_controller/command
     /spider/j_thigh_rr_position_controller/command
     /spider/j_tibia_lf_position_controller/command
     /spider/j_tibia_lm_position_controller/command
     /spider/j_tibia_lr_position_controller/command
     /spider/j_tibia_rf_position_controller/command
     /spider/j_tibia_rm_position_controller/command
     /spider/j_tibia_rr_position_controller/command


**_spider_description_**

Хранит параметры гексапода, его URDF-описание


**_spider_gazebo_**

     roslaunch spider_gazebo srider_gazebo.launch 
     



## Суставы 

Относительно головы -->  суффиксы  

     lf   rf   --передняя левая/правая
     lm   rm   --средняя левая/правая 
     lr   rr   --задняя левая/правая 

**_Привод у туловища_**

     j_c1_lf 
     j_c1_rf 
     j_c1_lm 
     j_c1_rm 
     j_c1_lr 
     j_c1_rr 

 **_Привод колена_**
 
     j_thigh_lf
     j_thigh_rf
     j_thigh_lm
     j_thigh_rm
     j_thigh_lr
     j_thigh_rr

**_Привод у стопы_**

     j_tibia_lf_
     j_tibia_rf
     j_tibia_lm
     j_tibia_rm
     j_tibia_lr
     j_tibia_rr


