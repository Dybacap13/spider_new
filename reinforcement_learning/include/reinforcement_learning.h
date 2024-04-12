#include <ctime>
#include <fstream>
#include <hexapod_msgs/MoveFeetLearning.h>
#include <iostream>
#include <matrix.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

class ReinforcementLearning {

public:
  ReinforcementLearning(ros::NodeHandle nh_);
  void algoritm();

private:
  int NUMBER_NEURON;
  double VOLTAGE_TRESHOLD;
  double DELTA;
  double LEARNING_RATE;
  double LEAK_RATE;
  double TIME_ITERATION;
  int ITERATION_SPIKE_IN;
  std::vector<double> THRESHOLD_GYROSCOPE;
  double THRESHOLD_COORDINATES;

  std::vector<double> VOLTAGE_TRESHOLD_TEST;
  std::vector<double> LEAK_RATE_TEST;
  std::string NAME_FILE_REWARD;
  std::string NAME_FILE_LEGS;
  std::string NAME_FILE_VOLT;

  ros::NodeHandle nh;
  ros::Subscriber reward_sub;
  ros::ServiceClient client_;
  void rewardCallback(std_msgs::Float32 msg);
  void falseInVectorLegs();
  void writerParamReward(double reward_);
  void writerParamLegs(hexapod_msgs::MoveFeet vector_legs_);
  void writerInitParamTimeData(int i);
  void writerParamVolt(Matrix vector_legs_);

  hexapod_msgs::MoveFeet vector_legs;
  double reward_gyroscope = 0.0;
  double reward_odometry = 0.0;
  double reward = 0.0;

  std::string path_to_pack = ros::package::getPath("reinforcement_learning");
};

void ReinforcementLearning::writerInitParamTimeData(int iner) {
  std::ofstream out;
  time_t now = time(0);
  char *dt = ctime(&now);
  std::string file = path_to_pack + NAME_FILE_REWARD;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    out << "____________________________________" << std::endl;
    out << dt << std::endl;
    out << "" << std::endl;
    out << "PARAMETERS" << std::endl;
    out << "NUMBER_NEURON = " << NUMBER_NEURON << std::endl;
    out << "VOLTAGE_TRESHOLD = " << VOLTAGE_TRESHOLD_TEST[iner] << std::endl;
    out << "LEARNING_RATE = " << LEARNING_RATE << std::endl;
    out << "LEAK_RATE_TEST = " << LEAK_RATE_TEST[iner] << std::endl;
    out << "TIME_ITERATION = " << TIME_ITERATION << std::endl;
    out << "ITERATION_SPIKE_IN = " << ITERATION_SPIKE_IN << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();
  file = path_to_pack + NAME_FILE_LEGS;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    out << "____________________________________" << std::endl;
    out << dt << std::endl;
    out << "" << std::endl;
    out << "PARAMETERS" << std::endl;
    out << "NUMBER_NEURON = " << NUMBER_NEURON << std::endl;
    out << "VOLTAGE_TRESHOLD = " << VOLTAGE_TRESHOLD_TEST[iner] << std::endl;
    out << "LEARNING_RATE = " << LEARNING_RATE << std::endl;
    out << "LEAK_RATE_TEST = " << LEAK_RATE_TEST[iner] << std::endl;
    out << "TIME_ITERATION = " << TIME_ITERATION << std::endl;
    out << "ITERATION_SPIKE_IN = " << ITERATION_SPIKE_IN << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();

  file = path_to_pack + NAME_FILE_VOLT;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    out << "____________________________________" << std::endl;
    out << dt << std::endl;
    out << "" << std::endl;
    out << "PARAMETERS" << std::endl;
    out << "NUMBER_NEURON = " << NUMBER_NEURON << std::endl;
    out << "VOLTAGE_TRESHOLD = " << VOLTAGE_TRESHOLD_TEST[iner] << std::endl;
    out << "LEARNING_RATE = " << LEARNING_RATE << std::endl;
    out << "LEAK_RATE_TEST = " << LEAK_RATE_TEST[iner] << std::endl;
    out << "TIME_ITERATION = " << TIME_ITERATION << std::endl;
    out << "ITERATION_SPIKE_IN = " << ITERATION_SPIKE_IN << std::endl;
    out << "THRESHOLD_COORDINATES = " << THRESHOLD_COORDINATES << std::endl;
    out << "THRESHOLD_GYROSCOPE = ";
    for (int i = 0; i < 3; i++) {
      out << THRESHOLD_GYROSCOPE[i] << "  ";
    }
    out << " " << std::endl;
  }
  out.close();
}

void ReinforcementLearning::writerParamReward(double reward_) {
  std::ofstream out;
  std::string file = path_to_pack + NAME_FILE_REWARD;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer reward" << std::endl;
    out << reward_ << std::endl;
  }
  out.close();
}

void ReinforcementLearning::writerParamVolt(Matrix vector_legs_) {
  std::ofstream out;

  std::string file = path_to_pack + NAME_FILE_VOLT;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer volt" << std::endl;
    for (int i = 0; i < 6; i++) {
      out << vector_legs_.get_data()[i] << "    ";
    }

    out << " " << std::endl;
  }
  out.close();
}

void ReinforcementLearning::writerParamLegs(
    hexapod_msgs::MoveFeet vector_legs_) {
  std::ofstream out;

  std::string file = path_to_pack + NAME_FILE_LEGS;
  out.open(file, std::ios::app);
  if (out.is_open()) {
    std::cout << "writer legs" << std::endl;
    for (int i = 0; i < vector_legs_.legs.size(); i++) {
      if (vector_legs_.legs[i] == true)
        out << 1 << "    ";
      else
        out << 0 << "    ";
    }

    out << " " << std::endl;
  }
  out.close();
}

ReinforcementLearning::ReinforcementLearning(ros::NodeHandle nh_) : nh(nh_) {
  reward_sub =
      nh_.subscribe("/reward", 5, &ReinforcementLearning::rewardCallback, this);

  ros::param::get("NUMBER_NEURON", NUMBER_NEURON);
  ros::param::get("VOLTAGE_TRESHOLD", VOLTAGE_TRESHOLD);
  ros::param::get("DELTA", DELTA);
  ros::param::get("LEARNING_RATE", LEARNING_RATE);
  ros::param::get("LEAK_RATE", LEAK_RATE);
  ros::param::get("TIME_ITERATION", TIME_ITERATION);
  ros::param::get("ITERATION_SPIKE_IN", ITERATION_SPIKE_IN);
  ros::param::get("NAME_FILE_REWARD", NAME_FILE_REWARD);
  ros::param::get("NAME_FILE_LEGS", NAME_FILE_LEGS);
  ros::param::get("THRESHOLD_GYROSCOPE", THRESHOLD_GYROSCOPE);
  ros::param::get("THRESHOLD_COORDINATES", THRESHOLD_COORDINATES);

  ros::param::get("VOLTAGE_TRESHOLD_TEST", VOLTAGE_TRESHOLD_TEST);
  ros::param::get("LEAK_RATE_TEST", LEAK_RATE_TEST);
  ros::param::get("NAME_FILE_VOLT", NAME_FILE_VOLT);
  client_ =
      nh_.serviceClient<hexapod_msgs::MoveFeetLearning>("/move_feet_learning");

  falseInVectorLegs();
  // writerInitParamTimeData();
}
void ReinforcementLearning::rewardCallback(std_msgs::Float32 msg) {
  reward = msg.data;
}

void ReinforcementLearning::algoritm() {

    // количество вариантов тестирования ( по порогу)
      for (int iter = 0; iter < 1; iter++) {

        writerInitParamTimeData(iter);

        // количество попыток тестирования с одним порогом
        for (int i = 0; i < 1; i++) {
          writerInitParamTimeData(iter);
          // рандомное распределение весов
          Matrix weight_sgp = Matrix(6, 6);
          Matrix weight_in = Matrix(1, 6);
          Matrix weight_gyro = Matrix(1, 6);
          weight_sgp.fill_random(1);
          weight_in.fill_random(1);
          weight_gyro.fill_random(1);

          // занулить главную диагональ
          weight_sgp.eye_zero();

          //инициализация потенциала
          Matrix voltage = Matrix(1, 6);
          Matrix voltage_last = Matrix(1, 6);
          voltage.zero();
          voltage_last.zero();

          // инициализая спайков

          Matrix spike_sgp = Matrix(6, 6);
          Matrix spike_sgp_last = Matrix(6, 6);
          Matrix spike_in = Matrix(1, 6);
          Matrix spike_gyro = Matrix(1, 6);
          spike_sgp.zero();
          spike_sgp_last.zero();

          spike_in.ones();
          spike_gyro.zero();

          // занулить главную диагональ
          weight_sgp.eye_zero();
          spike_sgp.eye_zero();
          spike_sgp_last.eye_zero();

          // эта часть, если хочется "простимулировать" нейтроны в начале
          // for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
          //   if (neuron == 0 || neuron == 2 || neuron == 4) {
          //     for (unsigned int neighbour = 0; neighbour < NUMBER_NEURON;
          //          neighbour++) {
          //       spike_sgp.get_data()[neuron + NUMBER_NEURON * neighbour] = 1.0;
          //     }
          //   }
          // }


          std::cout << "Random weight CPG" << std::endl;
          weight_sgp.output();
          std::cout << "Random weight in = " << std::endl;
          weight_gyro.output();

          std::cout << "Random weight gyro = " << std::endl;
          weight_in.output();

          std::cout << "Init spike GCP" << std::endl;
          spike_sgp.output();
          spike_sgp_last.output();
          std::cout << "Init spike in = " << std::endl;
          spike_in.output();
          std::cout << "Init spike gyro = " << std::endl;
          spike_gyro.output();
          std::cout << "Init voltage" << std::endl;
          voltage.output();

          // Алгоритм
          // Сумма вклада всех соседних нейронов
          double sum_contribution_neighbours = 0.0;

          //ток
          double current_neuron;

          for (unsigned int time = 0; time < TIME_ITERATION; time++) {

            // 1) ПРОВЕРЯЕМ НА ВСПЫШКИ НЕЙРОНЫ
            for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {
              for (unsigned int neighbour = 0; neighbour < NUMBER_NEURON;
                   neighbour++) {
                // считает вклад в ток от соседних нейронов

                sum_contribution_neighbours =
                    weight_sgp.get_data()[neighbour + NUMBER_NEURON * neuron] *
                        spike_sgp_last
                            .get_data()[neighbour + NUMBER_NEURON * neuron] +
                    sum_contribution_neighbours;
              }

              // считаем ток
              current_neuron =
                  weight_in.get_data()[neuron] * spike_in.get_data()[neuron] +
                  weight_gyro.get_data()[neuron] * spike_gyro.get_data()[neuron] +
                  sum_contribution_neighbours;

              std::cout << " current_neuron =  " << current_neuron << std::endl;

              // считаем потенциал
              voltage.get_data()[neuron] =
                  voltage_last.get_data()[neuron] / LEAK_RATE_TEST[iter] + current_neuron;
              std::cout << " voltage.get_data()[neuron] =  "
                        << voltage.get_data()[neuron] << std::endl;
              std::cout << " voltage_LAST.get_data()[neuron] =  "
                        << voltage_last.get_data()[neuron] << std::endl;

              // если прошли порог
              if (voltage.get_data()[neuron] > VOLTAGE_TRESHOLD_TEST[iter]) {

                std::cout << "------------SPIKE NEURON = " << neuron << std::endl;
                for (unsigned int neighbour = 0; neighbour < NUMBER_NEURON;
                     neighbour++) {
                  spike_sgp.get_data()[neuron + NUMBER_NEURON * neighbour] = 1.0;
                }
                voltage.get_data()[neuron] = 0.0;
                vector_legs.legs[neuron] = true;
              }

              sum_contribution_neighbours = 0.0;
            }

            //занулить главную диагональ
            spike_sgp.eye_zero();

            // 2) ОБНОВЛЯЕМ ВЕСА

            // запрос
            hexapod_msgs::MoveFeetLearning srv;
            srv.request.legs = vector_legs;
            srv.request.current_time = time;

            if (!client_.call(srv)) {
              std::cout << "Failed to call service /move_feet_learning"
                        << std::endl;
                if (!client_.call(srv)) {
                std::cout << "Failed to call service /move_feet_learning"
                          << std::endl;
                    return;
              }
            }

            // получаем награду от сервиса
            reward = srv.response.reward_general;
            reward_odometry = srv.response.reward_odometry;
            reward_gyroscope = srv.response.reward_general;
            std::string resust_str = srv.response.result;

            // обновляем веса
            for (unsigned int neuron = 0; neuron < NUMBER_NEURON * NUMBER_NEURON;
                 neuron++) {
              if (spike_sgp.get_data()[neuron] == 1.0) {

                weight_sgp.get_data()[neuron] =
                    weight_sgp.get_data()[neuron] +
                    LEARNING_RATE * (rand()%2) * reward;
              }
            }
            // занулить главную диагональ
            weight_sgp.eye_zero();
            spike_gyro.zero();

            for (unsigned int neuron = 0; neuron < NUMBER_NEURON; neuron++) {

              if (spike_sgp.get_data()[neuron] == 1.0) {
                weight_in.get_data()[neuron] =
                    weight_in.get_data()[neuron] +
                    LEARNING_RATE * (double)(rand()) / RAND_MAX * reward;

                if (resust_str == "balance lost") {
                  spike_gyro.get_data()[neuron] = 1.0;
                  weight_gyro.get_data()[neuron] =
                      weight_gyro.get_data()[neuron] +
                      LEARNING_RATE * (double)(rand()) / RAND_MAX * reward;
                }
              }
            }

            if (resust_str == "balance lost") {
              std::cout << "Duration" << std::endl;
              ros::Duration(2.0).sleep();
            }

            //импульс in
            if (time % ITERATION_SPIKE_IN == 0.0 && time != 0) {
              if (spike_in.get_data()[0] == 0.0)
                spike_in.ones();
              else
                spike_in.zero();
            }
            std::cout << "OSTATOK = " << time % ITERATION_SPIKE_IN << std::endl;

            std::cout << " voltage_last in time =  " << time << std::endl;
            voltage_last.output();

            // записываем в память текущее состояние потенциала нейрона
            for (unsigned int i = 0; i < NUMBER_NEURON; i++) {
              voltage_last.get_data()[i] = voltage.get_data()[i];
            }

            // здесь мы сохраняем спайки  прошлой итерации
            for (unsigned int i = 0; i < spike_sgp.get_col() * spike_sgp.get_row();
                 i++) {
              spike_sgp_last.get_data()[i] = spike_sgp.get_data()[i];
            }

            // std::cout <<" weight CPG" <<std::endl;
            // weight_sgp.output();
            // std::cout <<" weight in = "<< std::endl;
            // weight_gyro.output();

            // std::cout <<" weight gyro = " <<std::endl;
            // weight_in.output();

            // std::cout <<" spike GCP" <<std::endl;
            // spike_sgp.output();
            // spike_sgp_last.output();
            // std::cout <<" spike in = " <<std::endl;
            // spike_in.output();
            // std::cout <<" spike gyro = "<<std::endl;
            // spike_gyro.output();
            // std::cout <<" voltage" <<std::endl;
            // voltage.output();

            std::cout << " Weight in time =  " << time << std::endl;
            weight_sgp.output();

            std::cout << " weight_gyro in time =  " << time << std::endl;
            weight_gyro.output();

            std::cout << " weight_in in time =  " << time << std::endl;
            weight_in.output();

            std::cout << " voltage in time =  " << time << std::endl;
            voltage.output();

            std::cout << " voltage_last in time =  " << time << std::endl;
            voltage_last.output();

            std::cout << " spike_sgp_last in time =  " << time << std::endl;
            spike_sgp_last.output();

            std::cout << " spike_sgp in time =  " << time << std::endl;
            spike_sgp.output();

            std::cout << " spike_gyro in time =  " << time << std::endl;
            spike_gyro.output();

            std::cout << " spike_in in time =  " << time << std::endl;
            spike_in.output();

            std::cout << "reward = " << reward << std::endl;
            std::cout << "resust_str = " << resust_str << std::endl;
            writerParamReward(reward);
            writerParamLegs(vector_legs);
            writerParamVolt(voltage);
            falseInVectorLegs();
            spike_sgp.zero();
          }
        }
      }
}

void ReinforcementLearning::falseInVectorLegs() {
  for (int i = 0; i < NUMBER_NEURON; i++)
    vector_legs.legs[i] = false;
  vector_legs.cmd_vel = 0.5;
}
