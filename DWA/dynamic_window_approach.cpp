//2019-04-23 shanghai

#include <iostream>
#include <vector>
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define PI 3.141592653

using namespace std;

//the data structures
using Trajectory = vector< array<float, 5> >;
using Obstacle = vector< array<float, 2> >;
using State = array<float, 5>;
using Window = array<float, 4>;
using Point = array<float, 2>;
using Control = array<float, 2>;

// pre-defined kinematic parameters
struct Kinematic_parameters{
  public:
    float max_speed = 1.0;
    float min_speed = -0.5;
    float max_yaw_rate = 40.0 * PI / 180.0;
    float max_accelaration = 0.2;
    float robot_radius = 1.0;
    float max_differential_yaw_rate = 40.0 * PI / 180.0;

    float v_resolution = 0.01;
    float yaw_rate_resolution = 0.1 * PI / 180.0;

    float dt = 0.1;
    float predict_time = 3.0;
    float to_goal_cost_gain = 1.0;
    float speed_cost_gain = 1.0;
};

// the Motion_model model
State Motion_model(State x, Control u, float dt){
    x[2] += u[1] * dt;
    x[0] += u[0] * std::cos(x[2]) * dt;
    x[1] += u[0] * std::sin(x[2]) * dt;
    x[3] = u[0];
    x[4] = u[1];
    return x;
};

// calculate the window
Window Calculate_dynamic_window(State x, Kinematic_parameters kinematic_paramaters){
    Window window_calculation= {
                            std::max((x[3] - kinematic_paramaters.max_accelaration * kinematic_paramaters.dt), kinematic_paramaters.min_speed),
                            std::min((x[3] + kinematic_paramaters.max_accelaration * kinematic_paramaters.dt), kinematic_paramaters.max_speed),
                            std::max((x[4] - kinematic_paramaters.max_differential_yaw_rate * kinematic_paramaters.dt), -kinematic_paramaters.max_yaw_rate),
                            std::min((x[4] + kinematic_paramaters.max_differential_yaw_rate * kinematic_paramaters.dt), kinematic_paramaters.max_yaw_rate)
                        };  
  return window_calculation;
};

//calculate_trajectoryectory
Trajectory calculate_trajectoryectory( State x, float v, float y, Kinematic_parameters kinematic_paramaters ){
    Trajectory trajectory;
    trajectory.push_back(x);
    float time = 0.0;
    while (time <= kinematic_paramaters.predict_time){
        x = Motion_model(x, std::array<float, 2>{{v, y}}, kinematic_paramaters.dt);
        trajectory.push_back(x);
        time += kinematic_paramaters.dt;
    }
    return trajectory;
};


float calculate_obstacle_cost(Trajectory trajectory, Obstacle ob, Kinematic_parameters kinematic_paramaters){
    // calc obstacle cost inf: collistion, 0:free
    int skip_n = 2;
    float min_radius = std::numeric_limits<float>::max();

    for (unsigned int ii=0; ii<trajectory.size(); ii+=skip_n){
        for (unsigned int i=0; i< ob.size(); i++){
            float ox = ob[i][0];
            float oy = ob[i][1];
            float dx = trajectory[ii][0] - ox;
            float dy = trajectory[ii][1] - oy;

            float r = std::sqrt(dx*dx + dy*dy);
            if (r <= kinematic_paramaters.robot_radius){
                return std::numeric_limits<float>::max();
            }
            if (min_radius >= r){
                min_radius = r;
            }
        }
    }
    return 1.0 / min_radius;
};

float calculate_to_goal_cost(Trajectory trajectory, Point goal, Kinematic_parameters kinematic_paramaters){

    float goal_magnitude = std::sqrt(goal[0]*goal[0] + goal[1]*goal[1]);
    float trajectory_magnitude = std::sqrt(std::pow(trajectory.back()[0], 2) + std::pow(trajectory.back()[1], 2));
    float dot_product = (goal[0] * trajectory.back()[0]) + (goal[1] * trajectory.back()[1]);
    float error = dot_product / (goal_magnitude * trajectory_magnitude);
    float error_angle = std::acos(error);
    float cost = kinematic_paramaters.to_goal_cost_gain * error_angle;

    return cost;
  };

Trajectory calculate_final_input(
  State x, Control& u,
  Window dw, Kinematic_parameters kinematic_paramaters, Point goal,
  std::vector<std::array<float, 2>>ob){

    float min_cost = 10000.0;
    Control min_u = u;
    min_u[0] = 0.0;
    Trajectory best_trajectory;

    // evalucate all trajectoryectory with sampled input in dynamic window
    for (float v=dw[0]; v<=dw[1]; v+=kinematic_paramaters.v_resolution){
        for (float y=dw[2]; y<=dw[3]; y+=kinematic_paramaters.yaw_rate_resolution){

            Trajectory trajectory = calculate_trajectoryectory(x, v, y, kinematic_paramaters);

            float to_goal_cost = calculate_to_goal_cost(trajectory, goal, kinematic_paramaters);
            float speed_cost = kinematic_paramaters.speed_cost_gain * (kinematic_paramaters.max_speed - trajectory.back()[3]);
            float ob_cost = calculate_obstacle_cost(trajectory, ob, kinematic_paramaters);
            float final_cost = to_goal_cost + speed_cost + ob_cost;

            if (min_cost >= final_cost){
                min_cost = final_cost;
                min_u = Control{{v, y}};
                best_trajectory = trajectory;
            }
        }
    }
    u = min_u;
    return best_trajectory;
};


Trajectory dwa_control(State x, Control & u, Kinematic_parameters kinematic_paramaters,Point goal, Obstacle ob){
    // # Dynamic Window control
    Window dw = Calculate_dynamic_window(x, kinematic_paramaters);
    Trajectory trajectory = calculate_final_input(x, u, dw, kinematic_paramaters, goal, ob);
    return u, trajectory;
  }


cv::Point2i cv_offset( float x, float y, int image_width=2000, int image_height=2000 ){
    cv::Point2i output;
    output.x = int(x * 100) + image_width/2;
    output.y = image_height - int(y * 100) - image_height/3;
    return output;
};


int main(){

  State x={0.0, 0.0, PI/8.0, 0.0, 0.0};  // the initial state
  Point goal={10.0,10.0};                 // goal position
  //the obstacle
  Obstacle ob={ {-1, -1},{0, 2},{4.0, 2.0},{5.0, 4.0},{5.0, 5.0},{5.0, 6.0},{5.0, 9.0},{8.0, 9.0},{7.0, 9.0},{12.0, 12.0} };

  Control u={0.0, 0.0} ;
  Kinematic_parameters kinematic_paramaters;
  Trajectory trajectory;
  trajectory.push_back(x);

  bool terminal = false;

  cv::namedWindow("dwa", cv::WINDOW_NORMAL);
  int count = 0;

  for(int i=0; i<1000 && !terminal; i++){

          Trajectory ltrajectory = dwa_control(x, u, kinematic_paramaters, goal, ob);
          x = Motion_model(x, u, kinematic_paramaters.dt);
          trajectory.push_back(x);


    // visualization
    cv::Mat bg(3500,3500, CV_8UC3, cv::Scalar(255,255,255));
    cv::circle(bg, cv_offset(goal[0], goal[1], bg.cols, bg.rows), 30, cv::Scalar(255,0,0), 5);

    for(unsigned int j=0; j<ob.size(); j++){
      cv::circle(bg, cv_offset(ob[j][0], ob[j][1], bg.cols, bg.rows), 20, cv::Scalar(0,0,0), -1);
    }

    for(unsigned int j=0; j<ltrajectory.size(); j++){
      cv::circle(bg, cv_offset(ltrajectory[j][0], ltrajectory[j][1], bg.cols, bg.rows),  7, cv::Scalar(0,255,0), -1);
    }

    cv::circle(bg, cv_offset(x[0], x[1], bg.cols, bg.rows), 30, cv::Scalar(0,0,255), 5);

    cv::arrowedLine( bg,cv_offset(x[0], x[1], bg.cols, bg.rows),cv_offset(x[0] + std::cos(x[2]), x[1] + std::sin(x[2]), bg.cols, bg.rows), cv::Scalar(255,0,255),7);

    if (std::sqrt(std::pow((x[0] - goal[0]), 2) + std::pow((x[1] - goal[1]), 2)) <= kinematic_paramaters.robot_radius){
        terminal = true;
        for(unsigned int j=0; j<trajectory.size(); j++){
          cv::circle(bg, cv_offset(trajectory[j][0], trajectory[j][1], bg.cols, bg.rows),7, cv::Scalar(0,0,255), -1);
        }
    }


    cv::imshow("dwa", bg);
    cv::waitKey(5);


    count++;
  }
}
