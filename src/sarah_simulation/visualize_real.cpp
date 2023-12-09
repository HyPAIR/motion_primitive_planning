#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <visualization_msgs/InteractiveMarker.h>
#include "liom_local_planner/liom_local_planner.h"
#include "liom_local_planner/math/generate_obs.h"
#include "liom_local_planner/visualization/plot.h"
#include "liom_local_planner/math/math_utils.h"
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include "yaml-cpp/yaml.h"
#include "liom_local_planner/yaml_all.h"
#include "std_msgs/Int32.h"

using json = nlohmann::json;
using namespace liom_local_planner;
namespace ob = ompl::base;
int traj_ind = 0;
int time_step = 0;
struct MotionPrimitive {
    std::string name;
    double turningRadius;
    std::vector<std::string> motionTypes;
    std::vector<double> motionLengths;
};

void writetime_count(const std::vector<int> &time_set, std::string filename) {
    YAML::Emitter emitter;
    emitter << YAML::BeginSeq;
    for (const auto& element : time_set) {
        emitter << element;
    }
    emitter << YAML::EndSeq;
    std::ofstream fout(filename);
    fout << emitter.c_str();
    fout.close();

    std::cout << "YAML file written successfully.\n";
}

void readtime_count(std::vector<int> &time_set, std::string filename) {
  try {
    // 从文件中加载 YAML 文档
    YAML::Node config = YAML::LoadFile(filename);

    // 确保文档是一个序列
    if (config.IsSequence()) {
        // 读取序列中的每个元素并存储到 std::vector<int> 中
        time_set = config.as<std::vector<int>>();
    } else {
        std::cerr << "Error: YAML document is not a sequence.\n";
    }
  } catch (const YAML::Exception& e) {
      std::cerr << "YAML Exception: " << e.what() << "\n";
  }
}

void PlotPose(std::shared_ptr<liom_local_planner::PlannerConfig> config, int robot_index, int time_step, Trajectory_temp traj) {
    const std::string robot_name = "Footprint_" + std::to_string(robot_index);
    // visualization::Delete(i, robot_name);
    // visualization::Trigger();
    double theta = traj[time_step].theta;
    if (theta < -M_PI) {
        while (theta < -M_PI) {
            theta += 2 * M_PI;
        }
    }
    else if (theta > M_PI) {
        while (theta > M_PI) {
            theta -= 2 * M_PI;
        }
    }
    math::Pose robot_pos(traj[time_step].x, traj[time_step].y, theta);
    auto box = config->vehicle.GenerateBox(robot_pos);
    auto color = visualization::Color::Magenta;
    visualization::PlotPolygon(math::Polygon2d(box), 0.1, color, 1, robot_name);
    visualization::Trigger();
    // visualization::Plot(xs, ys, 0.5, visualization::Color::Magenta, i, robot_name_ta);
}

FullStates traj2fullstates(Trajectory_temp traj) {
    FullStates solution;
    solution.states.resize(traj.size());
    solution.tf = traj[traj.size() - 1].t;
    for (int i = 0; i < traj.size(); i++) {
        solution.states[i].x = traj[i].x;
        solution.states[i].y = traj[i].y;
        solution.states[i].theta = traj[i].theta;
        solution.states[i].v = traj[i].v;
        solution.states[i].phi = traj[i].phi;
        solution.states[i].a = traj[i].a;
        solution.states[i].omega = traj[i].omega;
    }
    return solution;
}

void DrawPathRivz(const std::vector<std::vector<double>> path, int robot_index) {
  std::vector<double> xs, ys;
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for (int i = 0; i < path.size(); i++) {
    xs.push_back(path[i][0]);
    ys.push_back(path[i][1]);
  }
  auto color = visualization::Color::White;
  color.set_alpha(0.4);
  visualization::Plot(xs, ys, 0.3, color, robot_index, robot_name);
  visualization::Trigger();
}

void DrawCircle(const double& x, const double& y, const double radius, const int& wp_index, const int obs_ind) {
    std::vector<double> xwp, ywp;
    const std::string robot_name = "Waypoint" + std::to_string(wp_index);
    for (int i = 0; i < 100; i++) {
        double theta = 2 * M_PI / 100 * i;
        xwp.push_back(x + radius * cos(theta));
        ywp.push_back(y + radius * sin(theta));
    }
    auto color = obs_ind == 1 ? visualization::Color::Red : visualization::Color::Blue;
    color.set_alpha(0.7);
    visualization::PlotPoints(xwp, ywp, 0.15, color, wp_index, robot_name);
    visualization::Trigger();
}

void DrawTrajectoryRviz(const FullStates sol_traj,std::shared_ptr<liom_local_planner::PlannerConfig> config,
      int robot_index, ros::Publisher path_pub) {
  const std::string robot_name = "Footprint_" + std::to_string(robot_index);
  for(int i = 0; i < 1e3; i++) {
    visualization::Delete(i, robot_name);
  }
  visualization::Trigger();

  nav_msgs::Path msg;
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();
  for(size_t i = 0; i < sol_traj.states.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = sol_traj.states[i].x;
    pose.pose.position.y = sol_traj.states[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(sol_traj.states[i].theta);
    msg.poses.push_back(pose);

    auto box = config->vehicle.GenerateBox(sol_traj.states[i].pose());
    auto color = robot_index > 0 ? visualization::Color::Yellow : visualization::Color::White;
    color.set_alpha(0.1);
    visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

FullStates generateTrajectory(const MotionPrimitive& primitive, double startX, double startY, double startTheta, int totalPoints) {
    FullStates trajectory;

    // 初始状态
    double currentX = startX;
    double currentY = startY;
    double currentTheta = startTheta;
    currentTheta = 0.0;

    for (size_t i = 0; i < primitive.motionTypes.size(); ++i) {
        std::string motionType = primitive.motionTypes[i];
        double motionLength = primitive.motionLengths[i];

        // 设置每一步产生的轨迹点数
        int pointsPerStep = totalPoints / primitive.motionTypes.size();

        // 根据运动类型进行逐步移动
        for (int point = 0; point < pointsPerStep; ++point) {
            double ratio = static_cast<double>(point) / static_cast<double>(pointsPerStep);
            double endX, endY, endTheta;

            double angle = motionLength * ratio / primitive.turningRadius;

            if (motionType == "L") {
                // 左转
                endX = currentX + primitive.turningRadius * std::sin(angle + currentTheta);
                endY = currentY - primitive.turningRadius * (1 - std::cos(angle + currentTheta));
                endTheta = currentTheta + angle;
            } else if (motionType == "R") {
                // 右转
                endX = currentX - primitive.turningRadius * std::sin(angle - currentTheta);
                endY = currentY - primitive.turningRadius * (1 - std::cos(angle - currentTheta));
                endTheta = currentTheta - angle;
            }

            // 插值计算轨迹点
            double interpolatedX = currentX + ratio * (endX - currentX);
            double interpolatedY = currentY + ratio * (endY - currentY);
            TrajectoryPoint temp_pt;
            temp_pt.x = interpolatedX;
            temp_pt.y = interpolatedY;
            temp_pt.theta = endTheta;
            trajectory.states.push_back(temp_pt);

            // 更新当前状态
            currentX = interpolatedX;
            currentY = interpolatedY;
            // currentTheta = endTheta;
        }
    }

    return trajectory;
}


void integerCallback(const std_msgs::Int32::ConstPtr& msg) {
    traj_ind = msg->data;
    // ROS_INFO("Received integer: %d", msg->data);
}

int main(int argc, char* argv[]) {
    double scale = 3;
    double step_size = 0.1;
    std::ifstream file("/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/data.json");

    if (!file.is_open()) {
        std::cerr << "Failed to open JSON file." << std::endl;
        return 1;
    }

    json jsonData;
    file >> jsonData;

    file.close();

    std::vector<std::vector<double>> complete_path;
    std::vector<math::Pose> complete_path_;
    std::vector<std::vector<std::string>> motionTypes_set;
    std::vector<std::vector<double>> motionLengths_set;
    std::vector<std::string> name_set;
    std::vector<std::vector<math::Pose>> traj_set;
    double obstacleRadius = jsonData["ObstacleRadius"];
    double turningRadius = jsonData["TurningRadius"];
    turningRadius *= scale;

    std::map<int, std::array<double, 2>> waypointsMap;
    for (const auto& waypoint : jsonData["Waypoints"].items()) {
        int waypointKey = std::stoi(waypoint.key());
        waypointsMap[waypointKey] = waypoint.value();
    }

    std::cout << "Sorted Waypoints:\n";
    for (const auto& waypoint : waypointsMap) {
        std::cout << "Waypoint " << waypoint.first << ": (" << waypoint.second[0] << ", " << waypoint.second[1] << ")\n";
    }

    json motionPrimitives = jsonData["MotionPrimitves"];
    for (const auto& primitive : motionPrimitives) {
        std::string name = primitive["name"];
        double turningRadius = primitive["turningRadius"];
        std::vector<std::string> motionTypes = primitive["motionTypes"];
        std::vector<double> motionLengths = primitive["motionLengths"];
        for (int i = 0; i < motionLengths.size(); i++) {
          motionLengths[i] = scale * motionLengths[i];
        }
        std::cout << "Name: " << name << "\n";
        std::cout << "Turning Radius: " << turningRadius << "\n";
        std::cout << "Motion Types: ";
        for (const auto& type : motionTypes) {
            std::cout << type << " ";
        }
        std::cout << "\n";
        std::cout << "Motion Lengths: ";
        for (const auto& length : motionLengths) {
            std::cout << length << " ";
        }
        std::cout << "\n\n";
        motionTypes_set.push_back(motionTypes);
        motionLengths_set.push_back(motionLengths);
        name_set.push_back(name);
    }

    json sequence = jsonData["Sequence"];
    for (const auto& step : sequence) {
        complete_path.push_back({scale * waypointsMap[step][0], scale * waypointsMap[step][1]});
    }
    complete_path_.resize(complete_path.size());
    for (int i = 1; i < complete_path.size(); i++) {
      math::Pose pose_temp(complete_path[i][0], complete_path[i][1], atan2(complete_path[i][1] - complete_path[i-1][1], 
        complete_path[i][0] - complete_path[i-1][0]));
        complete_path_[i] = pose_temp;
    }
    for (int i = 1; i < complete_path_.size(); i++) {
      if (complete_path_[i].theta() - complete_path_[i-1].theta() > M_PI) {
        complete_path_[i-1].setTheta( complete_path_[i-1].theta() +  2 * M_PI);
      }
    }
    ros::init(argc, argv, "liom_test_node");
    auto config_ = std::make_shared<PlannerConfig>();
    config_->vehicle.InitializeDiscs();
    auto env = std::make_shared<Environment>(config_);
    auto planner_ = std::make_shared<LiomLocalPlanner>(config_, env);
    ros::NodeHandle nh;
    visualization::Init(nh, "map", "/liom_test_vis");
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/liom_test_path", 1, false);\
    ros::Subscriber integer_sub = nh.subscribe("integer_topic", 10, integerCallback);
    ros::Rate rate(40);
    TrajectoryPoint start, goal;
    std::vector<FullStates> ref_traj_set;
    std::vector<FullStates> solution_set;
    FullStates solution;
    std::vector<Trajectory_temp> traj_all_set;
    Trajectory_temp traj_all;
    std::vector<int> time_set;
    std::vector<int> obs_ind;
    std::vector<bool> is_straight;
    obs_ind.resize(complete_path_.size(), 0);
    traj_all = generate_traj("/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/sarah_traj.yaml");
    readtime_count(time_set, "/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/sarah_time.yaml");
    traj_all_set.resize(time_set.size());
    is_straight.resize(time_set.size(), false);
    std::vector<FullStates> solution_all;
    solution_all.resize(time_set.size());
    std::vector<int> obs_trigger;
    for (int i = 0; i < is_straight.size(); i++) {
      if (time_set[i] == 21) {
        is_straight[i] = true;
        int k = 0;
        for (int j = 0; j < i; j++) {
          k += time_set[j];
        } 
      obs_trigger.push_back(k);
    }
    }
    int j = 0;
    int obs_ind_ = 0;
    while (ros::ok()) {
      DrawPathRivz(complete_path, 0);
      for (int i = 0; i < complete_path.size(); i++) {
          DrawCircle(complete_path[i][0], complete_path[i][1], scale * obstacleRadius, i, obs_ind[i]);
      }
      j = traj_ind;
      if (j > obs_trigger[obs_ind_]) {
        if (is_straight[traj_ind]) {
          obs_ind[obs_ind_++] = 1;
        }
        traj_ind++;
      }
      if (j == traj_all.size() - traj_all_set[traj_all.size() - 1].size()) {
        obs_ind[obs_ind_] = 1;
      }
      if (j == traj_all.size()) {
        break;
      }
      ros::spinOnce();
      rate.sleep();	
	}
    return 0;
}
