/**
 * file animation.cpp
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief formation planning algorithm test
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <ros/ros.h>
#include <memory>
#include "liom_local_planner/liom_local_planner.h"
#include "liom_local_planner/visualization/plot.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

#include "iris/iris.h"
#include <Eigen/Core>
#include "liom_local_planner/yaml_all.h"
#include "liom_local_planner/math/generate_obs.h"
#include <Eigen/Core>
#include <math.h>

using namespace liom_local_planner;

visualization_msgs::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.name = "Obstacle " + std::to_string(i);
  // marker.pose.position.x = polygon.center().x();
  // marker.pose.position.y = polygon.center().y();
  marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker polygon_marker;
  polygon_marker.header.frame_id = marker.header.frame_id;
  polygon_marker.header.stamp = ros::Time();
  polygon_marker.ns = "Obstacles";
  polygon_marker.id = i;

  polygon_marker.action = visualization_msgs::Marker::ADD;
  polygon_marker.type = visualization_msgs::Marker::LINE_STRIP;
  polygon_marker.pose.orientation.w = 1.0;
  polygon_marker.scale.x = width;
  polygon_marker.color = c.toColorRGBA();

  for (size_t i = 0; i < polygon.num_points(); i++) {
    geometry_msgs::Point pt;
    pt.x = polygon.points().at(i).x();
    pt.y = polygon.points().at(i).y();
    polygon_marker.points.push_back(pt);
  }
  polygon_marker.points.push_back(polygon_marker.points.front());

  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(polygon_marker);

  marker.controls.push_back(box_control);

  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  marker.controls.push_back(move_control);
  return marker;
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
    color.set_alpha(0.4);
    visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, robot_name);
  }
  path_pub.publish(msg);
  visualization::Trigger();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "liom_test_node");

  auto config_ = std::make_shared<PlannerConfig>();
  config_->vehicle.InitializeDiscs();

  auto env = std::make_shared<Environment>(config_);
  auto planner_ = std::make_shared<LiomLocalPlanner>(config_, env);

  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/liom_test_path", 1, false);
  ros::Publisher path_pub_diff1 = nh.advertise<nav_msgs::Path>("/liom_test_path_diff1", 1, false);
  ros::Publisher path_pub_diff2 = nh.advertise<nav_msgs::Path>("/liom_test_path_diff2", 1, false);
  ros::Publisher path_pub_car1 = nh.advertise<nav_msgs::Path>("/liom_test_path_car1", 1, false);

  interactive_markers::InteractiveMarkerServer server_("/liom_obstacle");
  math::GenerateObstacle generateobs;
  std::vector<math::Pose> obstacle;
  std::vector<math::Polygon2d> polys, polys_orig;
  std::vector<std::vector<math::Vec2d>> poly_vertices_set;

  // double inflated_radius;
  // config_->formation.calculateMinCircle(inflated_radius, 0.0, 0.0);
  // for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
  //   polys.push_back(math::Polygon2d(env->InflateObstacle(poly_vertices_set[obs_ind], inflated_radius)));
  // }
  iris::IRISProblem iris_problem(2);
  // Eigen::MatrixXd obs = Eigen::MatrixXd::Zero(2, 4);
  // for(int i = 0; i < polys.size(); i++) {
  //   obs << polys[i].points()[0].x(), polys[i].points()[1].x(), polys[i].points()[2].x(), polys[i].points()[3].x(),
  //           polys[i].points()[0].y(), polys[i].points()[1].y(), polys[i].points()[2].y(), polys[i].points()[3].y();
  //   iris_problem.addObstacle(obs);
  // }
  // add boundary
  // obs << -52, -52, -50, -50,
  //       -50, 50, 50, -50;
  // iris_problem.addObstacle(obs);
  // obs << -50, -50, 50, 50,
  //       50, 52, 52, 50;
  // iris_problem.addObstacle(obs);
  // obs << 50, 52, 52, 50,
  //       50, 50, -50, -10;
  // iris_problem.addObstacle(obs);
  // obs << -50, -50, 50, 50,
  //       -50, -52, -52, -50;
  // iris_problem.addObstacle(obs);
  // std::vector<math::Polygon2d> polys = {
  //     math::Polygon2d({{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}),
  // };
  poly_vertices_set.push_back({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}});
  poly_vertices_set.push_back({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}});
  poly_vertices_set.push_back({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}});
  poly_vertices_set.push_back({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}});

  polys.push_back(math::Polygon2d({{-30.5, -30}, {-30.5, 30}, {-30, 30}, {-30, -30}}));
  polys.push_back(math::Polygon2d({{-30, 30}, {-30, 30.5}, {30, 30.5}, {30, 30}}));
  polys.push_back(math::Polygon2d({{30, 30}, {30.5, 30}, {30.5, -30}, {30, -30}}));
  polys.push_back(math::Polygon2d({{-30, -30}, {-30, -30.5}, {30, -30.5}, {30, -30}}));
  // obstacle.push_back({0, 0, 0});
  // obstacle.push_back({0, 17, 0});
  // obstacle.push_back({0, -17, 0});
  // obstacle.push_back({-30, 30, 0.1});
	// obstacle.push_back({-15, 30, 0.8});
	// obstacle.push_back({0, 30, 0.3});
	// obstacle.push_back({15, 30, 0.56});
	// obstacle.push_back({30, 30, 0.26});
	// obstacle.push_back({-30, -30, 0.75});
	// obstacle.push_back({-15, -30, 0.83});
	// obstacle.push_back({0, -30, 0.34});
	// obstacle.push_back({15, -30, 0.2});
	// obstacle.push_back({30, -30, 0.98});
	// obstacle.push_back({-30, 15, 0.25});
	// obstacle.push_back({-15, 15, 0.34});
	// obstacle.push_back({0, 15, 0.63});
	// obstacle.push_back({15, 15, 0.45});
	// obstacle.push_back({30, 15, 0.72});
	// obstacle.push_back({-30, -15, 0.23});
	// obstacle.push_back({-15, -15, 0.62});
	// obstacle.push_back({0, -15, 0.27});
	// obstacle.push_back({15, -15, 0.86});
	// obstacle.push_back({30, -15, 0.56});
	// obstacle.push_back({22.5, 0, 0.25});
	// obstacle.push_back({-22.5, 0, 0.89});
  for (int obs_ind = 0; obs_ind < obstacle.size(); obs_ind++) {
    math::Vec2d centre(obstacle[obs_ind].x(), obstacle[obs_ind].y());
    std::vector<math::Vec2d> poly_vertices = generateobs.rotatePoint(centre, obstacle[obs_ind].theta(), false);
    poly_vertices_set.push_back(poly_vertices);
    polys.push_back(math::Polygon2d(poly_vertices));
  } 
  // auto interactive_cb = [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
  //   int idx = msg->marker_name.back() - '1';

  //   auto new_poly = polys[idx];
  //   new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
  //   env->polygons().at(idx) = new_poly;
  // };
  writeObstacleToYAML(poly_vertices_set, "/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/obs.yaml");
  env->polygons() = polys;
  // for(int i = 0; i < polys.size(); i++) {
  //   auto marker = CreateMarker(i+1, polys[i], 0.2, visualization::Color::Magenta);
  //   server_.insert(marker, interactive_cb);
  // }

  // server_.applyChanges();

  visualization::Init(nh, "map", "/liom_test_vis");

  TrajectoryPoint start, goal;
  FullStates solution;
  FullStates solution_car_like1, solution_diff_drive1, solution_diff_drive2;
  double max_error;
  // start.x = -28;
  // start.y = 12.5;
  // start.theta = M_PI / 2;
  // goal.x = -10;
  // goal.y = 28;
  // goal.theta = 0;
  start.x = 3;
  start.y = 25;
  start.theta = -M_PI / 2;
  goal.x = 3;
  goal.y = -25;
  goal.theta = -M_PI / 2;
  ros::Rate r(10);

  auto start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, [&](const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
    double x = pose->pose.pose.position.x;
    double y = pose->pose.pose.position.y;

    double min_distance = DBL_MAX;
    int idx = -1;
    for(int i = 0; i < solution.states.size(); i++) {
      double distance = hypot(solution.states[i].x - x, solution.states[i].y - y);
      if(distance < min_distance) {
        min_distance = distance;
        idx = i;
      }
    }

    start = solution.states[idx];
  });

  while(ros::ok()) { 
    for (int i = 0; i < 4; i++) {
      visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Red, i, "Boundary"+  std::to_string(i));
    }
    for (int i = 4; i < polys.size(); i++) {
      visualization::PlotPolygon(polys[i], 0.5, visualization::Color::Magenta, i, "Obstacle"+  std::to_string(i));
    }
    visualization::Trigger();   
    if(!planner_->Plan(solution, start, goal, iris_problem, solution)) {
      break;
    }
    // auto traj_leader = generate_traj("/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/traj_leader.yaml");
    auto traj_leader = fulltotraj(solution);
    auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, hypot(-18-(-20), -21.0083668325-(-20)), atan2(21.0083668325-20, 20-18));
    // auto traj_fol_diff1 = generate_ref_traj_diff(traj_leader, hypot(-10-(-10), -8.0-(-10.0)), atan2(10.0-10.0, 10.0-8.0));
    // auto traj_fol_diff2 = generate_ref_traj_diff(traj_leader, hypot(-10-(-12.016733665), -10.0-(-8.0)), atan2(12.016733665-10, 10-8));
    if(!planner_->Plan_diff_drive(traj_fol_diff1, solution_diff_drive1, start, goal, solution_diff_drive1, 1, max_error)) {
      break;
    }
    double max_error = 0.0, avg_error = 0.0, error_time = 0.0;
    for (int i = 0; i < solution_diff_drive1.states.size(); i++) {
      error_time = hypot(traj_fol_diff1.states[i].x - solution_diff_drive1.states[i].x, traj_fol_diff1.states[i].y - solution_diff_drive1.states[i].y);
      if (error_time > max_error) {
        max_error = error_time;
      }
      avg_error += error_time;
    }
    if (!planner_->Plan_car_like(solution, 2.016733665, solution_car_like1)) {
      break;
    }
    // if(!planner_->Plan_diff_drive(traj_fol_diff2, start, goal, solution_diff_drive2, 1)) {
    //   break;
    // }
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/traj_leader.yaml");
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/traj_diff1.yaml");
    // // delete_yaml("/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/traj_diff2.yaml");
    // delete_yaml("/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/traj_car1.yaml");
    if (max_error < 0.25) {
      writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/five_robots_linear/trl_6.yaml");
    }
    // writeTrajectoryToYAML(solution, "/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/traj_leader.yaml");
    // writeTrajectoryToYAML(solution_diff_drive1, "/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/traj_diff1.yaml");
    // writeTrajectoryToYAML(solution_car_like1, "/home/weijian/Heterogeneous_formation/src/liom_local_planner/traj_result/traj_car1.yaml");
    // for (int i = 0; i < solution.states.size(); i++) {
    // auto x0_disc = config_->vehicle.GetVertexPositions(solution.states[i].x, solution.states[i].y, solution.states[i].theta);
    //   std::vector<double> x1, y1;
    //   x1.push_back(x0_disc[0]);x1.push_back(x0_disc[2]);x1.push_back(x0_disc[4]);x1.push_back(x0_disc[6]);x1.push_back(x0_disc[0]);
    //   y1.push_back(x0_disc[1]);y1.push_back(x0_disc[3]);y1.push_back(x0_disc[5]);y1.push_back(x0_disc[7]);y1.push_back(x0_disc[1]);
    //   visualization::Plot(x1, y1, 0.2, visualization::Color::Red, i, "convex_hall");
    // }
    // visualization::Trigger();   
    DrawTrajectoryRviz(solution, config_, 0, path_pub);
    DrawTrajectoryRviz(solution_car_like1, config_, 1, path_pub_car1);
    DrawTrajectoryRviz(solution_diff_drive1, config_, 2, path_pub_diff1);
    // DrawTrajectoryRviz(solution_diff_drive2, config_, 3, path_pub_diff2);
    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
  return 0;
}