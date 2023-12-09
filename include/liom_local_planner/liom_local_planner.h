/**
 * file liom_local_planner.h
 * author Weijian Zhang (wxz163@student.bham.ac.uk)
 * brief head file for formation planning
 * data 2023-11-22
 * 
 * @copyright Copyroght(c) 2023
*/
#include <memory>
#include "optimizer_interface.h"
#include "planner_config.h"
#include "environment.h"
#include "coarse_path_planner.h"
#include <vector>
#include "iris/iris.h"

#ifndef SRC_LIOM_LOCAL_PLANNER_H
#define SRC_LIOM_LOCAL_PLANNER_H

namespace liom_local_planner {

class LiomLocalPlanner {
public:
  explicit LiomLocalPlanner(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env);

  void set_global_path(const std::vector<math::Pose> &path) {
    global_path_ = path;
  }

  bool Plan(const FullStates &prev_solution, const TrajectoryPoint &start, const TrajectoryPoint &goal, iris::IRISProblem &iris_problem, FullStates &result);
  bool Plan_car_like(const FullStates &prev_solution, const double offset, FullStates &result);
  bool Plan_diff_drive(const FullStates &guess, const FullStates &prev_solution, const TrajectoryPoint &start, const TrajectoryPoint &goal, FullStates &result, const int robot_index, double& max_error);
  bool Plan_car_like_replan(const FullStates &guess, const FullStates &prev_solution, FullStates &result, const int robot_index, double& infeasible);

private:
  std::shared_ptr<PlannerConfig> config_;
  std::shared_ptr<Environment> env_;
  std::shared_ptr<IOptimizer> problem_;

  std::vector<math::Pose> global_path_;

  CoarsePathPlanner coarse_path_planner_;

  bool CheckGuessFeasibility(const FullStates &guess);

  FullStates StitchPreviousSolution(const FullStates &solution, const TrajectoryPoint &start);

  FullStates GenerateGuessFromPath(const std::vector<math::Pose> &path, const TrajectoryPoint &start);

  bool CheckCarKinematic(const FullStates &current_result, const std::vector<double> offset_car);

  bool CheckDiffDriveKinematic(const FullStates &current_result, const std::vector<double> offset, const std::vector<double> rephi);

  FullStates ResamplePath(const std::vector<math::Pose> &path) const;

  std::vector<double> GenerateOptimalTimeProfileSegment(const std::vector<double> &stations, double start_time) const;
};

}

#endif //SRC_LIOM_LOCAL_PLANNER_H
