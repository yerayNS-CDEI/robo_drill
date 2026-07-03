////////////////////////////////
/// ORIGINAL
////////////////////////////////

#include "exploration_base.h"
#include <algorithm>  // for std::sort

class ExplorationClosestFrontier : public ExplorationBase
{
  protected:
    //////////////////////////////////////////////////////////////////////
    // TODO 1a: if you need your own attributes (variables) and/or
    //          methods (functions), define them HERE
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE ATTRIBUTE:
    // double time_max_goal_; // max time to reach a goal before aborting

    double path_length_;    // length of the computed path to the goal
    int i_closest_;    // closest frontier id
    double closest_frontier_dist_;    // minimum distance to the current frontiers
    double dist_goal_;    // cartesian distance to goal
    double dt_last_goal_;    // time elapsed since last goal sent
    double dist_goal_replan_;    // threshold distance to recompute goal
    double dt_last_goal_replan_;    // threshold time elapsed to recompute goal
    
    // Stuck detection
    geometry_msgs::msg::Point last_stuck_check_pos_;
    rclcpp::Time last_stuck_check_time_;
    double stuck_distance_threshold_;  // meters moved to not be considered stuck
    double stuck_time_threshold_;      // seconds before checking if stuck
    
    // Multiple candidate goals tracking
    struct FrontierCandidate {
        int frontier_index;
        double score;
        double path_length;
        geometry_msgs::msg::Pose pose;
    };
    std::vector<FrontierCandidate> candidate_goals_;  // Sorted by score (highest first)
    size_t current_candidate_index_;                   // Which candidate we're currently trying
    int consecutive_failures_;                         // Track failures to clear candidates

    //////////////////////////////////////////////////////////////////////
    // TODO 1a END
    //////////////////////////////////////////////////////////////////////

  public:
    ExplorationClosestFrontier();

  protected:
    bool                replan() override;
    geometry_msgs::msg::Pose decideGoal() override;
};

ExplorationClosestFrontier::ExplorationClosestFrontier() : ExplorationBase("exploration_closest_frontier")
{
    //////////////////////////////////////////////////////////////////////
    // TODO 1b: You can set the value of attributes using ros param
    //          for changing the value without need of recompiling.
    //////////////////////////////////////////////////////////////////////

    // // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
    // // Get the value of the param "time_max_goal" and store it to the attribute 'time_max_goal_'.
    // // If the parameters is not defined, use the default value 20:
    // get_parameter_or("time_max_goal", time_max_goal_, 20);

    // Get the maximum distance to the goal in which the robot recomputes a new goal
    get_parameter_or("dist_goal_replan", dist_goal_replan_, 1.0);

    // Get the maximum amount of time elapsed between goals computation
    get_parameter_or("dt_last_goal_replan", dt_last_goal_replan_, 4.0);
    
    // Stuck detection parameters
    get_parameter_or("stuck_distance_threshold", stuck_distance_threshold_, 1.0);  // 1 meter
    get_parameter_or("stuck_time_threshold", stuck_time_threshold_, 10.0);         // 10 seconds
    
    // Initialize stuck detection
    last_stuck_check_pos_.x = 0.0;
    last_stuck_check_pos_.y = 0.0;
    last_stuck_check_pos_.z = 0.0;
    last_stuck_check_time_ = this->now();
    
    // Initialize candidate tracking
    current_candidate_index_ = 0;
    consecutive_failures_ = 0;

    //////////////////////////////////////////////////////////////////////
    // TODO 1b END
    //////////////////////////////////////////////////////////////////////
}

geometry_msgs::msg::Pose ExplorationClosestFrontier::decideGoal()
{
    geometry_msgs::msg::Pose g;

    ////////////////////////////////////////////////////////////////////
    // IMPROVED: Build sorted list of candidate goals
    // When first goal fails, automatically try second, third, etc.
    ////////////////////////////////////////////////////////////////////

    // Build new candidate list only if:
    // 1. We don't have candidates yet, OR
    // 2. We've exhausted all candidates (tried them all), OR
    // 3. Too many consecutive failures (indicates environment changed)
    bool need_new_candidates = candidate_goals_.empty() || 
                               current_candidate_index_ >= candidate_goals_.size() ||
                               consecutive_failures_ > 3;
    
    if (need_new_candidates) {
        RCLCPP_INFO(this->get_logger(), "Building new candidate list from %zu frontiers", 
                    frontiers_msg_.frontiers.size());
        
        candidate_goals_.clear();
        current_candidate_index_ = 0;
        consecutive_failures_ = 0;
        
        // Evaluate all frontiers and build candidate list
        for (int i = 0; i < static_cast<int>(frontiers_msg_.frontiers.size()); i++)
        {
            g.position = frontiers_msg_.frontiers[i].center_point;
            double path_len = -1.0;
            bool valid = isValidGoal(g, path_len);

            if (!valid) continue;

            // Get frontier size (number of cells in cluster)
            uint32_t frontier_size = frontiers_msg_.frontiers[i].size;

            // Weighted scoring: favor larger frontiers and closer ones
            const double size_weight = 50.0;
            const double distance_weight = 1.0;
            
            double size_score = size_weight * std::log(std::max(1u, frontier_size));
            double distance_score = distance_weight * path_len;
            double total_score = size_score - distance_score;

            // Add to candidate list
            FrontierCandidate candidate;
            candidate.frontier_index = i;
            candidate.score = total_score;
            candidate.path_length = path_len;
            candidate.pose.position = frontiers_msg_.frontiers[i].center_point;
            candidate.pose.orientation = robot_pose_.orientation;
            candidate_goals_.push_back(candidate);

            RCLCPP_INFO(this->get_logger(),
                "[F%d] size=%u path_len=%.2f score=%.2f (size_contrib=%.2f dist_penalty=%.2f)",
                i, frontier_size, path_len, total_score, size_score, distance_score);
        }
        
        // Sort candidates by score (highest first)
        std::sort(candidate_goals_.begin(), candidate_goals_.end(),
                  [](const FrontierCandidate& a, const FrontierCandidate& b) {
                      return a.score > b.score;
                  });
        
        RCLCPP_INFO(this->get_logger(), "Built candidate list with %zu valid goals", 
                    candidate_goals_.size());
        for (size_t i = 0; i < candidate_goals_.size() && i < 5; i++) {
            RCLCPP_INFO(this->get_logger(), "  Candidate %zu: F%d score=%.2f dist=%.2f",
                        i, candidate_goals_[i].frontier_index, 
                        candidate_goals_[i].score, candidate_goals_[i].path_length);
        }
    }
    
    // Select goal from current candidate index
    if (current_candidate_index_ < candidate_goals_.size()) {
        const auto& candidate = candidate_goals_[current_candidate_index_];
        g = candidate.pose;
        i_closest_ = candidate.frontier_index;
        closest_frontier_dist_ = candidate.path_length;
        
        RCLCPP_INFO(this->get_logger(),
            "Selecting candidate %zu/%zu: F%d (score=%.2f, dist=%.2f)",
            current_candidate_index_ + 1, candidate_goals_.size(),
            i_closest_, candidate.score, candidate.path_length);
    } else {
        // No valid candidates available
        i_closest_ = -1;
        RCLCPP_WARN(this->get_logger(),
            "No valid frontier candidates. decideGoalBase() will generate random goal.");
    }

    ////////////////////////////////////////////////////////////////////
    // END IMPROVED FRONTIER SELECTION
    ////////////////////////////////////////////////////////////////////

    return g;
}

bool ExplorationClosestFrontier::replan()
{
    // REMEMBER:
    // goal_time_ has the time since last goal was sent (seconds)
    // goal_distance_ has remaining distance to reach the last goal (meters)
    // robot_status_: 0 = moving, 1 = goal reached, 2 = failed to reach goal

    ////////////////////////////////////////////////////////////////////
    // FIX: Only replan when navigation action completes (reached/failed)
    // or when robot is truly stuck (not moving)
    ////////////////////////////////////////////////////////////////////

    // The robot_status_ is automatically updated by the navigation action callbacks:
    // - Set to 0 (moving) when goal is accepted
    // - Set to 1 (reached) when goal succeeds
    // - Set to 2 (aborted/cancelled) when goal fails
    
    // Stuck detection: if robot hasn't moved stuck_distance_threshold_ in stuck_time_threshold_ seconds
    double time_since_check = (this->now() - last_stuck_check_time_).seconds();
    if (time_since_check >= stuck_time_threshold_)
    {
        double dx = robot_pose_.position.x - last_stuck_check_pos_.x;
        double dy = robot_pose_.position.y - last_stuck_check_pos_.y;
        double distance_moved = std::sqrt(dx*dx + dy*dy);
        
        if (distance_moved < stuck_distance_threshold_)
        {
            RCLCPP_WARN(this->get_logger(), 
                "Robot stuck! Moved only %.2fm in %.1fs. Executing backup before replan.", 
                distance_moved, time_since_check);
            
            // Execute backup to give robot space to maneuver
            const double backup_distance = 0.5; // meters
            if (executeBackup(backup_distance))
            {
                RCLCPP_INFO(this->get_logger(), "Backup successful, now replanning");
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Backup failed, replanning anyway");
            }
            
            // Stuck = current goal is unreachable, try next candidate
            consecutive_failures_++;
            current_candidate_index_++;
            
            if (current_candidate_index_ < candidate_goals_.size()) {
                RCLCPP_INFO(this->get_logger(), 
                    "Stuck detected, advancing to next candidate (%zu/%zu)", 
                    current_candidate_index_ + 1, candidate_goals_.size());
            } else {
                RCLCPP_WARN(this->get_logger(), 
                    "Stuck detected and no more candidates. Will rebuild candidate list.");
            }
            
            // Reset stuck detection
            last_stuck_check_pos_ = robot_pose_.position;
            last_stuck_check_time_ = this->now();
            return true;
        }
        
        // Not stuck, update position and time for next check
        last_stuck_check_pos_ = robot_pose_.position;
        last_stuck_check_time_ = this->now();
    }
    
    // Safety check: if robot has been trying for too long (e.g., 60 seconds) without completion
    const double max_time_for_goal = 60.0; // seconds
    if (robot_status_ == 0 && goal_time_ > max_time_for_goal)
    {
        RCLCPP_WARN(this->get_logger(), "Goal timeout (%.1f seconds). Forcing replan.", goal_time_);
        consecutive_failures_++;
        current_candidate_index_++;  // Try next candidate
        return true;
    }
    
    // Handle goal completion
    if (robot_status_ == 1) {
        // Goal reached successfully
        RCLCPP_INFO(this->get_logger(), "Goal reached successfully! Clearing candidates for fresh evaluation.");
        consecutive_failures_ = 0;
        candidate_goals_.clear();  // Force new candidate evaluation on next cycle
        current_candidate_index_ = 0;
        return true;
    }
    
    if (robot_status_ == 2) {
        // Goal failed - try next candidate from our sorted list
        consecutive_failures_++;
        current_candidate_index_++;
        
        if (current_candidate_index_ < candidate_goals_.size()) {
            RCLCPP_WARN(this->get_logger(), 
                "Goal failed! Trying next candidate (%zu/%zu)", 
                current_candidate_index_ + 1, candidate_goals_.size());
        } else {
            RCLCPP_WARN(this->get_logger(), 
                "Goal failed and no more candidates. Will rebuild candidate list.");
        }
        return true;
    }

    return false;
}

////// MAIN ////////////////////////////////////////////////////////////////////////////
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<ExplorationClosestFrontier>();
  exec.add_node(node);
  exec.spin();

  return 0;
}










// #include "exploration_base.h"

// #include <limits>
// #include <cmath>
// #include <algorithm>
// #include <vector>

// class ExplorationClosestFrontier : public ExplorationBase
// {
//   protected:
//     //////////////////////////////////////////////////////////////////////
//     // TODO 1a: if you need your own attributes (variables) and/or
//     //          methods (functions), define them HERE
//     //////////////////////////////////////////////////////////////////////

//     // // EXAMPLE ATTRIBUTE:
//     // double time_max_goal_; // max time to reach a goal before aborting

//     double path_length_;    // length of the computed path to the goal
//     double i_closest_;    // closest frontier id
//     double closest_frontier_dist_;    // minimum distance to the current frontiers
//     double dist_goal_;    // cartesian distance to goal
//     double dt_last_goal_;    // time elapsed since last goal sent
//     double dist_goal_replan_;    // threshold distance to recompute goal
//     double dt_last_goal_replan_;    // threshold time elapsed to recompute goal

//     double startup_delay_sec_;
//     int    validate_top_k_;
//     rclcpp::Time node_start_time_;

//     int    frontier_free_search_rings_;
//     int    frontier_free_steps_;
//     double frontier_min_clearance_m_;

//     //////////////////////////////////////////////////////////////////////
//     // TODO 1a END
//     //////////////////////////////////////////////////////////////////////

//   public:
//     ExplorationClosestFrontier();

//   protected:
//     bool                replan() override;
//     geometry_msgs::msg::Pose decideGoal() override;
// };

// ExplorationClosestFrontier::ExplorationClosestFrontier() : ExplorationBase("exploration_closest_frontier")
// {
//     //////////////////////////////////////////////////////////////////////
//     // TODO 1b: You can set the value of attributes using ros param
//     //          for changing the value without need of recompiling.
//     //////////////////////////////////////////////////////////////////////

//     // // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
//     // // Get the value of the param "time_max_goal" and store it to the attribute 'time_max_goal_'.
//     // // If the parameters is not defined, use the default value 20:
//     // get_parameter_or("time_max_goal", time_max_goal_, 20);

//     // Get the maximum distance to the goal in which the robot recomputes a new goal
//     get_parameter_or("dist_goal_replan", dist_goal_replan_, 1.0);

//     // Get the maximum amount of time elapsed between goals computation
//     get_parameter_or("dt_last_goal_replan", dt_last_goal_replan_, 4.0);

//     get_parameter_or("startup_delay_sec", startup_delay_sec_, 3.0);
//     get_parameter_or("validate_top_k",     validate_top_k_,     12);
//     node_start_time_ = now();

//     get_parameter_or("frontier_free_search_rings",  frontier_free_search_rings_,  5);  // ~5 celdas (~0.25 m si res=0.05)
//     get_parameter_or("frontier_free_steps",        frontier_free_steps_,        24);  // 24 direcciones
//     get_parameter_or("frontier_min_clearance_m",   frontier_min_clearance_m_,  0.10); // 10 cm de “aire” alrededor

//     //////////////////////////////////////////////////////////////////////
//     // TODO 1b END
//     //////////////////////////////////////////////////////////////////////
// }

// geometry_msgs::msg::Pose ExplorationClosestFrontier::decideGoal()
// {
//     geometry_msgs::msg::Pose g;

//     auto isFreeWithClearance = [&](const geometry_msgs::msg::Point& p)->bool {
//       if (!isFree(p)) return false;
//       // comprueba 8-vecindad a una distancia mínima
//       const double res = map_.info.resolution;
//       const double r   = std::max(res, frontier_min_clearance_m_);
//       for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
//         if (dx==0 && dy==0) continue;
//         geometry_msgs::msg::Point q{p.x + dx*r, p.y + dy*r, 0.0};
//         if (!isFree(q)) return false;
//       }
//       return true;
//     };

//     auto nearestFreeAround = [&](const geometry_msgs::msg::Point& c,
//                                 geometry_msgs::msg::Point& best)->bool {
//       const double res = map_.info.resolution;
//       for (int ring = 0; ring <= frontier_free_search_rings_; ++ring) {
//         double R = ring * res; // 0, 1*res, 2*res, ...
//         int K    = std::max(8, frontier_free_steps_);
//         for (int k = 0; k < K; ++k) {
//           double th = (2.0 * M_PI * k) / K;
//           geometry_msgs::msg::Point p{ c.x + R*std::cos(th), c.y + R*std::sin(th), 0.0 };
//           if (isFreeWithClearance(p)) { best = p; return true; }
//         }
//       }
//       return false;
//     };

//     ////////////////////////////////////////////////////////////////////
//     // TODO 2: decide goal
//     ////////////////////////////////////////////////////////////////////

//     // // EXAMPLE iterating over detected frontiers
//     // for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
//     // {
//     //   // Accessing different fields
//     //   frontiers_msg_.frontiers[i].size;
//     //   frontiers_msg_.frontiers[i].center_point.x;
//     //   frontiers_msg_.frontiers[i].center_point.y;
//     //   frontiers_msg_.frontiers[i].center_point.z;
//     // }

//     // // EXAMPLE filling Pose message
//     // // The goal position can be filled with the center_point of the "best" frontier
//     // g.position = frontiers_msg_.frontiers[i_best].center_free_point;
//     //
//     // // The orientation has to be filled as well.
//     // g.orientation = robot_pose_.orientation;             // EXAMPLE1: the same orientation as the current one
//     // g.orientation = tf::createQuaternionMsgFromYaw(0.0); // EXAMPLE2: zero yaw

//     // // EXAMPLE check if a goal is valid and get path length to the goal
//     // double path_length;
//     // bool valid = isValidGoal(g, path_length)

//     // 0) Sin fronteras -> usa pose del robot (no fuerces random)
//     if (frontiers_msg_.frontiers.empty()) {
//       RCLCPP_WARN(this->get_logger(), "[exploration] No frontiers available.");
//       return robot_pose_;
//     }

//     // 1) Ordena por distancia euclídea y valida sólo las K más cercanas
//     std::vector<std::pair<double,int>> cand;
//     cand.reserve(frontiers_msg_.frontiers.size());
//     for (int i = 0; i < (int)frontiers_msg_.frontiers.size(); ++i) {
//       const auto& p = frontiers_msg_.frontiers[i].center_point;
//       double dx = p.x - robot_pose_.position.x;
//       double dy = p.y - robot_pose_.position.y;
//       cand.emplace_back(dx*dx + dy*dy, i);
//     }
//     std::sort(cand.begin(), cand.end(), [](auto& a, auto& b){ return a.first < b.first; });

//     closest_frontier_dist_ = std::numeric_limits<double>::infinity();
//     i_closest_ = -1;
//     geometry_msgs::msg::Pose g_best;

//     const int K = std::max(1, validate_top_k_);
//     for (int j = 0; j < (int)cand.size() && j < K; ++j) {
//       int i = cand[j].second;
      
//       // 0) buscar punto libre cercano al centro (descarta columnas)
//       geometry_msgs::msg::Point p_free;
//       if (!nearestFreeAround(frontiers_msg_.frontiers[i].center_point, p_free)) {
//         // columna o encerrada -> ignora esta frontera
//         continue;
//       }

//       // 1) candidato sobre celda libre
//       g.position = p_free;
//       g.position.z = 0.0;

//       // orientar mirando al objetivo
//       double yaw = std::atan2(g.position.y - robot_pose_.position.y,
//                               g.position.x - robot_pose_.position.x);
//       tf2::Quaternion q; q.setRPY(0,0,yaw);
//       g.orientation = tf2::toMsg(q);

//       // 2) valida y quédate con el mejor por longitud de camino
//       if (isValidGoal(g, path_length_) && path_length_ < closest_frontier_dist_) {
//         closest_frontier_dist_ = path_length_;
//         i_closest_ = i;
//         g_best = g;
//       }
//     }

//     // 2) Si ninguna pasó el validador, usa la más cercana euclídea sin invalidar índices
//     if (i_closest_ < 0) {
//       int i = cand.front().second;
//       geometry_msgs::msg::Point p;
//       if (!nearestFreeAround(frontiers_msg_.frontiers[i].center_point, p)) {
//         // ni siquiera hay libre cerca: devuelve robot_pose_ y deja que la base haga random
//         RCLCPP_WARN(this->get_logger(), "[exploration] Frontier near (%0.2f,%0.2f) has no free neighbors; skipping",
//                     frontiers_msg_.frontiers[i].center_point.x,
//                     frontiers_msg_.frontiers[i].center_point.y);
//         return robot_pose_;
//       }
//       g_best.position = p;
//       g_best.position.z = 0.0;
//       double yaw = std::atan2(p.y - robot_pose_.position.y,
//                               p.x - robot_pose_.position.x);
//       tf2::Quaternion q; q.setRPY(0,0,yaw);
//       g_best.orientation = tf2::toMsg(q);
//     }

//     // Revalidación final + pequeño empujón hacia el robot si aún no pasa
//     double _L;
//     if (!isValidGoal(g_best, _L)) {
//       auto p = g_best.position;
//       const auto& rp = robot_pose_.position;
//       double dx = rp.x - p.x, dy = rp.y - p.y;
//       double n = std::hypot(dx, dy);
//       if (n > 1e-6) {
//         p.x += 0.20 * dx / n;
//         p.y += 0.20 * dy / n;
//         p.z  = 0.0;
//         g_best.position = p;
//         double yaw = std::atan2(p.y - rp.y, p.x - rp.x);
//         tf2::Quaternion q; q.setRPY(0,0,yaw);
//         g_best.orientation = tf2::toMsg(q);
//         (void) isValidGoal(g_best, _L); // segundo intento
//       }
//     }
//     ////////////////////////////////////////////////////////////////////
//     // TODO 2 END
//     ////////////////////////////////////////////////////////////////////

//     return g_best;
// }

// bool ExplorationClosestFrontier::replan()
// {
//     // REMEMBER:
//     // goal_time_ has the time since last goal was sent (seconds)
//     // goal_distance_ has remaining distance to reach the last goal (meters)

//     ////////////////////////////////////////////////////////////////////
//     // TODO 3: replan
//     ////////////////////////////////////////////////////////////////////

//     if ((now() - node_start_time_).seconds() < startup_delay_sec_) {
//       return false;
//     }

//     // Recompute a goal if the robot is near the previous one
//     // or more time than the specified has passed since last computation
//     if (goal_distance_ < dist_goal_replan_ || goal_time_ > dt_last_goal_replan_){
//       robot_status_ = 1;
//     }
    
//     ////////////////////////////////////////////////////////////////////
//     // TODO 3 END
//     ////////////////////////////////////////////////////////////////////

//     // Replan ANYWAY if the robot reached or aborted the goal (DO NOT ERASE THE FOLLOWING LINES)
//     if (robot_status_ != 0) return true;

//     return false;
// }

// ////// MAIN ////////////////////////////////////////////////////////////////////////////
// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::executors::MultiThreadedExecutor exec;
//   auto node = std::make_shared<ExplorationClosestFrontier>();
//   exec.add_node(node);
//   exec.spin();

//   return 0;
// }















// #include "exploration_base.h"
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// #include <limits>
// #include <cmath>

// class ExplorationClosestFrontier : public ExplorationBase
// {
//   protected:
//     //////////////////////////////////////////////////////////////////////
//     // TODO 1a: if you need your own attributes (variables) and/or
//     //          methods (functions), define them HERE
//     //////////////////////////////////////////////////////////////////////

//     // // EXAMPLE ATTRIBUTE:
//     // double time_max_goal_; // max time to reach a goal before aborting

//     double path_length_;    // length of the computed path to the goal
//     int i_closest_;    // closest frontier id
//     double closest_frontier_dist_;    // minimum distance to the current frontiers
//     double dist_goal_;    // cartesian distance to goal
//     double dt_last_goal_;    // time elapsed since last goal sent
//     double dist_goal_replan_;    // threshold distance to recompute goal
//     double dt_last_goal_replan_;    // threshold time elapsed to recompute goal

//     // Snapping del goal cerca del centro de la frontera
//     double goal_snap_max_radius_;    // m (p. ej., 0.6)
//     double goal_snap_step_;          // m (p. ej., 0.1)
//     int    goal_snap_num_angles_;    // divisiones angulares (p. ej., 16)
//     double goal_nudge_towards_robot_; // m, empuja el goal ligeramente hacia el robot (p. ej., 0.15)

//     //////////////////////////////////////////////////////////////////////
//     // TODO 1a END
//     //////////////////////////////////////////////////////////////////////

//   public:
//     ExplorationClosestFrontier();

//   protected:
//     bool                replan() override;
//     geometry_msgs::msg::Pose decideGoal() override;
// };

// ExplorationClosestFrontier::ExplorationClosestFrontier() : ExplorationBase("exploration_closest_frontier")
// {
//     //////////////////////////////////////////////////////////////////////
//     // TODO 1b: You can set the value of attributes using ros param
//     //          for changing the value without need of recompiling.
//     //////////////////////////////////////////////////////////////////////

//     // // EXAMPLE FOR LOADING PARAMS TO YOUR ATTRIBUTES:
//     // // Get the value of the param "time_max_goal" and store it to the attribute 'time_max_goal_'.
//     // // If the parameters is not defined, use the default value 20:
//     // get_parameter_or("time_max_goal", time_max_goal_, 20);

//     // Get the maximum distance to the goal in which the robot recomputes a new goal
//     get_parameter_or("dist_goal_replan", dist_goal_replan_, 1.0);

//     // Get the maximum amount of time elapsed between goals computation
//     get_parameter_or("dt_last_goal_replan", dt_last_goal_replan_, 4.0);

//     get_parameter_or("goal_snap_max_radius",     goal_snap_max_radius_,     0.8);
//     get_parameter_or("goal_snap_step",           goal_snap_step_,           0.15);
//     get_parameter_or("goal_snap_num_angles",     goal_snap_num_angles_,     24);
//     get_parameter_or("goal_nudge_towards_robot", goal_nudge_towards_robot_, 0.25);

//     this->set_parameter(rclcpp::Parameter("robot_base_frame", "turret_link"));
//     this->set_parameter(rclcpp::Parameter("robot_frame", "turret_link"));
//     this->set_parameter(rclcpp::Parameter("global_frame", "map"));
//     this->set_parameter(rclcpp::Parameter("odom_frame", "odom"));

//     this->set_parameter(rclcpp::Parameter("tf_timeout", 0.2));

//     //////////////////////////////////////////////////////////////////////
//     // TODO 1b END
//     //////////////////////////////////////////////////////////////////////
// }

// geometry_msgs::msg::Pose ExplorationClosestFrontier::decideGoal()
// {
//     geometry_msgs::msg::Pose g;

//     ////////////////////////////////////////////////////////////////////
//     // TODO 2: decide goal
//     ////////////////////////////////////////////////////////////////////

//     // // EXAMPLE iterating over detected frontiers
//     // for (unsigned int i = 0; i < frontiers_msg_.frontiers.size(); i++)
//     // {
//     //   // Accessing different fields
//     //   frontiers_msg_.frontiers[i].size;
//     //   frontiers_msg_.frontiers[i].center_point.x;
//     //   frontiers_msg_.frontiers[i].center_point.y;
//     //   frontiers_msg_.frontiers[i].center_point.z;
//     // }

//     // // EXAMPLE filling Pose message
//     // // The goal position can be filled with the center_point of the "best" frontier
//     // g.position = frontiers_msg_.frontiers[i_best].center_free_point;
//     //
//     // // The orientation has to be filled as well.
//     // g.orientation = robot_pose_.orientation;             // EXAMPLE1: the same orientation as the current one
//     // g.orientation = tf::createQuaternionMsgFromYaw(0.0); // EXAMPLE2: zero yaw

//     // // EXAMPLE check if a goal is valid and get path length to the goal
//     // double path_length;
//     // bool valid = isValidGoal(g, path_length)

//     if (frontiers_msg_.frontiers.empty()) {
//         RCLCPP_WARN(this->get_logger(), "[exploration] No frontiers available.");
//         return robot_pose_;
//     }

//     auto make_pose = [&](const geometry_msgs::msg::Point& p)->geometry_msgs::msg::Pose {
//       geometry_msgs::msg::Pose out;
//       out.position = p;
//       // orientar hacia el goal (mejora la validez en algunos validadores)
//       double yaw = std::atan2(p.y - robot_pose_.position.y, p.x - robot_pose_.position.x);
//       tf2::Quaternion q; q.setRPY(0,0,yaw);
//       out.orientation = tf2::toMsg(q);
//       return out;
//     };

//     auto snap_goal = [&](const geometry_msgs::msg::Point& center,
//                         geometry_msgs::msg::Pose& snapped,
//                         double& best_len)->bool {
//       // nudge hacia el robot
//       geometry_msgs::msg::Point seed = center;
//       {
//         double vx = center.x - robot_pose_.position.x;
//         double vy = center.y - robot_pose_.position.y;
//         double n  = std::hypot(vx, vy);
//         if (n > 1e-6) {
//           seed.x -= goal_nudge_towards_robot_ * (vx / n);
//           seed.y -= goal_nudge_towards_robot_ * (vy / n);
//         }
//       }
//       geometry_msgs::msg::Pose cand = make_pose(seed);
//       if (isValidGoal(cand, best_len)) { snapped = cand; return true; }

//       // anillos concéntricos
//       const int   K   = std::max(4, goal_snap_num_angles_);
//       const double Rm = std::max(0.0, goal_snap_max_radius_);
//       const double dR = std::max(0.02, goal_snap_step_);

//       bool found = false;
//       double best = std::numeric_limits<double>::infinity();
//       geometry_msgs::msg::Pose best_pose;

//       for (double r = dR; r <= Rm + 1e-6; r += dR) {
//         for (int k = 0; k < K; ++k) {
//           double th = (2.0 * M_PI * k) / K;
//           geometry_msgs::msg::Point p;
//           p.x = seed.x + r * std::cos(th);
//           p.y = seed.y + r * std::sin(th);
//           p.z = 0.0;
//           geometry_msgs::msg::Pose cand_k = make_pose(p);
//           double len_k;
//           if (isValidGoal(cand_k, len_k) && len_k < best) {
//             best = len_k; best_pose = cand_k; found = true;
//           }
//         }
//         if (found && best < 0.5 * r) break; // salir temprano si ya tenemos uno bueno
//       }
//       if (found) { snapped = best_pose; best_len = best; }
//       return found;
//     };

//     closest_frontier_dist_ = 1e12;
//     i_closest_ = -1;
//     geometry_msgs::msg::Pose g_best;

//     for (int i = 0; i < (int)frontiers_msg_.frontiers.size(); ++i) {
//       const auto& f = frontiers_msg_.frontiers[i];
//       geometry_msgs::msg::Pose g_try;
//       double len_try;
//       bool ok = snap_goal(f.center_point, g_try, len_try);
//       if (ok && len_try < closest_frontier_dist_) {
//         closest_frontier_dist_ = len_try;
//         i_closest_ = i;
//         g_best = g_try;
//       }
//     }
//     RCLCPP_INFO(this->get_logger(),
//             "[exploration] Picked frontier id=%d, path=%.2f m, goal=(%.2f, %.2f)",
//             i_closest_, closest_frontier_dist_, g_best.position.x, g_best.position.y);


//     // Fallback: euclídea si ninguna pasó el validador
//     if (i_closest_ < 0) {
//       double best_d2 = std::numeric_limits<double>::infinity();
//       int best_i = -1;
//       for (int i = 0; i < (int)frontiers_msg_.frontiers.size(); ++i) {
//         const auto& p = frontiers_msg_.frontiers[i].center_point;
//         double dx = p.x - robot_pose_.position.x;
//         double dy = p.y - robot_pose_.position.y;
//         double d2 = dx*dx + dy*dy;
//         if (d2 < best_d2) { best_d2 = d2; best_i = i; }
//       }
//       if (best_i >= 0) {
//         i_closest_ = best_i;
//         g_best = make_pose(frontiers_msg_.frontiers[best_i].center_point);
//         RCLCPP_WARN(this->get_logger(),
//           "[exploration] All snapped goals invalid; falling back to Euclidean id=%d", i_closest_);
//       } else {
//         // último salvavidas: NO devuelvas (0,0,0), usa robot_pose_
//         RCLCPP_WARN(this->get_logger(), "[exploration] No usable frontier; returning robot pose.");
//         return robot_pose_;
//       }
//     }

//     ////////////////////////////////////////////////////////////////////
//     // TODO 2 END
//     ////////////////////////////////////////////////////////////////////

//     double _relen;
//     if (!isValidGoal(g_best, _relen)) {
//       // nudge 0.20 m hacia el robot para sacar el goal del borde de obstáculo/desconocido
//       geometry_msgs::msg::Point p = g_best.position;
//       double dx = robot_pose_.position.x - p.x, dy = robot_pose_.position.y - p.y;
//       double n = std::hypot(dx, dy);
//       if (n > 1e-6) { p.x += 0.20 * dx / n; p.y += 0.20 * dy / n; }
//       geometry_msgs::msg::Pose g_retry = make_pose(p);
//       if (isValidGoal(g_retry, _relen)) { g_best = g_retry; }
//     }
//     return g_best;
// }

// bool ExplorationClosestFrontier::replan()
// {
//     // REMEMBER:
//     // goal_time_ has the time since last goal was sent (seconds)
//     // goal_distance_ has remaining distance to reach the last goal (meters)

//     ////////////////////////////////////////////////////////////////////
//     // TODO 3: replan
//     ////////////////////////////////////////////////////////////////////

//     // Recompute a goal if the robot is near the previous one
//     // or more time than the specified has passed since last computation
//     if (goal_distance_ < dist_goal_replan_ || goal_time_ > dt_last_goal_replan_){
//       robot_status_ = 1;
//     }
    
//     ////////////////////////////////////////////////////////////////////
//     // TODO 3 END
//     ////////////////////////////////////////////////////////////////////

//     // Replan ANYWAY if the robot reached or aborted the goal (DO NOT ERASE THE FOLLOWING LINES)
//     if (robot_status_ != 0) return true;

//     return false;
// }

// ////// MAIN ////////////////////////////////////////////////////////////////////////////
// int main(int argc, char *argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::executors::MultiThreadedExecutor exec;
//   auto node = std::make_shared<ExplorationClosestFrontier>();
//   exec.add_node(node);
//   exec.spin();

//   return 0;
// }
