#ifndef FRONTIER_SEARCH_H_
#define FRONTIER_SEARCH_H_

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace frontier_exploration
{
/**
 * @brief Represents a frontier
 *
 */
struct Frontier {
  std::uint32_t size;
  // Geodesic (real path) distance in metres from the robot to the frontier's
  // closest reachable approach cell, measured through free space by the wavefront
  // in searchFrom (NOT straight-line), so frontiers behind walls rank as far.
  double min_distance;
  double cost;
  geometry_msgs::msg::Point initial;
  geometry_msgs::msg::Point centroid;
  geometry_msgs::msg::Point middle;
  // Clearance-safe approach point that a navigation goal should target. Computed
  // by projecting the frontier's closest cell into nearby free space that keeps
  // a minimum clearance from lethal obstacles. Valid only if goal_valid is true.
  geometry_msgs::msg::Point goal;
  bool goal_valid = false;
  std::vector<geometry_msgs::msg::Point> points;
};

/**
 * @brief Thread-safe implementation of a frontier-search task for an input
 * costmap.
 */
class FrontierSearch
{
public:
  FrontierSearch() : logger_(rclcpp::get_logger("frontier_search")) {} // Default constructor for the logger

  /**
   * @brief Constructor for search task
   * @param costmap Reference to costmap data to search.
   */
  FrontierSearch(nav2_costmap_2d::Costmap2D* costmap, double potential_scale,
                 double gain_scale, double min_frontier_size,
                 double min_obstacle_clearance, double clearance_search_radius,
                 double frontier_size_cap, double footprint_clearing_radius,
                 rclcpp::Logger logger);

  /**
   * @brief Runs search implementation, outward from the start position
   * @param position Initial position to search from
   * @return List of frontiers, if any
   */
  std::vector<Frontier> searchFrom(geometry_msgs::msg::Point position);

protected:
  /**
   * @brief Starting from an initial cell, build a frontier from valid adjacent
   * cells
   * @param initial_cell Index of cell to start frontier building
   * @param reference Reference index to calculate position from
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return new frontier
   */
  Frontier buildNewFrontier(unsigned int initial_cell, unsigned int reference,
                            std::vector<bool>& frontier_flag);

  /**
   * @brief isNewFrontierCell Evaluate if candidate cell is a valid candidate
   * for a new frontier.
   * @param idx Index of candidate cell
   * @param frontier_flag Flag vector indicating which cells are already marked
   * as frontiers
   * @return true if the cell is frontier cell
   */
  bool isNewFrontierCell(unsigned int idx,
                         const std::vector<bool>& frontier_flag);

  /**
   * @brief computes frontier cost
   * @details cost function is defined by potential_scale and gain_scale
   *
   * @param frontier frontier for which compute the cost
   * @return cost of the frontier
   */
  double frontierCost(const Frontier& frontier);

  /**
   * @brief Project a frontier reference point into nearby free space that keeps
   * a minimum clearance from lethal obstacles.
   * @details Searches concentric rings (out to clearance_search_radius_) around
   * the seed point and returns the first FREE_SPACE cell whose distance to the
   * nearest lethal cell is >= min_obstacle_clearance_. This keeps navigation
   * goals off walls even though the raw SLAM map carries no inflation layer.
   * @param seed Reference point (world coords), typically the frontier's closest
   * cell to the robot.
   * @param result Output clearance-safe point (world coords).
   * @return true if a clearance-safe free point was found.
   */
  bool projectToFree(const geometry_msgs::msg::Point& seed,
                     geometry_msgs::msg::Point& result);

  /**
   * @brief Minimum distance (in meters) from a cell to the nearest lethal cell,
   * capped at min_obstacle_clearance_ (we only need to know if it is "far enough").
   * @param mx,my Cell coordinates.
   * @return Clearance in meters (>= min_obstacle_clearance_ means safe).
   */
  double obstacleClearance(unsigned int mx, unsigned int my);

private:
  nav2_costmap_2d::Costmap2D* costmap_;
  unsigned char* map_;
  unsigned int size_x_, size_y_;
  double potential_scale_, gain_scale_;
  double min_frontier_size_;
  double min_obstacle_clearance_;
  double clearance_search_radius_;
  double frontier_size_cap_;  // cap (in cells) on the size bonus in frontierCost
  // Radius (m) around the robot whose unknown cells are forced to FREE_SPACE
  // before searching, so no frontier is ever detected on/under the robot. 0
  // disables.
  double footprint_clearing_radius_;
  rclcpp::Logger logger_;
};
}  // namespace frontier_exploration
#endif