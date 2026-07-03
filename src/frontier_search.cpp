#include <robo_drill/costmap_tools.h>
#include <robo_drill/frontier_search.h>

#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <cmath>
#include <mutex>

#include "nav2_costmap_2d/cost_values.hpp"

namespace frontier_exploration
{
using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

FrontierSearch::FrontierSearch(nav2_costmap_2d::Costmap2D* costmap,
                               double potential_scale, double gain_scale,
                               double min_frontier_size,
                               double min_obstacle_clearance,
                               double clearance_search_radius,
                               double frontier_size_cap,
                               double footprint_clearing_radius,
                               rclcpp::Logger logger)
  : costmap_(costmap)
  , potential_scale_(potential_scale)
  , gain_scale_(gain_scale)
  , min_frontier_size_(min_frontier_size)
  , min_obstacle_clearance_(min_obstacle_clearance)
  , clearance_search_radius_(clearance_search_radius)
  , frontier_size_cap_(frontier_size_cap)
  , footprint_clearing_radius_(footprint_clearing_radius)
  , logger_(logger)
{
}

std::vector<Frontier>
FrontierSearch::searchFrom(geometry_msgs::msg::Point position)
{
  std::vector<Frontier> frontier_list;

  // Sanity check that robot is inside costmap bounds before searching
  unsigned int mx, my;
  if (!costmap_->worldToMap(position.x, position.y, mx, my)) {
    RCLCPP_ERROR(logger_, "[FrontierSearch] Robot out of costmap bounds, cannot search for frontiers");
    return frontier_list;
  }

  // make sure map is consistent and locked for duration of search
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> lock(
      *(costmap_->getMutex()));

  map_ = costmap_->getCharMap();
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  // Clear the robot's footprint to free space so no frontier is ever detected
  // on/under the robot. The space the robot physically occupies is necessarily
  // free, but the raw SLAM map may still mark it unknown at startup, which spawns
  // frontiers (and thus goals) inside the footprint that Nav2 cannot drive to.
  // Only UNKNOWN cells are converted; genuine obstacles are left untouched so a
  // real obstacle hugging the robot is never erased. This mutates the explore
  // node's own costmap copy (refreshed from /map each cycle), not the SLAM map.
  if (footprint_clearing_radius_ > 0.0) {
    const double resolution = costmap_->getResolution();
    const int rad = static_cast<int>(
        std::ceil(footprint_clearing_radius_ / resolution));
    const int rad2 = rad * rad;
    for (int dy = -rad; dy <= rad; ++dy) {
      for (int dx = -rad; dx <= rad; ++dx) {
        if (dx * dx + dy * dy > rad2) {
          continue;  // circular footprint
        }
        const int cx = static_cast<int>(mx) + dx;
        const int cy = static_cast<int>(my) + dy;
        if (cx < 0 || cy < 0 || cx >= static_cast<int>(size_x_) ||
            cy >= static_cast<int>(size_y_)) {
          continue;
        }
        const unsigned int idx = costmap_->getIndex(cx, cy);
        if (map_[idx] == NO_INFORMATION) {
          map_[idx] = FREE_SPACE;
        }
      }
    }
  }

  // initialize flag arrays to keep track of visited and frontier cells
  std::vector<bool> frontier_flag(size_x_ * size_y_, false);
  std::vector<bool> visited_flag(size_x_ * size_y_, false);

  // Geodesic distance (in cells) from the start cell to every reachable free
  // cell, accumulated during the wavefront below. -1 == unreached. This lets us
  // rank frontiers by REAL path distance through free space instead of
  // straight-line distance, so a frontier behind a wall is correctly seen as far.
  std::vector<double> distance(size_x_ * size_y_, -1.0);

  // initialize breadth first search
  std::queue<unsigned int> bfs;

  // find closest clear cell to start search
  unsigned int clear, pos = costmap_->getIndex(mx, my);
  if (nearestCell(clear, pos, FREE_SPACE, *costmap_)) {
    bfs.push(clear);
  } else {
    bfs.push(pos);
    RCLCPP_WARN(logger_, "[FrontierSearch] Could not find nearby clear cell to start search");
  }
  visited_flag[bfs.front()] = true;
  distance[bfs.front()] = 0.0;

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // iterate over 4-connected neighbourhood
    for (unsigned nbr : nhood4(idx, *costmap_)) {
      // add to queue all free, unvisited cells, use descending search in case
      // initialized on non-free cell
      if (map_[nbr] <= map_[idx] && !visited_flag[nbr]) {
        visited_flag[nbr] = true;
        // FIFO uniform-cost BFS => first visit is the shortest 4-connected path.
        distance[nbr] = distance[idx] + 1.0;
        bfs.push(nbr);
        // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
        // neighbour)
      } else if (isNewFrontierCell(nbr, frontier_flag)) {
        frontier_flag[nbr] = true;
        Frontier new_frontier = buildNewFrontier(nbr, pos, frontier_flag);
        if (new_frontier.size * costmap_->getResolution() <
            min_frontier_size_) {
          continue;
        }
        // Override the straight-line min_distance from buildNewFrontier with the
        // geodesic path distance. Because the wavefront visits cells in
        // nondecreasing distance order, `idx` (the first free cell to touch this
        // frontier) is its closest reachable approach cell. Fall back to the
        // Euclidean value only if, defensively, idx was somehow unreached.
        if (distance[idx] >= 0.0) {
          new_frontier.min_distance = distance[idx] * costmap_->getResolution();
        }
        // Project the frontier's closest cell into clearance-safe free space so
        // the navigation goal is never sent onto/next to a wall (the raw SLAM
        // map carries no inflation layer to push the goal away by itself).
        new_frontier.goal_valid =
            projectToFree(new_frontier.middle, new_frontier.goal);
        if (!new_frontier.goal_valid) {
          // No reachable free point with enough clearance -> skip this frontier
          // rather than emit a goal Nav2 will silently reject.
          continue;
        }
        frontier_list.push_back(new_frontier);
      }
    }
  }

  // set costs of frontiers
  for (auto& frontier : frontier_list) {
    frontier.cost = frontierCost(frontier);
  }
  std::sort(
      frontier_list.begin(), frontier_list.end(),
      [](const Frontier& f1, const Frontier& f2) { return f1.cost < f2.cost; });

  return frontier_list;
}

Frontier FrontierSearch::buildNewFrontier(unsigned int initial_cell,
                                          unsigned int reference,
                                          std::vector<bool>& frontier_flag)
{
  // initialize frontier structure
  Frontier output;
  output.centroid.x = 0;
  output.centroid.y = 0;
  output.size = 1;
  output.min_distance = std::numeric_limits<double>::infinity();

  // record initial contact point for frontier
  unsigned int ix, iy;
  costmap_->indexToCells(initial_cell, ix, iy);
  costmap_->mapToWorld(ix, iy, output.initial.x, output.initial.y);

  // push initial gridcell onto queue
  std::queue<unsigned int> bfs;
  bfs.push(initial_cell);

  // cache reference position in world coords
  unsigned int rx, ry;
  double reference_x, reference_y;
  costmap_->indexToCells(reference, rx, ry);
  costmap_->mapToWorld(rx, ry, reference_x, reference_y);

  while (!bfs.empty()) {
    unsigned int idx = bfs.front();
    bfs.pop();

    // try adding cells in 8-connected neighborhood to frontier
    for (unsigned int nbr : nhood8(idx, *costmap_)) {
      // check if neighbour is a potential frontier cell
      if (isNewFrontierCell(nbr, frontier_flag)) {
        // mark cell as frontier
        frontier_flag[nbr] = true;
        unsigned int mx, my;
        double wx, wy;
        costmap_->indexToCells(nbr, mx, my);
        costmap_->mapToWorld(mx, my, wx, wy);

        geometry_msgs::msg::Point point;
        point.x = wx;
        point.y = wy;
        output.points.push_back(point);

        // update frontier size
        output.size++;

        // update centroid of frontier
        output.centroid.x += wx;
        output.centroid.y += wy;

        // determine frontier's distance from robot, going by closest gridcell
        // to robot
        double distance = sqrt(pow((double(reference_x) - double(wx)), 2.0) +
                               pow((double(reference_y) - double(wy)), 2.0));
        if (distance < output.min_distance) {
          output.min_distance = distance;
          output.middle.x = wx;
          output.middle.y = wy;
        }

        // add to queue for breadth first search
        bfs.push(nbr);
      }
    }
  }

  // average out frontier centroid
  output.centroid.x /= output.size;
  output.centroid.y /= output.size;
  return output;
}

bool FrontierSearch::isNewFrontierCell(unsigned int idx,
                                       const std::vector<bool>& frontier_flag)
{
  // check that cell is unknown and not already marked as frontier
  if (map_[idx] != NO_INFORMATION || frontier_flag[idx]) {
    return false;
  }

  // frontier cells should have at least one cell in 4-connected neighbourhood
  // that is free
  for (unsigned int nbr : nhood4(idx, *costmap_)) {
    if (map_[nbr] == FREE_SPACE) {
      return true;
    }
  }

  return false;
}

double FrontierSearch::frontierCost(const Frontier& frontier)
{
  // Distance-dominant cost: proximity (in meters) drives selection. The size
  // bonus is CAPPED (frontier_size_cap_ cells) so a huge frontier can never
  // swamp the distance term - it can only break near-ties. Lower cost == more
  // attractive.
  const double effective_size =
      std::min(static_cast<double>(frontier.size), frontier_size_cap_);
  return (potential_scale_ * frontier.min_distance) -
         (gain_scale_ * effective_size * costmap_->getResolution());
}

double FrontierSearch::obstacleClearance(unsigned int mx, unsigned int my)
{
  // Expanding-ring scan around (mx,my) for the nearest LETHAL_OBSTACLE cell.
  // We only care whether clearance reaches min_obstacle_clearance_, so the scan
  // stops as soon as that radius is exceeded (returns the cap).
  const double resolution = costmap_->getResolution();
  const int max_radius_cells =
      static_cast<int>(std::ceil(min_obstacle_clearance_ / resolution));

  double nearest = min_obstacle_clearance_;  // cap: "safe" if nothing found
  for (int r = 1; r <= max_radius_cells; ++r) {
    bool found = false;
    // Scan the square ring at Chebyshev radius r.
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (std::max(std::abs(dx), std::abs(dy)) != r) {
          continue;  // only the outer ring
        }
        int cx = static_cast<int>(mx) + dx;
        int cy = static_cast<int>(my) + dy;
        if (cx < 0 || cy < 0 || cx >= static_cast<int>(size_x_) ||
            cy >= static_cast<int>(size_y_)) {
          continue;
        }
        unsigned int idx = costmap_->getIndex(cx, cy);
        if (map_[idx] == LETHAL_OBSTACLE) {
          double dist = std::hypot(dx * resolution, dy * resolution);
          if (dist < nearest) {
            nearest = dist;
            found = true;
          }
        }
      }
    }
    // Anything closer than the current ring's inner edge is already captured; a
    // lethal cell on ring r means we can stop (no nearer cell can appear later).
    if (found) {
      break;
    }
  }
  return nearest;
}

bool FrontierSearch::projectToFree(const geometry_msgs::msg::Point& seed,
                                   geometry_msgs::msg::Point& result)
{
  unsigned int smx, smy;
  if (!costmap_->worldToMap(seed.x, seed.y, smx, smy)) {
    return false;
  }
  const double resolution = costmap_->getResolution();
  const int max_ring = static_cast<int>(
      std::ceil(std::max(0.0, clearance_search_radius_) / resolution));
  // Number of angular samples per ring (denser on outer rings is unnecessary;
  // 16 is enough to find an opening in typical clutter).
  constexpr int kAngles = 16;

  // Ring 0 = the seed cell itself, then expand outward and keep the first
  // FREE_SPACE cell that has enough clearance from lethal obstacles.
  for (int ring = 0; ring <= max_ring; ++ring) {
    const double radius = ring * resolution;
    const int samples = (ring == 0) ? 1 : kAngles;
    for (int k = 0; k < samples; ++k) {
      const double theta = (2.0 * M_PI * k) / kAngles;
      const double wx = seed.x + radius * std::cos(theta);
      const double wy = seed.y + radius * std::sin(theta);
      unsigned int cmx, cmy;
      if (!costmap_->worldToMap(wx, wy, cmx, cmy)) {
        continue;
      }
      unsigned int idx = costmap_->getIndex(cmx, cmy);
      if (map_[idx] != FREE_SPACE) {
        continue;
      }
      if (obstacleClearance(cmx, cmy) >= min_obstacle_clearance_) {
        costmap_->mapToWorld(cmx, cmy, result.x, result.y);
        result.z = 0.0;
        return true;
      }
    }
  }
  return false;
}
}  // namespace frontier_exploration