/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <explore/explore.h>

#include <thread>

inline static bool operator==(const geometry_msgs::Point& one,
                              const geometry_msgs::Point& two)
{
  double dx = one.x - two.x;
  double dy = one.y - two.y;
  double dist = sqrt(dx * dx + dy * dy);
  return dist < 0.01;
}

namespace explore
{
Explore::Explore()
  : private_nh_("~")
  , tf_listener_(ros::Duration(10.0))
  , costmap_client_(private_nh_, relative_nh_, &tf_listener_)
  , move_base_client_("move_base")
  , prev_distance_(0)
  , last_markers_count_(0)
{
  prev_idx = -10;//luke wrote:
  //target_cell = 7198153; // luke wrote
  prev_costmap_ = NULL; //liao wrote

  double timeout;
  double min_frontier_size;
  private_nh_.param("planner_frequency", planner_frequency_, 1.0);
  private_nh_.param("progress_timeout", timeout, 30.0);
  progress_timeout_ = ros::Duration(timeout);
  private_nh_.param("visualize", visualize_, false);
  private_nh_.param("potential_scale", potential_scale_, 1e-3);
  private_nh_.param("orientation_scale", orientation_scale_, 0.0);
  private_nh_.param("gain_scale", gain_scale_, 1.0);
  private_nh_.param("min_frontier_size", min_frontier_size, 0.5);

  search_ = frontier_exploration::FrontierSearch(costmap_client_.getCostmap(),
                                                 potential_scale_, gain_scale_,
                                                 min_frontier_size);

  if (visualize_) {
    marker_array_publisher_ =
        private_nh_.advertise<visualization_msgs::MarkerArray>("frontiers", 10);
  }

  ROS_INFO("Waiting to connect to move_base server");
  move_base_client_.waitForServer();
  ROS_INFO("Connected to move_base server");

  exploring_timer_ =
      relative_nh_.createTimer(ros::Duration(1. / planner_frequency_),
                               [this](const ros::TimerEvent&) { makePlan(); });
}

Explore::~Explore()
{
  stop();
}

void Explore::visualizeFrontiers(
    const std::vector<frontier_exploration::Frontier>& frontiers)
{
  std_msgs::ColorRGBA blue;
  blue.r = 0;
  blue.g = 0;
  blue.b = 1.0;
  blue.a = 1.0;
  std_msgs::ColorRGBA red;
  red.r = 1.0;
  red.g = 0;
  red.b = 0;
  red.a = 1.0;
  std_msgs::ColorRGBA green;
  green.r = 0;
  green.g = 1.0;
  green.b = 0;
  green.a = 1.0;

  ROS_DEBUG("visualising %lu frontiers", frontiers.size());
  visualization_msgs::MarkerArray markers_msg;
  std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
  visualization_msgs::Marker m;

  m.header.frame_id = costmap_client_.getGlobalFrameID();
  m.header.stamp = ros::Time::now();
  m.ns = "frontiers";
  m.scale.x = 1.0;
  m.scale.y = 1.0;
  m.scale.z = 1.0;
  m.color.r = 0;
  m.color.g = 0;
  m.color.b = 255;
  m.color.a = 255;
  // lives forever
  m.lifetime = ros::Duration(0);
  m.frame_locked = true;

  // weighted frontiers are always sorted
  double min_cost = frontiers.empty() ? 0. : frontiers.front().cost;

  m.action = visualization_msgs::Marker::ADD;
  size_t id = 0;
  for (auto& frontier : frontiers) {
    m.type = visualization_msgs::Marker::POINTS;
    m.id = int(id);
    m.pose.position = {};
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.points = frontier.points;
    if (goalOnBlacklist(frontier.centroid)) {
      m.color = red;
    } else {
      m.color = blue;
    }
    markers.push_back(m);
    ++id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.id = int(id);
    m.pose.position = frontier.initial;
    // scale frontier according to its cost (costier frontiers will be smaller)
    double scale = std::min(std::abs(min_cost * 0.4 / frontier.cost), 0.5);
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.points = {};
    m.color = green;
    markers.push_back(m);
    ++id;
  }
  size_t current_markers_count = markers.size();

  // delete previous markers, which are now unused
  m.action = visualization_msgs::Marker::DELETE;
  for (; id < last_markers_count_; ++id) {
    m.id = int(id);
    markers.push_back(m);
  }

  last_markers_count_ = current_markers_count;
  marker_array_publisher_.publish(markers_msg);
}

void Explore::makePlan()
{
  //liao wrote================================
  // if (prev_costmap_ == NULL) {
  //   // prev_costmap_ = search_.getCostMap();
  //   costmap_2d::Costmap2D pv_ctmap = costmap_client_.getCostmapObj();
  //   prev_costmap_ = new costmap_2d::Costmap2D(pv_ctmap);
  // }
  // costmap_2d::Costmap2D* new_costmap_ = search_.getCostMap();
  // size_t costmap_size = new_costmap_->getSizeInCellsX() * new_costmap_->getSizeInCellsY();
  // std::vector<int> new_cells_ids;
  // int num_diff = 0;
  // for (size_t i = 0; i < costmap_size; ++i) {
  //   unsigned char* prev_costmap_data = prev_costmap_->getCharMap();
  //   unsigned char* cur_costmap_data = new_costmap_->getCharMap();
  //   if (i == 7162149){
  //     ROS_DEBUG("obj value = %d", cur_costmap_data[i]);
  //   }
  //   if (prev_costmap_data[i] != cur_costmap_data[i]) {
  //     // new_cells_ids.push_back(static_cast<int>(i));
  //     num_diff += 1;
  //     // ROS_DEBUG_STREAM("value diff");
  //     if (cur_costmap_data[i] > 50 && cur_costmap_data[i] < 70) {
  //       ROS_DEBUG_STREAM("Object is found");
  //       stop();
  //       return;
  //     }
  //   }
  // }
  // ROS_DEBUG("num_diff = %d", num_diff);
  // // prev_costmap_ = search_.getCostMap();
  // delete prev_costmap_;
  // costmap_2d::Costmap2D pv_ctmap = costmap_client_.getCostmapObj();
  // prev_costmap_ = new costmap_2d::Costmap2D(pv_ctmap);

  costmap_2d::Costmap2D* new_costmap_ = search_.getCostMap();
  size_t costmap_size = new_costmap_->getSizeInCellsX() * new_costmap_->getSizeInCellsY();
  unsigned char* costmap_data = new_costmap_->getCharMap();
  // for (size_t i =0; i < costmap_size; ++i){
  //   if (static_cast<int>(costmap_data[i]) ==100){
  //     ROS_DEBUG_STREAM("Object is found");
  //     stop();
  //     return;
  //   }
  // }

  //======================================================

  //luke wrote =================================
  // costmap_2d::Costmap2D* new_costmap_ = search_.getCostMap();
  // unsigned char* costmap_data = new_costmap_->getCharMap();
  //size_t costmap_size = new_costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
  //ROS_DEBUG("full map update, %lu values", costmap_size);
  // costmap_size = 10;

  std::string debug_str;
  for (size_t i = 0; i < costmap_size; ++i) {
    //unsigned char cell_cost = static_cast<unsigned char>(msg->data[i]);
    // costmap_data[i] = cost_translation_table__[costmap_data[i]];
    if (static_cast<int>(costmap_data[i]) != 255 && static_cast<int>(costmap_data[i]) != 0 ){
      debug_str.append(std::to_string(static_cast<int>(costmap_data[i])));
      debug_str.append(" ");
    }
  }
  // ROS_DEBUG_STREAM(debug_str);
  //===============================================

  // find frontiers
  auto pose = costmap_client_.getRobotPose();
  // get frontiers sorted according to cost
  auto frontiers = search_.searchFrom(pose.position);
  ROS_DEBUG("found %lu frontiers", frontiers.size());
  for (size_t i = 0; i < frontiers.size(); ++i) {
    ROS_DEBUG("frontier %zd cost: %f", i, frontiers[i].cost);
  }

  if (frontiers.empty()) {
    stop();
    return;
  }

  // publish frontiers as visualization markers
  if (visualize_) {
    visualizeFrontiers(frontiers);
  }

  // find non blacklisted frontier
  auto frontier =
      std::find_if_not(frontiers.begin(), frontiers.end(),
                       [this](const frontier_exploration::Frontier& f) {
                         return goalOnBlacklist(f.centroid);
                       });
  if (frontier == frontiers.end()) {
    stop();
    return;
  }
  //luke wrote================================================
  geometry_msgs::Point target_position = frontier->centroid;
  // ROS_DEBUG("original frontier->idx = %d", frontier->idx);
  unsigned int ix, iy;
  search_.getCostMap()->indexToCells(frontier->idx, ix, iy);
  //ROS_DEBUG("frontier->idx = (%d, %d)", ix, iy);
  //unsigned int mx, my;
  unsigned int clear11, pos11 = new_costmap_->getIndex(ix, iy);
  // ROS_DEBUG("converted frontier->idx = %d, %d", clear11, pos11);
  //=============================================================

  // time out if we are not making any progress
  bool same_goal = prev_goal_ == target_position;
  prev_goal_ = target_position;
  if (!same_goal || prev_distance_ > frontier->min_distance) {
    // we have different goal or we made some progress
    last_progress_ = ros::Time::now();
    prev_distance_ = frontier->min_distance;
  }
  // black list if we've made no progress for a long time
  if (ros::Time::now() - last_progress_ > progress_timeout_) {
    frontier_blacklist_.push_back(target_position);
    ROS_DEBUG("Adding current goal to black list");
    makePlan();
    return;
  }

  if (prev_idx == -10) {
    prev_idx = frontier->idx;
   }// luke wrote
  // we don't need to do anything if we still pursuing the same goal
  unsigned int target_ix, target_iy;
  search_.getCostMap()->indexToCells(prev_idx, target_ix, target_iy);
  if (same_goal) {
    return;
  } 
  //luke wrote=============
  else {
    if (target_ix< 2174 && target_ix >2124 && target_iy <1819 && target_iy>1769) {
      stop();
      return;
    }else {
      prev_idx = frontier->idx;
    }
  }

  // else{
  //   // if the cell in the scanned area
  //   std::string target_area;
  //   int sum=0;
  //   for(int target_ix = 2144; target_ix < 2154; ++target_ix)
  //   {
  //     for(int target_iy =1789; target_iy < 1799; ++target_iy)
  //     {
  //       unsigned int clear11, pos11 = new_costmap_->getIndex(target_ix, target_iy);
  //       // target_area.append(std::to_string(static_cast<int>(costmap_data[pos11])));
  //       // target_area.append(" ");
  //       target_area.append(std::to_string(static_cast<int>(pos11)));
  //       target_area.append(" ");
  //       sum=sum+static_cast<int>(costmap_data[pos11]);
  //     } 
  //   }
  //   ROS_DEBUG_STREAM(target_area); 
  //   if (sum==0){
  //     stop();
  //     return;
  //   }

  //if robot position is on that cell
    // if (target_robotx< 2174 && target_robotx >2124 && target_roboty <1819 && target_roboty>1769) {
    //   stop();
    //   return;
    // }else {
    //   prev_idx = frontier->idx;
    // }

  // }
  //===========================

  // send goal to move_base if we have something new to pursue
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.pose.position = target_position;
  goal.target_pose.pose.orientation.w = 1.;
  goal.target_pose.header.frame_id = costmap_client_.getGlobalFrameID();
  goal.target_pose.header.stamp = ros::Time::now();
  move_base_client_.sendGoal(
      goal, [this, target_position](
                const actionlib::SimpleClientGoalState& status,
                const move_base_msgs::MoveBaseResultConstPtr& result) {
        reachedGoal(status, result, target_position);
      });
}

bool Explore::goalOnBlacklist(const geometry_msgs::Point& goal)
{
  constexpr static size_t tolerace = 5;
  costmap_2d::Costmap2D* costmap2d = costmap_client_.getCostmap();

  // check if a goal is on the blacklist for goals that we're pursuing
  for (auto& frontier_goal : frontier_blacklist_) {
    double x_diff = fabs(goal.x - frontier_goal.x);
    double y_diff = fabs(goal.y - frontier_goal.y);

    if (x_diff < tolerace * costmap2d->getResolution() &&
        y_diff < tolerace * costmap2d->getResolution())
      return true;
  }
  return false;
}

void Explore::reachedGoal(const actionlib::SimpleClientGoalState& status,
                          const move_base_msgs::MoveBaseResultConstPtr&,
                          const geometry_msgs::Point& frontier_goal)
{
  ROS_DEBUG("Reached goal with status: %s", status.toString().c_str());
  if (status == actionlib::SimpleClientGoalState::ABORTED) {
    frontier_blacklist_.push_back(frontier_goal);
    ROS_DEBUG("Adding current goal to black list");
  }

  // find new goal immediatelly regardless of planning frequency.
  // execute via timer to prevent dead lock in move_base_client (this is
  // callback for sendGoal, which is called in makePlan). the timer must live
  // until callback is executed.
  oneshot_ = relative_nh_.createTimer(
      ros::Duration(0, 0), [this](const ros::TimerEvent&) { makePlan(); },
      true);
}

void Explore::start()
{
  exploring_timer_.start();
}

void Explore::stop()
{
  move_base_client_.cancelAllGoals();
  exploring_timer_.stop();
  ROS_INFO("Exploration stopped.");
}

}  // namespace explore

int main(int argc, char** argv)
{
  ros::init(argc, argv, "explore");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  explore::Explore explore;
  ros::spin();

  return 0;
}
