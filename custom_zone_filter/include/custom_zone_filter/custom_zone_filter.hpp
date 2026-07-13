#ifndef CUSTOM_ZONE_FILTER_HPP_
#define CUSTOM_ZONE_FILTER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include <nav_msgs/msg/occupancy_grid.hpp>


namespace custom_zone_filter
{

class CustomZoneFilter : public nav2_costmap_2d::Layer
{

public:

  CustomZoneFilter();

  virtual ~CustomZoneFilter();

  virtual void onInitialize() override;

  virtual void updateBounds(
      double robot_x,
      double robot_y,
      double robot_yaw,
      double * min_x,
      double * min_y,
      double * max_x,
      double * max_y) override;


  virtual void updateCosts(
      nav2_costmap_2d::Costmap2D & master_grid,
      int min_i,
      int min_j,
      int max_i,
      int max_j) override;
      
  virtual void reset() override;
  virtual bool isClearable() override;


private:

  void maskCallback(
      const nav_msgs::msg::OccupancyGrid::SharedPtr msg);


  void toggleCallback(
      const std_srvs::srv::SetBool::Request::SharedPtr request,
      std_srvs::srv::SetBool::Response::SharedPtr response);


  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_sub_;

  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_service_;


  nav_msgs::msg::OccupancyGrid::SharedPtr mask_;

  bool toggle_enabled_;

};

}

#endif
