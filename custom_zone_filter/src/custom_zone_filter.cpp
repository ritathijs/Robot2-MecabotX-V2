#include "custom_zone_filter/custom_zone_filter.hpp"

#include "pluginlib/class_list_macros.hpp"

#include "nav2_costmap_2d/cost_values.hpp"


namespace custom_zone_filter
{


CustomZoneFilter::CustomZoneFilter()
:
toggle_enabled_(false)
{

}


CustomZoneFilter::~CustomZoneFilter()
{

}



void CustomZoneFilter::onInitialize()
{

  auto node = node_.lock();


  mask_sub_ =
    node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/zone_mask",
      10,
      std::bind(
        &CustomZoneFilter::maskCallback,
        this,
        std::placeholders::_1));



  toggle_service_ =
    node->create_service<std_srvs::srv::SetBool>(
      "/toggle_zone_filter",
      std::bind(
        &CustomZoneFilter::toggleCallback,
        this,
        std::placeholders::_1,
        std::placeholders::_2));


  RCLCPP_INFO(
    node->get_logger(),
    "Custom Zone Filter started");

}



void CustomZoneFilter::maskCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{

  mask_ = msg;

}



void CustomZoneFilter::toggleCallback(
  const std_srvs::srv::SetBool::Request::SharedPtr request,
  std_srvs::srv::SetBool::Response::SharedPtr response)
{

  toggle_enabled_ = request->data;

  response->success = true;

  response->message = "toggle changed";

}



void CustomZoneFilter::updateBounds(
  double,
  double,
  double,
  double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{

  if(mask_)
  {

    *min_x = -100;
    *min_y = -100;
    *max_x = 100;
    *max_y = 100;

  }

}



void CustomZoneFilter::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i,
  int min_j,
  int max_i,
  int max_j)
{


  if(!mask_)
    return;



  for(int y = min_j; y < max_j; y++)
  {

    for(int x = min_i; x < max_i; x++)
    {


      unsigned int index =
        y * mask_->info.width + x;



      if(index >= mask_->data.size())
        continue;



      int value = mask_->data[index];



      // zwart = obstakel
      if(value < 20)
      {

        master_grid.setCost(
          x,
          y,
          nav2_costmap_2d::LETHAL_OBSTACLE);

      }



      // grijs = afhankelijk van toggle
      else if(value > 80 && value < 180)
      {

        if(toggle_enabled_)
        {

          master_grid.setCost(
            x,
            y,
            nav2_costmap_2d::LETHAL_OBSTACLE);

        }

      }



      // wit = vrij
      else
      {

        master_grid.setCost(
          x,
          y,
          nav2_costmap_2d::FREE_SPACE);

      }


    }

  }


}



void CustomZoneFilter::reset()
{

  mask_.reset();

}



bool CustomZoneFilter::isClearable()
{

  return false;

}


}  // namespace custom_zone_filter



PLUGINLIB_EXPORT_CLASS(
  custom_zone_filter::CustomZoneFilter,
  nav2_costmap_2d::Layer
)
