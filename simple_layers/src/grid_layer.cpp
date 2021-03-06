#include<simple_layers/grid_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::GridLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

float old_x_, old_y_;

namespace simple_layer_namespace
{

GridLayer::GridLayer() {}

void GridLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  default_value_ = NO_INFORMATION;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &GridLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void GridLayer::matchSize()
{
  Costmap2D* master = layered_costmap_->getCostmap();
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
            master->getOriginX(), master->getOriginY());
}


void GridLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void GridLayer::updateBounds(costmap_2d::Costmap2D& master_grid, double robot_x, double robot_y, double robot_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;
  
  float old_x , old_y;

  tf::TransformListener listener;
  tf::StampedTransform transform;

  try
  {
     ros::Time now = ros::Time(0);
     listener.waitForTransform("/odom", "/box",
                        now, ros::Duration(3.0));
     listener.lookupTransform("/odom", "/box",
                        now, transform);
     old_x = transform.getOrigin().x();
     old_y = transform.getOrigin().y();
     old_x_ = old_x;
     old_y_ = old_y;
  }
  catch (tf::TransformException ex)
  {
     ROS_ERROR("%s",ex.what());
     old_x = old_x_;
     old_y = old_y_;
     ros::Duration(0.01).sleep();
  }
 

  double mark_x = old_x, mark_y = old_y;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x, mark_y, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }
  
  *min_x = std::min(*min_x, mark_x);
  *min_y = std::min(*min_y, mark_y);
  *max_x = std::max(*max_x, mark_x);
  *max_y = std::max(*max_y, mark_y);
}

void GridLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;

  for (int j = min_j; j < max_j; j++)
  {
    for (int i = min_i; i < max_i; i++)
    {
      int index = getIndex(i, j);
      if (costmap_[index] == NO_INFORMATION)
        continue;
      master_grid.setCost(i, j, costmap_[index]); 
    }
  }
}

} // end namespace
