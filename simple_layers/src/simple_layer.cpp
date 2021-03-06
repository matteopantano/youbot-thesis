#include<simple_layers/simple_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;
using costmap_2d::INSCRIBED_INFLATED_OBSTACLE;


float old_x_ = 3.0, old_y_ = 3.0;

namespace simple_layer_namespace
{

SimpleLayer::SimpleLayer() {}

void SimpleLayer::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  matchSize();

  dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
      &SimpleLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}


void SimpleLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
{
  enabled_ = config.enabled;
}

void SimpleLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
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
                        now, ros::Duration(1.0));
     listener.lookupTransform("/odom", "/box",
                        now, transform);
     old_x = transform.getOrigin().x();
     old_y = transform.getOrigin().y();
     old_x_ = old_x;
     old_y_ = old_y;
  }
  catch (tf::TransformException ex)
  {
     //ROS_ERROR("%s",ex.what());
     old_x = old_x_;
     old_y = old_y_;
     //ros::Duration(0.01).sleep();
  }
 
  mark_x_ = old_x;
  mark_y_ = old_y;
	  
  printf("\nx: %2f",mark_x_);
  printf("\ny: %2f\n",mark_y_);

  //mark_x_ = robot_x + cos(robot_yaw);
  //mark_y_ = robot_y + sin(robot_yaw); 

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
  if (!enabled_)
    return;
  unsigned int mx;
  unsigned int my;
  if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
    master_grid.setCost(mx, my, LETHAL_OBSTACLE);
  }

  float j = 0.0;

  // First & Second ring of light blue
  j = 0.0;	
  float l = 0.0;
  int n = 0;

  for (int k=0; k < 2; k++)
  {
	  l = l + 0.05;
	  n = 3 + k * 2;
	  for (int i=0; i < n; i++)
	  { 		
	  	if(master_grid.worldToMap(mark_x_ + l, mark_y_ + l + j, mx, my)){
		  master_grid.setCost(mx, my, INSCRIBED_INFLATED_OBSTACLE);
		}
		if(master_grid.worldToMap(mark_x_ - l, mark_y_ + l + j, mx, my)){
		  master_grid.setCost(mx, my, INSCRIBED_INFLATED_OBSTACLE);
		}
		if(master_grid.worldToMap(mark_x_ + l + j, mark_y_ + l, mx, my)){
		  master_grid.setCost(mx, my, INSCRIBED_INFLATED_OBSTACLE);
		}
		if(master_grid.worldToMap(mark_x_ + l + j, mark_y_ - l, mx, my)){
		  master_grid.setCost(mx, my, INSCRIBED_INFLATED_OBSTACLE);
		}
		j = j - 0.05;
	  }
	  j = 0.0;
  }

  j = 0.0;	
  l = 0.10;
  n = 0;
  double factor = 1.0;

  for (int k=0; k < 3; k++)
  {
	  l = l + 0.05;
	  n = 7 + k * 2;
	  for (int i=0; i < n; i++)
	  { 		
	  	if(master_grid.worldToMap(mark_x_ + l, mark_y_ + l + j, mx, my)){
		  master_grid.setCost(mx, my, (INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
		}
		if(master_grid.worldToMap(mark_x_ - l, mark_y_ + l + j, mx, my)){
		  master_grid.setCost(mx, my, (INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
		}
		if(master_grid.worldToMap(mark_x_ + l + j, mark_y_ + l, mx, my)){
		  master_grid.setCost(mx, my, (INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
		}
		if(master_grid.worldToMap(mark_x_ + l + j, mark_y_ - l, mx, my)){
		  master_grid.setCost(mx, my, (INSCRIBED_INFLATED_OBSTACLE - 1) * factor);
		}
		j = j - 0.05;
	  }
	  factor = factor - 0.49;
	  j = 0.0;
  }

  

}

} // end namespace
