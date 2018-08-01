
/*
 * Desc: Plugin for a model of the Bitcraze MultiRanger deck for Crazyflie 2.0
 * Author: Joseph Sullivan
 */

#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/sensors.hh"
#include "MultiRangerPlugin.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <limits>
#include <vector>
#include <map>

using namespace gazebo;

/////////////////////////////////////////////////
MultiRangerPlugin::~MultiRangerPlugin()
{
  front_link.reset();
  front_sensor.reset();
  left_link.reset();
  left_sensor.reset();
  back_link.reset();
  back_sensor.reset();
  right_link.reset();
  right_sensor.reset();
  top_link.reset();
  top_sensor.reset();
  this->world.reset();
  this->nh->shutdown();
}

/////////////////////////////////////////////////
void MultiRangerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get then name of the parent sensor
  this->model = _parent;

  if (!this->model)
    gzthrow("MultiRangerPlugin requires a model as its parent");

  this->world = this->model->GetWorld();
  this->sensor_manager = sensors::SensorManager::Instance();

  /* Sensor Setup */
  /* Front sensor setup */
  front_link = this->model->GetLink("front_sensor");
  front_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor_manager->GetSensor(front_link->GetSensorName(0)));
  front_sensor_connection =
    front_sensor->LaserShape()->ConnectNewLaserScans(boost::bind(&MultiRangerPlugin::LaserScanEvent, this, front));

  /* Left sensor setup */
  left_link = this->model->GetLink("left_sensor");
  left_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor_manager->GetSensor(left_link->GetSensorName(0)));
  left_sensor_connection =
    left_sensor->LaserShape()->ConnectNewLaserScans(boost::bind(&MultiRangerPlugin::LaserScanEvent, this, left));

  /* Back sensor setup */
  back_link = this->model->GetLink("back_sensor");
  back_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor_manager->GetSensor(back_link->GetSensorName(0)));
  back_sensor_connection =
    back_sensor->LaserShape()->ConnectNewLaserScans(boost::bind(&MultiRangerPlugin::LaserScanEvent, this, back));

  /* Right sensor setup */
  right_link = this->model->GetLink("right_sensor");
  right_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor_manager->GetSensor(right_link->GetSensorName(0)));
  right_sensor_connection =
    right_sensor->LaserShape()->ConnectNewLaserScans(boost::bind(&MultiRangerPlugin::LaserScanEvent, this, right));

  /* Top sensor setup */
  top_link = this->model->GetLink("top_sensor");
  top_sensor = std::dynamic_pointer_cast<sensors::RaySensor>(this->sensor_manager->GetSensor(top_link->GetSensorName(0)));
  top_sensor_connection =
    top_sensor->LaserShape()->ConnectNewLaserScans(boost::bind(&MultiRangerPlugin::LaserScanEvent, this, top));


  /* Ros Setup */
  this->_namespace.clear();
  this->topic = "multiranger";
  this->frame_id = "/multiranger_link";

  /* Thanks to the Hector gazebo package for the following ros related code */
  if (_sdf->HasElement("robotNamespace"))
  _namespace = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();

  if (_sdf->HasElement("frameId"))
    frame_id = _sdf->GetElement("frameId")->GetValue()->GetAsString();

  if (_sdf->HasElement("topicName"))
    topic = _sdf->GetElement("topicName")->GetValue()->GetAsString();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->nh = new ros::NodeHandle("multiranger");

  /* Now adding the publishers */
  front_sensor_publisher = nh->advertise<std_msgs::Float64>("front", 1000);
  left_sensor_publisher = nh->advertise<std_msgs::Float64>("left", 1000);
  back_sensor_publisher = nh->advertise<std_msgs::Float64>("back", 1000);
  right_sensor_publisher = nh->advertise<std_msgs::Float64>("right", 1000);
  top_sensor_publisher = nh->advertise<std_msgs::Float64>("top", 1000);
}

/////////////////////////////////////////////////
void MultiRangerPlugin::LaserScanEvent(int sensor_position)
{
  sensors::RaySensorPtr sensor;
  ros::Publisher* pub;
  switch(sensor_position) {

    case front:
      sensor = front_sensor;
      pub = &front_sensor_publisher;
      break;

    case left:
      sensor = left_sensor;
      pub = &left_sensor_publisher;
      break;

    case back:
      sensor = back_sensor;
      pub = &back_sensor_publisher;
      break;

    case right:
      sensor = right_sensor;
      pub = &right_sensor_publisher;
      break;

    case top:
      sensor = top_sensor;
      pub = &top_sensor_publisher;

    default:
      return;
  }
  double range = std::numeric_limits<double>::infinity();

  std::vector<double> ranges;
  sensor->Ranges(ranges);

  for(int i = 0; i < ranges.size(); i++) {
    if (ranges[i] < range) range = ranges[i];
  }

  if (range > sensor->RangeMax()) range = sensor->RangeMax();
  else if (range < sensor->RangeMin()) range = sensor->RangeMin();

  // Publish the range
  std_msgs::Float64 msg;
  msg.data = range;
  pub->publish(msg);
}
