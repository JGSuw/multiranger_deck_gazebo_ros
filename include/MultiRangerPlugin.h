/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
/*
 * Desc: Plugin for a model of the Bitcraze MultiRanger deck for Crazyflie 2.0
 * Author: Joseph Sullivan
 */

#ifndef _GAZEBO_MULTIRANGER_PLUGIN_H_
#define _GAZEBO_MULTIRANGER_PLUGIN_H_
#include <vector>
#include <string>
#include "gazebo/common/Plugin.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/RaySensor.hh"
#include "gazebo/util/system.hh"
#include "ros/ros.h"

namespace gazebo
{
  /// \brief A Ray Sensor Plugin
  class MultiRangerPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: MultiRangerPlugin()
    {
    }

    /// \brief Destructor
    public: virtual ~MultiRangerPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief
    public: void OnUpdate();

    /// \brief Handle new laser scans
    public: virtual void LaserScanEvent(int);

    /// \brief Pointer to parent
    protected: physics::WorldPtr world;

    /* ROS related members */
    private: ros::NodeHandle* nh;

    private: std::string _namespace;

    private: std::string topic;

    private: std::string frame_id;


    /// \brief The parent model

    private: enum {
      front,
      left,
      back,
      right,
      top
    };

    private: physics::ModelPtr model;

    private:
      ros::Publisher front_sensor_publisher;
      ros::Publisher left_sensor_publisher;
      ros::Publisher back_sensor_publisher;
      ros::Publisher right_sensor_publisher;
      ros::Publisher top_sensor_publisher;

    private:
      physics::LinkPtr front_link;
      physics::LinkPtr left_link;
      physics::LinkPtr back_link;
      physics::LinkPtr right_link;
      physics::LinkPtr top_link;

    /// \brief Instance of the SensorManager
    private: sensors::SensorManager* sensor_manager;

    private:
      event::ConnectionPtr front_sensor_connection;
      event::ConnectionPtr left_sensor_connection;
      event::ConnectionPtr back_sensor_connection;
      event::ConnectionPtr right_sensor_connection;
      event::ConnectionPtr top_sensor_connection;

    /// \brief Pointer to sensors
    private:
      sensors::RaySensorPtr front_sensor;
      sensors::RaySensorPtr left_sensor;
      sensors::RaySensorPtr back_sensor;
      sensors::RaySensorPtr right_sensor;
      sensors::RaySensorPtr top_sensor;
  };
  GZ_REGISTER_MODEL_PLUGIN(MultiRangerPlugin)
}
#endif
