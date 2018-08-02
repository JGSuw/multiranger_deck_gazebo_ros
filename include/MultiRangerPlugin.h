/*
Copyright (c) 2018, University of Washington
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the University of Washington nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL UNIVERSITY OF WASHINGTON BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

    private: enum {
      front,
      left,
      back,
      right,
      top
    };

    /// \brief The parent model
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
