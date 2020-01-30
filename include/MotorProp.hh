#ifndef MOTORPROP_HH
#define MOTORPROP_HH

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"

#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>

#include "armadillo"

using namespace std;
using namespace arma;
using namespace gazebo;


/// \brief A plugin that simulates lift and drag.
class MotorProp : public ModelPlugin
{
public:
    /// \brief Constructor.
    MotorProp();

    // Documentation Inherited.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    virtual void Init();

private:
    /// \brief Callback for World Update events.
    virtual void OnUpdate();

    /// \brief Function callback to read messages
    void MotorMsg(const std_msgs::Float64ConstPtr &msg);

    /// \brief Connection to World Update events.
    event::ConnectionPtr updateConnection;

    /// \brief Pointer to world.
    physics::WorldPtr world;

    /// \brief Pointer to physics engine.
    physics::PhysicsEnginePtr physics;

    /// \brief Pointer to model containing plugin.
    physics::ModelPtr model;

    /// \brief Name of model containing plugin.
    std::string modelName;

    /// \brief Pointer to link
    physics::LinkPtr link;

    /// \brief Name of the link
    std::string linkName;

    /// \brief Joint pointer
    physics::JointPtr joint;

    /// \brief SDF pointer for this plugin;
    sdf::ElementPtr sdf;

    /// \brief Declare a ros node
    ros::NodeHandlePtr MotorNode;

    /// \brief Declare a subscriber
    ros::Subscriber MotorSub;

    /// Frame Conversion variable
    ignition::math::Pose3d modelXYZToAirplaneXForwardZDown, gazeboXYZToNED;

    /// Motor params
    double PropDiameter, MotorKV, Ct, BatteryVoltage;

    /// PWM Input
    double ScaledControlSignal;
};


#endif // MOTORPROP_HH
