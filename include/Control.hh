#ifndef CONTROL_HH
#define CONTROL_HH

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include "gazebo_swarm/State.h"
#include "gazebo_swarm/Control.h"

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

//typedef const boost::shared_ptr<const mycustom_msgs::msgs::ControlCmd> ControlCmdPtr;
//typedef const boost::shared_ptr<const mycustom_msgs::msgs::StateMsg> StateMsgPtr;

/// \brief A plugin that simulates lift and drag.
class Control : public ModelPlugin
{
public:
    /// \brief Constructor.
    Control();

    // Documentation Inherited.
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Documentation Inherited.
    virtual void Init();

private:
    /// \brief Callback for World Update events.
    virtual void OnUpdate();

    /// \brief Function callback to read messages
    void ControlMsg(const gazebo_swarm::ControlConstPtr &msg);

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

    /// \brief Frame Conversion from Gazebo's XYZ axis to North East Down frame (i.e. Aerospace convention frame)
    ignition::math::Pose3d modelXYZToAirplaneXForwardZDown;

    /// \brief Frame conversion from the modelXYZ frame to Aerospace body frame (i.e. XForwardZDown frame)
    ignition::math::Pose3d gazeboXYZToNED;

    /// \brief Declare a ros node
    ros::NodeHandlePtr ControlNode;

    /// \brief Declare a publisher
    // This publishes to the Aerodynamic control surfaces
    ros::Publisher ControlSystemPub;

    //This publishes to the Motor
    ros::Publisher MotorPub;

    // Publishes state information
    ros::Publisher StatePub;

    /// \brief Declare a subscriber
    // The subscribes to control data
    ros::Subscriber ControlSubscriber;

    bool AILERON_FLAG, ELEVATOR_FLAG, RUDDER_FLAG, THROTTLE_FLAG;

    double Aileron_data, Elevator_data, Rudder_data, Motor_data;

};

#endif // Control_HH
