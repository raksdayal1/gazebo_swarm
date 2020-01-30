#ifndef AEROFORCES_HH
#define AEROFORCES_HH

#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

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
class AeroForce : public ModelPlugin
{
public:
    /// \brief Constructor
    AeroForce();

    // Document Inherited
    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    // Document Inherited
    virtual void Init();

private:
    /// \brief Callback for World Update events.
    virtual void OnUpdate();

    /// \brief Function callback to read messages
    void ControlSurfaceMsg(const geometry_msgs::Vector3ConstPtr &msg);

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

    /// \brief SDF pointer for this plugin;
    sdf::ElementPtr sdf;

    /// \brief Create a ros node ptr
    ros::NodeHandlePtr Aerodynamics_node;

    /// \brief Declare a ros subscriber
    ros::Subscriber ControlSub;

    /// \brief Frame Conversion from Gazebo's XYZ axis to North East Down frame (i.e. Aerospace convention frame)
    ignition::math::Pose3d modelXYZToAirplaneXForwardZDown;

    /// \brief Frame conversion from the modelXYZ frame to Aerospace body frame (i.e. XForwardZDown frame)
    ignition::math::Pose3d gazeboXYZToNED;

    // Create polynomials to fit the data
    vec Poly_Lift, Poly_Drag, Poly_Moment;

    /// Wing Params
    // Surface Area and chord
    double WingSpan, ChordLength;

    /// Aerodynamic stability and control coefficients
    // Lift, Drag and Moment (Longitudinal)
    double CL, CD, Cm;
    double CL_q, CD_q, Cm_q;
    double CL_de, CD_de, Cm_de;

    // Side force, Roll and Yaw moments (Lateral)
    double Cy_0, Cl_0, Cn_0;
    double Cy_beta, Cl_beta, Cn_beta;
    double Cy_p, Cl_p, Cn_p;
    double Cy_r, Cl_r, Cn_r;
    double Cy_da, Cl_da, Cn_da;
    double Cy_dr, Cl_dr, Cn_dr;

    /// \brief Flags to check if control surfaces exists
    bool ELEVATOR_FLAG, AILERON_FLAG, RUDDER_FLAG;

    /// \brief Aileron Elevator and Rudder JointPtr
    physics::JointPtr AileronLeft, AileronRight, Elevator, Rudder;

    /// \brief file name that contains the LIft Drag data
    std::string LiftDragCoeffFileName;

    /// \brief Total mass of the aircraft
    double mass;

};

#endif // AEROFORCES_HH
