#include "Control.hh"

#include <unistd.h>

using namespace gazebo;
using namespace std;
using namespace arma;
using namespace ignition::math;

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

GZ_REGISTER_MODEL_PLUGIN(Control)

Control::Control()
{
    this->AILERON_FLAG = false;
    this->ELEVATOR_FLAG = false;
    this->RUDDER_FLAG = false;
    this->THROTTLE_FLAG = false;

}

/////////////////////////////////////////////////
void Control::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "Control _model pointer is NULL");
    this->model = _model; // model pointer
    this->modelName = _model->GetName(); // model name

    GZ_ASSERT(_sdf, "Control _sdf pointer is NULL");
    this->sdf = _sdf; // sdf Element pointer

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "Control world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "Control physics pointer is NULL");

    /// \brief Get the name of the surface on which the aerodynamic forces act
    if (_sdf->HasElement("ActingSurface"))
    {
      sdf::ElementPtr elem = _sdf->GetElement("ActingSurface");

      this->linkName = elem->Get<std::string>();

      this->link = this->model->GetLink(this->linkName);
    }
    else
    {

        cout << "[Plugin Warn]: Acting surface not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;

    }

    /// \brief Initialize the transformation from modelXYZ to Body Frame
    if (_sdf->HasElement("modelXYZToAirplaneXForwardZDown"))
    {
      // get user transformation for model XYZ to bodyframe
      this->modelXYZToAirplaneXForwardZDown = sdf->GetElement("modelXYZToAirplaneXForwardZDown")->Get<Pose3d>();
    }
    else
    {
        cout << "[Plugin Warn]: modelXYZToAirplaneXForwardZDown not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;

        // Default transformation from modelXYZ to Airplane X forward Z down
        this->modelXYZToAirplaneXForwardZDown.Pos().X(0);
        this->modelXYZToAirplaneXForwardZDown.Pos().Y(0);
        this->modelXYZToAirplaneXForwardZDown.Pos().Z(0);
        this->modelXYZToAirplaneXForwardZDown.Rot().Euler(M_PI,0,0);

    }

    /// \brief transformation from gazebo XYZ frame to North East Down.
    /// For e.q. if gazebo XYZ is East North Up, the transform is pi radians around East axis, resulting in EN'D' frame and pi radians around the D' axis
    /// resulting in NED
    if (_sdf->HasElement("gazeboXYZToNED"))
    {
      // get user transformation for gazebo XYZ to NED
      this->gazeboXYZToNED = sdf->GetElement("gazeboXYZToNED")->Get<Pose3d>();
    }
    else
    {
        // Default transformation from gazebo XYZ frame to North East Down
        this->gazeboXYZToNED.Pos().X(0);
        this->gazeboXYZToNED.Pos().Y(0);
        this->gazeboXYZToNED.Pos().Z(0);
        this->gazeboXYZToNED.Rot().Euler(M_PI,0,M_PI/2.0);

    }

    // Create a name for the gazebo node
    std::string nodename = this->modelName + "_ControlSystem";

    // Create the gazebo node
    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodename);
    }

    ROS_INFO("ControlSystems plugin ready");
    this->ControlNode.reset(new ros::NodeHandle(nodename));

    // Publisher: Publish to the control suface
    this->ControlSystemPub = this->ControlNode->advertise<geometry_msgs::Vector3>("/Skyhunter_Plugin/ControlSurface", 1000);

    // Publisher: Publish to the Motor
    this->MotorPub = this->ControlNode->advertise<std_msgs::Float64>("/Skyhunter_Plugin/MotorPropulsion", 1000);

    //Publisher: Publish State information for feedback control
    this->StatePub = this->ControlNode->advertise<gazebo_swarm::State>("/Skyhunter_Plugin/State", 1000);

    // Subscriber: Subscribe to the control values
    std::string topicName = "/" + this->model->GetName() + "/Control";

    // Subscribe to the topic, and register a callback

    this->ControlSubscriber = this->ControlNode->subscribe(topicName, 1000, &Control::ControlMsg, this);

    // Intialize values for motor, aileron, elevator, rudder
    this->Motor_data = 0.0;
    this->Aileron_data = 0.0;
    this->Elevator_data = 0.0;
    this->Rudder_data = 0.0;
}

void Control::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Control::OnUpdate, this));
}

void Control::ControlMsg(const gazebo_swarm::ControlConstPtr &msg)
{

    /*
    cout << "Motor = " << msg->motor() << endl;
    cout << "Aileron = " << msg->aileron()<< endl;
    cout << "Elevator = "<< msg->elevator() << endl;
    cout << "Rudder = " << msg->rudder() << endl;
    cout << "=====================" <<endl;
    */

    this->Motor_data = msg->throttle;
    this->Aileron_data = msg->roll;
    this->Elevator_data = msg->pitch;
    this->Rudder_data = msg->yaw;

}

void Control::OnUpdate()
{

    gazebo_swarm::State state;

    std_msgs::Float64 msg_motor;
    geometry_msgs::Vector3 msg_aero;

    // This below gives the pose of the model in gazebo's XYZ frame
    const Pose3d gazeboXYZToModelXForwardZDown = this->modelXYZToAirplaneXForwardZDown + this->model->WorldPose();

    // This below gives the pose of the model in the calculated NED frame
    const Pose3d NEDToModelXForwardZUp = gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

    double Roll, Pitch, Yaw;
    double p, q, r;

    Roll = NEDToModelXForwardZUp.Rot().Roll()*RAD2DEG;
    Pitch = NEDToModelXForwardZUp.Rot().Pitch()*RAD2DEG;
    Yaw = NEDToModelXForwardZUp.Rot().Yaw()*RAD2DEG;

    ignition::math::Vector3d CorrectedAngularVel = gazeboXYZToModelXForwardZDown.Rot().RotateVector(this->model->WorldAngularVel()); // This is p,q,r

    CorrectedAngularVel.Correct();

    p = CorrectedAngularVel.X(); //Check if returns correctly
    q = CorrectedAngularVel.Y(); //Check if returns correctly
    r = CorrectedAngularVel.Z(); //Check if returns correctly

    ignition::math::Vector3d CorrectedVel = gazeboXYZToModelXForwardZDown.Rot().RotateVectorReverse(this->model->WorldLinearVel() -
                                                                           this->world->Wind().LinearVel()); // This is velocity in body frame
    CorrectedVel.Correct();

    double Alpha = atan2(CorrectedVel.Z(), CorrectedVel.X()); // Angle of attack
    double Beta = asin(CorrectedVel.Y()/CorrectedVel.Length()); // Side slip angle

    state.alpha = Alpha;
    state.beta = Beta;

    state.position.x = NEDToModelXForwardZUp.Pos().X();
    state.position.y = NEDToModelXForwardZUp.Pos().Y();
    state.position.z = NEDToModelXForwardZUp.Pos().Z();

    state.orientation.w = NEDToModelXForwardZUp.Rot().W();
    state.orientation.x = NEDToModelXForwardZUp.Rot().X();
    state.orientation.y = NEDToModelXForwardZUp.Rot().Y();
    state.orientation.z = NEDToModelXForwardZUp.Rot().Z();

    state.linearvel.x = CorrectedVel.X();
    state.linearvel.y = CorrectedVel.Y();
    state.linearvel.z = CorrectedVel.Z();

    state.angularvel.x = p;
    state.angularvel.y = q;
    state.angularvel.z = r;


    msg_motor.data = this->Motor_data;

    msg_aero.x = this->Aileron_data;
    msg_aero.y = this->Elevator_data;
    msg_aero.z = this->Rudder_data;

    this->ControlSystemPub.publish(msg_aero);

    this->MotorPub.publish(msg_motor);

    this->StatePub.publish(state);

    ros::spinOnce();

}

