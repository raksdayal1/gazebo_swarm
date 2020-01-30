#include "MotorProp.hh"

using namespace gazebo;
using namespace std;
using namespace arma;
using namespace ignition::math;

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

#define AirDensity 1.228

GZ_REGISTER_MODEL_PLUGIN(MotorProp)

MotorProp::MotorProp()
{

}

void MotorProp::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "MotorProp _model pointer is NULL");
    this->model = _model; // model pointer
    this->modelName = _model->GetName(); // model name

    GZ_ASSERT(_sdf, "MotorProp _sdf pointer is NULL");
    this->sdf = _sdf; // sdf Element pointer

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "MotorProp world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "MotorProp physics pointer is NULL");

    /// Get the name of the surface where the aerodynamic forces act
    if (_sdf->HasElement("ActingSurface"))
    {
      sdf::ElementPtr elem = _sdf->GetElement("ActingSurface");
      this->linkName = elem->Get<std::string>();
      this->link = this->model->GetLink(this->linkName);
    }
    else {
        cout << "[Plugin Warn]: Acting surface not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;
    }

    if (_sdf->HasElement("PropDiameter"))
    {
      sdf::ElementPtr elem = _sdf->GetElement("PropDiameter");
      this->PropDiameter = elem->Get<double>();
    }
    else {
        cout << "[Plugin Warn]: PropArea not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;
        this->PropDiameter = 7;
    }

    // This is motor KV given by the motor specs
    if (_sdf->HasElement("MotorKV"))
    {
        this->MotorKV = _sdf->GetElement("MotorKV")->Get<double>();
    }
    else {
        cout << "[Plugin Warn]: KMotor not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;
        this->MotorKV = 920;
    }

    // This is coeff of thrust
    if (_sdf->HasElement("Ct"))
    {
        this->Ct = _sdf->GetElement("Ct")->Get<double>();
    }else {
        cout << "[Plugin Warn]: KTp not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;
        this->Ct = 0.02;
    }

    // This is the nominal battery voltage from the attached battery
    if (_sdf->HasElement("BatteryVoltage"))
    {
        this->BatteryVoltage = _sdf->GetElement("BatteryVoltage")->Get<double>();
    }else {
        cout << "[Plugin Warn]: BatteryVoltage not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;
        this->BatteryVoltage = 11.1;
    }

    if (this->link->GetChildCount() == 1)
    {
        // Using the link information get the joint
        this->joint = this->link->GetChildJoints().at(0);
    }

    // Create a name for the gazebo node
    std::string nodename = this->modelName + "_Propulsion";

    // Create the ros node
    // Create a ros node
    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodename);
    }

    ROS_INFO("Motor plugin ready");
    this->MotorNode.reset(new ros::NodeHandle(nodename));

    std::string topicName = "/" + this->model->GetName() + "/MotorPropulsion";

    // Subscribe to the topic, and register a callback
    this->MotorSub = this->MotorNode->subscribe(topicName, 1000, &MotorProp::MotorMsg, this);

    this->ScaledControlSignal = 0;

}

void MotorProp::Init()
{
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&MotorProp::OnUpdate, this));
}

void MotorProp::MotorMsg(const std_msgs::Float64ConstPtr &msg)
{

    this->ScaledControlSignal = msg->data;
    //cout << "Motor Scaled Control value = " << this->ScaledControlSignal << endl;
}

void MotorProp::OnUpdate()
{
    /*
    Reference:
    Small Unmanned Aircrafts: Theory and practice by Randal Beard
    */

    double Volt_Out(0), Angularspeed(0), PropRadius(0), Thrust(0);
    Vector3d PropulsionForce(0,0,0);
    //cout << "Battery Volt = " << this->BatteryVoltage <<endl;

    Volt_Out = this->BatteryVoltage*this->ScaledControlSignal;

    //cout << "V = "<<Volt_Out<<endl;

    Angularspeed = (this->MotorKV*Volt_Out)*(2*M_PI/60); // Angular speed in rad/s

    if (this->link->GetChildCount() == 1){ //i.e. a prop is attached then calculate forces and apply to body
        //this->joint->SetVelocity(0, Angularspeed);

        PropRadius = this->PropDiameter*0.0254/2.0;
        Thrust = 0.5*AirDensity*pow(Angularspeed*PropRadius ,2)*M_PI*pow(PropRadius,2)*this->Ct;

        //cout << "Thrust = " << Thrust <<endl;

        PropulsionForce.X(Thrust);
        PropulsionForce.Y(0);
        PropulsionForce.Z(0);

        //cout << "PropulsionForce = " << PropulsionForce << endl;

        //this->model->GetLink("Fuselage")->AddLinkForce(PropulsionForce);
        PropulsionForce.Correct();
        this->link->AddRelativeForce(PropulsionForce);
    }

    ros::spinOnce();

}

