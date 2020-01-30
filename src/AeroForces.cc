#include "AeroForces.hh"

using namespace gazebo;
using namespace std;
using namespace arma;
using namespace ignition::math;

#define RAD2DEG 180/M_PI
#define DEG2RAD M_PI/180

#define AirDensity 1.228

GZ_REGISTER_MODEL_PLUGIN(AeroForce)

/// \brief Constructor
AeroForce::AeroForce()
{

}

/// \brief Load the sdf and model
void AeroForce::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    GZ_ASSERT(_model, "AeroForce _model pointer is NULL");
    this->model = _model; // model pointer
    this->modelName = _model->GetName(); // model name

    GZ_ASSERT(_sdf, "AeroForce _sdf pointer is NULL");
    this->sdf = _sdf; // sdf Element pointer

    this->world = this->model->GetWorld();
    GZ_ASSERT(this->world, "AeroForce world pointer is NULL");

    this->physics = this->world->Physics();
    GZ_ASSERT(this->physics, "AeroForce physics pointer is NULL");

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

    /// \brief Check to see if Surface area of surface is set and get the value
    if (_sdf->HasElement("WingSpan"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("WingSpan");

        this->WingSpan = elem->Get<double>();
    }
    else
    {
        cout << "[Plugin Warn]: WingSpan not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;

        this->WingSpan = 1;
    }

    /// \brief Check to see if chord length is set and get the value
    if (_sdf->HasElement("ChordLength"))
    {
        sdf::ElementPtr elem = _sdf->GetElement("ChordLength");

        this->ChordLength = elem->Get<double>();
    }
    else
    {
        cout << "[Plugin Warn]: Chord length not set in plugin for model name "<< this->modelName <<". It may run incorrectly. " <<endl;

        this->ChordLength = 1;
    }

    /// \brief Check the sdf to which control joints are available
    // AileronJoints
    if (_sdf->HasElement("AileronJoint"))
    {
        // Get the left Aileron JointPtr
        this->AileronLeft = this->model->GetJoint(_sdf->GetElement("AileronJoint")->GetElement("Left")->Get<std::string>());

        // Get the left Aileron JointPtr
        this->AileronRight = this->model->GetJoint(_sdf->GetElement("AileronJoint")->GetElement("Right")->Get<std::string>());

        this->AILERON_FLAG = true;
    }
    else
    {
        // Set to false if control joint is not set
        this->AILERON_FLAG = false;
    }

    // ElevatorJoints
    if (_sdf->HasElement("ElevatorJoint"))
    {
        // Get the Elevator JointPtr
        this->Elevator = this->model->GetJoint(_sdf->GetElement("ElevatorJoint")->Get<std::string>());

        this->ELEVATOR_FLAG = true;
    }
    else
    {
        // Set to false if control joint is not set
        this->ELEVATOR_FLAG = false;
    }

    // RudderJoints
    if (_sdf->HasElement("RudderJoint"))
    {
        // Get the Elevator JointPtr
        this->Rudder = this->model->GetJoint(_sdf->GetElement("RudderJoint")->Get<std::string>());

        this->RUDDER_FLAG = true;

    }
    else
    {
        // Set to false if control joint is not set
        this->RUDDER_FLAG = false;
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

    /// Check to see if AeroCoeff sdf name is available
    if (_sdf->HasElement("AeroCoeffData"))
    {
      sdf::ElementPtr elem = _sdf->GetElement("AeroCoeffData");
      if (elem->HasElement("CoeffLiftDragMoment"))
        this->LiftDragCoeffFileName = elem->GetElement("CoeffLiftDragMoment")->Get<std::string>();
      if (elem->HasElement("CL_q"))
        this->CL_q = elem->GetElement("CL_q")->Get<double>();
      if (elem->HasElement("CD_q"))
        this->CD_q = elem->GetElement("CD_q")->Get<double>();
      if (elem->HasElement("Cm_q"))
        this->Cm_q = elem->GetElement("Cm_q")->Get<double>();
      if (elem->HasElement("CL_de"))
        this->CL_de = elem->GetElement("CL_de")->Get<double>();
      if (elem->HasElement("CD_de"))
        this->CD_de = elem->GetElement("CD_de")->Get<double>();
      if (elem->HasElement("Cm_de"))
        this->Cm_de = elem->GetElement("Cm_de")->Get<double>();
      if (elem->HasElement("Cy_0"))
        this->Cy_0 = elem->GetElement("Cy_0")->Get<double>();
      if (elem->HasElement("Cy_beta"))
        this->Cy_beta = elem->GetElement("Cy_beta")->Get<double>();
      if (elem->HasElement("Cy_p"))
        this->Cy_p = elem->GetElement("Cy_p")->Get<double>();
      if (elem->HasElement("Cy_r"))
        this->Cy_r = elem->GetElement("Cy_r")->Get<double>();
      if (elem->HasElement("Cy_da"))
        this->Cy_da = elem->GetElement("Cy_da")->Get<double>();
      if (elem->HasElement("Cy_dr"))
        this->Cy_dr = elem->GetElement("Cy_dr")->Get<double>();
      if (elem->HasElement("Cl_0"))
        this->Cl_0 = elem->GetElement("Cl_0")->Get<double>();
      if (elem->HasElement("Cl_beta"))
        this->Cl_beta = elem->GetElement("Cl_beta")->Get<double>();
      if (elem->HasElement("Cl_p"))
        this->Cl_p = elem->GetElement("Cl_p")->Get<double>();
      if (elem->HasElement("Cl_r"))
        this->Cl_r = elem->GetElement("Cl_r")->Get<double>();
      if (elem->HasElement("Cl_da"))
        this->Cl_da = elem->GetElement("Cl_da")->Get<double>();
      if (elem->HasElement("Cl_dr"))
        this->Cl_dr = elem->GetElement("Cl_dr")->Get<double>();
      if (elem->HasElement("Cn_0"))
        this->Cn_0 = elem->GetElement("Cn_0")->Get<double>();
      if (elem->HasElement("Cn_beta"))
        this->Cn_beta = elem->GetElement("Cn_beta")->Get<double>();
      if (elem->HasElement("Cn_p"))
        this->Cn_p = elem->GetElement("Cn_p")->Get<double>();
      if (elem->HasElement("Cn_r"))
        this->Cn_r = elem->GetElement("Cn_r")->Get<double>();
      if (elem->HasElement("Cn_da"))
        this->Cn_da = elem->GetElement("Cn_da")->Get<double>();
      if (elem->HasElement("Cn_dr"))
        this->Cn_dr = elem->GetElement("Cn_dr")->Get<double>();

    }

    /*
    cout << "CL_q = " << this->CL_q <<endl;
    cout << "CD_q = " << this->CD_q <<endl;
    cout << "Cm_q = " << this->Cm_q <<endl;
    cout << "CL_de = " << this->CL_de <<endl;
    cout << "CD_de = " << this->CD_de <<endl;
    cout << "Cm_de = " << this->Cm_de <<endl;
    cout << "Cy_0 = " << this->Cy_0 <<endl;
    cout << "Cy_beta = " << this->Cy_beta <<endl;
    cout << "Cy_p = " << this->Cy_p <<endl;
    cout << "Cy_r = " << this->Cy_r <<endl;
    cout << "Cy_da = " << this->Cy_da <<endl;
    cout << "Cy_dr = " << this->Cy_dr <<endl;
    cout << "Cl_0 = " << this->Cl_0 <<endl;
    cout << "Cl_beta = " << this->Cl_beta <<endl;
    cout << "Cl_p = " << this->Cl_p <<endl;
    cout << "Cl_r = " << this->Cl_r <<endl;
    cout << "Cl_da = " << this->Cl_da <<endl;
    cout << "Cl_dr = " << this->Cl_dr <<endl;
    cout << "Cn_0 = " << this->Cn_0 <<endl;
    cout << "Cn_beta = " << this->Cn_beta <<endl;
    cout << "Cn_p = " << this->Cn_p <<endl;
    cout << "Cn_r = " << this->Cn_r <<endl;
    cout << "Cn_da = " << this->Cn_da <<endl;
    cout << "Cn_dr = " << this->Cn_dr <<endl;
    */

    // Load the data
    mat CoeffData;
    CoeffData.load(this->LiftDragCoeffFileName);

    this->Poly_Lift = polyfit(CoeffData.col(0), CoeffData.col(1), 10);
    this->Poly_Drag = polyfit(CoeffData.col(0), CoeffData.col(2), 10);
    this->Poly_Moment = polyfit(CoeffData.col(0), CoeffData.col(3), 10);

    this->mass = 0;
    physics::Link_V v = this->model->GetLinks();

    for(std::vector<physics::LinkPtr>::iterator it = v.begin(); it != v.end(); ++it)
    {
        physics::InertialPtr Inertia;
        Inertia = it->get()->GetInertial();
        this->mass += Inertia->Mass();
    }

    if (this->ELEVATOR_FLAG)
        this->Elevator->SetPosition(0, 0*DEG2RAD, true ); // The last argument is preserveworldvelocity which by by default is false. This is causing some wierd motion behavior
    if (this->AILERON_FLAG){
        this->AileronLeft->SetPosition(0, -0*DEG2RAD, true );
        this->AileronRight->SetPosition(0, 0*DEG2RAD, true );
     }
    if (this->RUDDER_FLAG)
        this->Rudder->SetPosition(0, 0*DEG2RAD, true);

    // Create a name for the gazebo node
    std::string nodename = this->modelName + "_AeroDynamics";

    // Create a ros node
    if(!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, nodename);
    }

    std::string topicName = "/" + this->model->GetName() + "/ControlSurface";

    ROS_INFO("AeroDynamics plugin ready");
    this->Aerodynamics_node.reset(new ros::NodeHandle(nodename));
    this->ControlSub = this->Aerodynamics_node->subscribe(topicName, 1000, &AeroForce::ControlSurfaceMsg, this);

}

/// \brief Initailize
void AeroForce::Init()
{
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&AeroForce::OnUpdate, this));
}

void AeroForce::ControlSurfaceMsg(const geometry_msgs::Vector3ConstPtr &msg)
{

    if (this->AILERON_FLAG)
    {
        // x value in msg is the aileron angle value scaled between -1 and 1
        // Get the lower angle limit and upper angle limit and equate lower limit to -1 and upper limit to 1
        double angle_aileron_left = msg->x*(this->AileronLeft->UpperLimit() - this->AileronLeft->LowerLimit())/2.0; // left aileron angle
        double angle_aileron_right = msg->x*(this->AileronRight->UpperLimit() - this->AileronRight->LowerLimit())/2.0; // right aileron angle

        cout << "aileron left = " << angle_aileron_left << endl;
        cout << "aileron right = " << angle_aileron_right << endl;

        // SetPosition will set joint angle in radians
        this->AileronLeft->SetPosition(0, -angle_aileron_left, true ); // Left aileron has opposite deflection to convention (Trailing edge down is +ve in convention)
        this->AileronRight->SetPosition(0, angle_aileron_right, true );
    }


    if (this->ELEVATOR_FLAG)
    {
        // y value in msg is the elevator angle value scaled between -1 and 1
        // Get the lower angle limit and upper angle limit and equate lower limit to -1 and upper limit to 1
        double angle_elevator = msg->y*(this->Elevator->UpperLimit() - this->Elevator->LowerLimit())/2.0;

        cout << "elevator = " << angle_elevator << endl;

        // SetPosition will set joint angle value in radians
        this->Elevator->SetPosition(0, angle_elevator, true ); // The last argument is preserveworldvelocity which by by default is false. This is causing some wierd motion behavior
\
    }


    if (this->RUDDER_FLAG)
    {
        // z value in msg is the elevator angle value scaled between -1 and 1
        // Get the lower angle limit and upper angle limit and equate lower limit to -1 and upper limit to 1
        double angle_rudder = msg->z*(this->Rudder->UpperLimit() - this->Rudder->LowerLimit())/2.0;
        this->Rudder->SetPosition(0, angle_rudder, true);
    }

}

/// \brief Constant update for each iteration
void AeroForce::OnUpdate()
{
    /*
     *     Reference:
     *     Small Unmanned Aircrafts: Theory and practice by Randal Beard
     */

    //Pose3d modelXYZToAirplaneXForwardZDown(0,0,0,arma::datum::pi,0,0), gazeboXYZToNED(0,0,0,arma::datum::pi,0,arma::datum::pi/2.0);
    Vector3d ForceonBodyFrame, MomentonBodyFrame, CorrectedVel, CorrectedAngularVel;
    Vector3d ElevatorVector, AileronVector, RudderVector;
    double Alpha, Beta; //Angle of attack, side slip angle
    double Lift, Drag, PitchMoment, Fx, Fz;
    double Fy, RollMoment, YawMoment;
    double Va, Dynamic_pressure;
    double SurfaceArea;
    double p, q, r; // p, q, r angular velocities
    double elevator_angle, aileron_angle, rudder_angle; // control angles
    colvec Aoa_vec(1), Cl(1), Cd(1), Cm(1);

    // This below gives the pose of the model in gazebo's XYZ frame
    const Pose3d gazeboXYZToModelXForwardZDown = this->modelXYZToAirplaneXForwardZDown + this->model->WorldPose();

    // This below gives the pose of the model in the calculated NED frame
    const Pose3d NEDToModelXForwardZUp = gazeboXYZToModelXForwardZDown - this->gazeboXYZToNED;

    //cout << "gazeboXYZToModelXForwardZDown = " <<gazeboXYZToModelXForwardZDown << endl;
    //cout << "gazeboXYZToModelXForwardZDown Roll = " << gazeboXYZToModelXForwardZDown.Rot().Roll()*RAD2DEG
    //     << " ,Pitch = " << gazeboXYZToModelXForwardZDown.Rot().Pitch()*RAD2DEG  << " ,Yaw = " << gazeboXYZToModelXForwardZDown.Rot().Yaw()*RAD2DEG  << endl;
    //cout << "NEDToModelXForwardZUp Roll = " << NEDToModelXForwardZUp.Rot().Roll()*RAD2DEG
    //     << " ,Pitch = " << NEDToModelXForwardZUp.Rot().Pitch()*RAD2DEG  << " ,Yaw = " << NEDToModelXForwardZUp.Rot().Yaw()*RAD2DEG  << endl;
    //cout << "NEDToModelXForwardZUp = " << NEDToModelXForwardZUp << endl;


    //cout << "World Linear Vel = " << this->model->WorldLinearVel() << endl;
    //cout << "Wind Linear Vel = " << this->world->Wind().LinearVel() << endl;

    //cout << "Linear Vel NED = " << gazeboXYZToNED.Rot().RotateVector(this->model->WorldLinearVel()) << endl;
    //cout << "Wind Linear Vel NED = " << gazeboXYZToNED.Rot().RotateVector(this->world->Wind().LinearVel()) << endl;

    CorrectedVel = gazeboXYZToModelXForwardZDown.Rot().RotateVectorReverse(this->model->WorldLinearVel() -
                                                                           this->world->Wind().LinearVel()); // This is velocity in body frame
    CorrectedVel.Correct();

    //cout << "Mass = "<< this->mass << endl;
    //cout << "Corrected Vel = " << CorrectedVel << endl;
    //cout << "Vel mag = " << CorrectedVel.Length() <<endl;

    // If velocity is less than 1m/s dont execute the rest of plugin
    if (CorrectedVel.Length() < 2)
        return;

    //cout<<"Acceleration = " << this->model->WorldLinearAccel() <<endl;

    Alpha = atan2(CorrectedVel.Z(), CorrectedVel.X()); // Angle of attack
    Beta = asin(CorrectedVel.Y()/CorrectedVel.Length()); // Side slip angle
    //cout << "Angle of attack (alpha) = " << Alpha << endl;
    //cout << "Side slip angle (beta) = " << Beta*(180/3.14159265) << endl;

    Aoa_vec(0) = Alpha*RAD2DEG; // NOTE: Cl Cd and Cm model curves take deg values for Aoa in Polyval. Need to make it consistent in the future

    //cout << "Angle of attack vec = " << Aoa_vec <<endl;
    Va = CorrectedVel.Length();
    Cl = polyval(this->Poly_Lift, Aoa_vec);
    Cd = polyval(this->Poly_Drag, Aoa_vec);
    Cm = polyval(this->Poly_Moment, Aoa_vec);

    this->CL = Cl(0);
    this->CD = Cd(0);
    this->Cm = Cm(0);

    Dynamic_pressure = 0.5*AirDensity*pow(Va, 2); // dynamic pressure for body in air. NOTE: Need to make it more generic for air, water and other mediums(future)
    SurfaceArea = this->WingSpan*this->ChordLength;

    //cout << "World angular vel" << this->model->WorldAngularVel() << endl;
    //cout << "Xforward Z Down angular vel" << gazeboXYZToModelXForwardZDown.Rot().RotateVector(this->model->WorldAngularVel()) << endl;

    CorrectedAngularVel = gazeboXYZToModelXForwardZDown.Rot().RotateVector(this->model->WorldAngularVel()); // This is p,q,r

    p = CorrectedAngularVel.X(); //Check if returns correctly
    q = CorrectedAngularVel.Y(); //Check if returns correctly
    r = CorrectedAngularVel.Z(); //Check if returns correctly
    //cout << "p - " << p << ", q = " << q << ", r = " << r <<endl;

    elevator_angle = 0;
    if (this->ELEVATOR_FLAG){
            ElevatorVector.X(0);
            ElevatorVector.Y(this->Elevator->Position(0)); // Need to takein axis input in later revisions
            ElevatorVector.Z(0);
            elevator_angle = this->modelXYZToAirplaneXForwardZDown.Rot().RotateVectorReverse(ElevatorVector).Y();
            //cout << "elevator angle = " << elevator_angle*RAD2DEG <<endl;
    }

    aileron_angle = 0;
    if (this->AILERON_FLAG){
        AileronVector.X(0);
        AileronVector.Y(0.5*(this->AileronLeft->Position(0) - this->AileronRight->Position(0))); // Need to takein axis input in later revisions
        AileronVector.Z(0);
        aileron_angle = this->modelXYZToAirplaneXForwardZDown.Rot().RotateVectorReverse(AileronVector).Y();;
        //cout << "aileron angle = " << aileron_angle*RAD2DEG <<endl;
    }

    rudder_angle = 0;
    if (this->RUDDER_FLAG){
        rudder_angle = this->Rudder->Position(0);
        //cout << "rudder angle = " << rudder_angle <<endl;
    }

    Lift = Dynamic_pressure*SurfaceArea*( this->CL + this->CL_q*(this->ChordLength/(2*Va)) + this->CL_de*elevator_angle );
    Drag = 10*Dynamic_pressure*SurfaceArea*( this->CD + this->CD_q*(this->ChordLength/(2*Va)) + this->CD_de*elevator_angle );
    PitchMoment = Dynamic_pressure*SurfaceArea*this->ChordLength*( this->Cm + this->Cm_q*(this->ChordLength/(2*Va))*q + this->Cm_de*elevator_angle );

    Fx = -Drag*cos(Alpha) + Lift*sin(Alpha);
    Fz = -Drag*sin(Alpha) - Lift*cos(Alpha);

    Fy = Dynamic_pressure*SurfaceArea*( this->Cy_0 + this->Cy_beta*Beta + this->Cy_p*(this->WingSpan/(2*Va))*p + this->Cy_r*(this->WingSpan/(2*Va))*r
                                        + this->Cy_da*aileron_angle + this->Cy_dr*rudder_angle );
    RollMoment = Dynamic_pressure*SurfaceArea*this->WingSpan*(this->Cl_0 + this->Cl_beta*Beta + this->Cl_p*(this->WingSpan/(2*Va))*p + this->Cl_r*(this->WingSpan/(2*Va))*r
                                                        + this->Cl_da*aileron_angle + this->Cl_dr*rudder_angle );
    YawMoment = Dynamic_pressure*SurfaceArea*this->WingSpan*(this->Cn_0 + this->Cn_beta*Beta + this->Cn_p*(this->WingSpan/(2*Va))*p + this->Cn_r*(this->WingSpan/(2*Va))*r
                                                       + this->Cn_da*aileron_angle + this->Cn_dr*rudder_angle );


    /*
    cout << "Lift = " << Lift << endl;
    cout << "Drag = " << Drag << endl;
    cout <<"Fx = " << Fx <<endl;
    cout <<"Fy = " << Fy <<endl;
    cout <<"Fz = " << Fz <<endl;
    cout <<"RollMoment = " << RollMoment <<endl;
    cout <<"PitchMoment = " << PitchMoment <<endl;
    cout <<"YawMoment = " << YawMoment <<endl;
    */

    ForceonBodyFrame.X(Fx);
    ForceonBodyFrame.Y(Fy);
    ForceonBodyFrame.Z(Fz);
    MomentonBodyFrame.X(RollMoment);
    MomentonBodyFrame.Y(PitchMoment);
    MomentonBodyFrame.Z(YawMoment);

    //cout << "Force in Body X forward Z down frame  = " << ForceonBodyFrame << endl;
    //cout << "Moment in Body X Forwadr Z down frame = " << MomentonBodyFrame << endl;

    //rotate from X forward to Z down frame to ModelXYZ frame
    Vector3d ForceModelXYZ= this->modelXYZToAirplaneXForwardZDown.Rot().RotateVectorReverse(ForceonBodyFrame);
    Vector3d MomentModelXYZ= this->modelXYZToAirplaneXForwardZDown.Rot().RotateVectorReverse(MomentonBodyFrame);
    //cout << "Force in Model frame  = " << ForceModelXYZ << endl;
    //cout << "Moment in Model frame  = " << MomentModelXYZ << endl;

    ForceModelXYZ.Correct();
    MomentModelXYZ.Correct();
    this->link->AddRelativeForce(ForceModelXYZ);
    this->link->AddRelativeTorque(MomentModelXYZ);
    //cout << "===========================================================" << endl;

    //cout << "Current Position WorldFrame= " <<  << endl;
    //cout << "ChildJoints = " << this->link->GetChildJoints().at(0)->Position(0) << endl;

    ros::spinOnce();

}

