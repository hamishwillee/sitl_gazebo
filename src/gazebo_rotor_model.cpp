/*
*
* Copyright (C) 2018 Marek Kuhn, ThunderFly s.r.o
*
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
*/


#include "gazebo_rotor_model.h"


namespace gazebo {

	GazeboRotorModel::GazeboRotorModel(){
		gzdbg << "Rotor model init \n";
		MockParams();
	}

	GazeboRotorModel::~GazeboRotorModel(){
		gzdbg << "Rotor model destr \n";
		updateConnection_->~Connection();
	}

	void GazeboRotorModel::MockParams(){
		gzdbg << "Mocking stuff" << "\n";


	}

	void GazeboRotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
		
		GZ_ASSERT(_model, "RotorModel _model pointer is NULL");
		GZ_ASSERT(_sdf, "RotorModel _sdf pointer is NULL");

		model_ = _model;
		world_ = model_->GetWorld();
		physics_ = world_->Physics();

		ReadSDFParams(_sdf);
		SetupLinks();
		InitSimulation(); 
		InitEventHandlers();


		GZ_ASSERT(world_, "RotorModel world_ pointer is NULL");
		GZ_ASSERT(blade_template_link_, "RotorModel link_ pointer is NULL");
		GZ_ASSERT(rotor_joint_, "RotorModel joint_ pointer is NULL");
		// todo more checks
		
		// Example of visual pose update
		// visual_pub_ = node_handle_->Advertise<msgs::Visual>("~/visual");
		// uint32_t visualID;
		// link_->VisualId("rotor_visual", visualID);
		// link_->SetVisualPose(visualID, ignition::math::Pose3d(0, 0, 0, 1, 1, 1));
		// blade_template_link_->VisualId(LINK_VISUAL_NAME, link_visual_id_);
		// blade_template_link_->VisualId("rotor_cyl_visual", link_visual_cyl_id_);


		// Bind the onUpdate function
		this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
			std::bind(&GazeboRotorModel::OnUpdate, this));
	}

	void GazeboRotorModel::ReadSDFParams(sdf::ElementPtr _sdf){
		gzdbg << "Reading SDF params." << "\n";

		if(_sdf->HasElement(SDF_TAG_BLADE_TEMPLATE_LINK)){
 			blade_template_link_ = model_->GetChildLink(_sdf->GetElement(SDF_TAG_BLADE_TEMPLATE_LINK)->Get<std::string>());
		}
		else {
			gzerr << "sdf tag " << SDF_TAG_BLADE_TEMPLATE_LINK << " not found. \n";
		}

		if(_sdf->HasElement(SDF_TAG_PARENT_LINK)){
			parent_link_ = model_->GetChildLink(_sdf->GetElement(SDF_TAG_PARENT_LINK)->Get<std::string>());
		}
		else {
			gzerr << "sdf tag " << SDF_TAG_PARENT_LINK << "not found. \n";
		}

		if(_sdf->HasElement(SDF_TAG_SHAFT_TEMPLATE_LINK)){
			shaft_template_link_ = model_->GetChildLink(_sdf->GetElement(SDF_TAG_SHAFT_TEMPLATE_LINK)->Get<std::string>());
		}
		else {
			gzerr << "sdf tag " << SDF_TAG_SHAFT_TEMPLATE_LINK << "not found. \n";
		}

		if(_sdf->HasElement(SDF_TAG_FLAP_JOINT_TEMPLATE)){
			flap_joint_template_ = model_->GetJoint(_sdf->GetElement(SDF_TAG_FLAP_JOINT_TEMPLATE)->Get<std::string>());
		}
		else {
			gzerr << "sdf tag " << SDF_TAG_FLAP_JOINT_TEMPLATE << "not found. \n";
		}
		
		if(_sdf->HasElement(SDF_TAG_ROTOR_POSE)){
			rotor_pose_ = _sdf->GetElement(SDF_TAG_ROTOR_POSE)->Get<ignition::math::Pose3d>();
		}
		else {
			gzerr << "sdf tag " << SDF_TAG_ROTOR_POSE << "not found. \n";
		}

		if(_sdf->HasElement(SDF_TAG_TEST_CASE)){
			test_case_ = _sdf->GetElement(SDF_TAG_TEST_CASE)->Get<std::string>();

			if(_sdf->HasElement(SDF_TAG_TEST_CASE_DELAY)){
				test_delay_ = _sdf->GetElement(SDF_TAG_TEST_CASE_DELAY)->Get<double>();
			}
			gzdbg << "[Rotor] Running in testmode: " << test_case_ << "\n";
		}
		gzdbg << "testcase " << test_case_ ;

		getSdfParam<int>(_sdf, SDF_TAG_BLADE_COUNT, n_blades_, n_blades_);
		getSdfParam<double>(_sdf, SDF_TAG_BLADE_REL_START, blade_rel_start_, blade_rel_start_);
		getSdfParam<double>(_sdf, SDF_TAG_BLADE_FLAP_ANGLE_MAX, blade_flap_angle_max_, blade_flap_angle_max_);
		getSdfParam<double>(_sdf, SDF_TAG_BLADE_FLAP_ANGLE_MIN, blade_flap_angle_min_, blade_flap_angle_min_);
		getSdfParam<double>(_sdf, SDF_TAG_PROFILE_PITCH, profile_pitch_, profile_pitch_);
		getSdfParam<double>(_sdf, SDF_TAG_PROFILE_CHORD, profile_pitch_, profile_pitch_);
		getSdfParam<double>(_sdf, SDF_TAG_ROTOR_PITCH_MAX, rotor_pitch_max_, rotor_pitch_max_);
		getSdfParam<double>(_sdf, SDF_TAG_ROTOR_PITCH_MIN, rotor_pitch_min_, rotor_pitch_min_);
		getSdfParam<double>(_sdf, SDF_TAG_ROTOR_ROLL_MAX, rotor_roll_max_, rotor_roll_max_);
		getSdfParam<double>(_sdf, SDF_TAG_ROTOR_ROLL_MIN, rotor_roll_min_, rotor_roll_min_);

		// // 
		// if (_sdf->HasElement("control_channels")) {
		//     sdf::ElementPtr control_channels = _sdf->GetElement("control_channels");
		//     sdf::ElementPtr channel = control_channels->GetElement("channel");
		// if(getSdfParam<std::string>(_sdf, LINK_NAME_SDF_TAG, link_name_, NULL, true)){
		// 	link_ = model_->GetChildLink(link_name_);
		// }

	}

	void GazeboRotorModel::SetupLinks(){
		gzdbg << "Setup links " << "\n";

		// Construct the shaft link & joint
		shaft_link_ = InsertLinkCopy(model_, shaft_template_link_, "shaft", rotor_pose_);
		rotor_joint_ = model_->CreateJoint("rotor_joint_0", "revolute", parent_link_, shaft_link_);
		rotor_joint_->SetAxis(0, rotor_pose_.Rot().ZAxis());
		rotor_joint_->Init();

		// Create blades
		blades_.resize(n_blades_);
		for(int i = 0; i<n_blades_; i++){
			ignition::math::Pose3d pose = ignition::math::Pose3d(
				rotor_pose_.Pos(), 
				ignition::math::Quaternion<double>(rotor_pose_.Rot().ZAxis(), 
				i*2*3.1416/n_blades_)
			);

			physics::LinkPtr blade_link = InsertLinkCopy(
				model_, blade_template_link_, "phantom_blade_"+std::to_string(i), pose);
	
		 	//physics::JointPtr blade_joint_template = model_->GetJoint("flap_joint_template");
			physics::JointPtr blade_joint =  model_->CreateJoint("flap_joint_"+std::to_string(i), "revolute", shaft_link_, blade_link);

		 	// Copy SDF params from template joint (without copying the joint doesn't show in inspector)
			sdf::ElementPtr params(new sdf::Element);
  	 		params->Copy(flap_joint_template_->GetSDF()->Clone());
			params->GetAttribute("name")->Set("flap_joint_"+std::to_string(i));
  	 		params->GetElement("parent")->Set(shaft_link_->GetName());
  	 		params->GetElement("child")->Set(blade_link->GetName());
			
			// Load SDF Params and override
			blade_joint->Load(params);
			blade_joint->SetAxis(0, -pose.Rot().YAxis());
			blade_joint->SetLowerLimit(0, blade_flap_angle_min_);
			blade_joint->SetUpperLimit(0, blade_flap_angle_max_);
			// blade_joint->SetParam("friction", 0, 0.1);
			// blade_joint->SetStiffnessDamping(0, 100.0, 0.5, 0.05);
			blade_joint->Init();

			blades_[i].link = blade_link;
			blades_[i].flap_joint = blade_joint;
		}

		// Move the template links away
		blade_template_link_->SetWorldPose(ignition::math::Pose3d(0,0,0,0,0,0));
		blade_template_link_->SetLinkStatic(true);
		blade_template_link_->SetKinematic(true);
		shaft_template_link_->SetWorldPose(ignition::math::Pose3d(0,0,0,0,0,0));
		shaft_template_link_->SetLinkStatic(true);
		shaft_template_link_->SetKinematic(true);
	}

	void GazeboRotorModel::InitSimulation(){
		gzdbg << "InitSimulation " << "\n";

		blade_cog_pos_ = blade_cog_rel_pos_ * rotor_radius_;
	
		// Prepare discretization
		for(int i = 0; i<n_blades_; i++){
			blades_[i].elements.resize(n_elements_);
			for(int j = 0; j<n_elements_; j++){
				BladeElement* e = &(blades_[i].elements[j]);
				e->dr = rotor_radius_/(double)n_elements_;
				e->r = e->dr*j + e->dr*0.5;		// add minimal offset from rotor shaft
			}
		}
		gzdbg << "Initialized "<< n_blades_*n_elements_ <<" elements on " << n_blades_ << " blades. " << "Dr: " << blades_[0].elements[0].dr << " \n";

		// Initial conditions
		// rotor_joint_->SetVelocity(0,60.0);	
	}

	void GazeboRotorModel::InitEventHandlers(){
		
		// Create the transport nodes
		node_handle_ = transport::NodePtr(new transport::Node());
		node_handle_->Init(model_->GetWorld()->Name());

		std::string rollCmdTopic = "~/" + model_->GetName() + ROLL_CMD_GZTOPIC;
		std::string pitchCmdTopic = "~/" + model_->GetName() + PITCH_CMD_GZTOPIC;

		// Subscribe to the topics, and register a callbacks
		control_roll_sub_ = node_handle_->Subscribe(rollCmdTopic, &GazeboRotorModel::OnRollCmdMsg, this);
		control_pitch_sub_ = node_handle_->Subscribe(pitchCmdTopic, &GazeboRotorModel::OnPitchCmdMsg, this);		
		rotor_speed_pub_ = node_handle_->Advertise<msgs::Any>("~/" + model_->GetName() + ROTOR_SPEED_GZTOPIC, 1);
		blade_flapping_pub_ = node_handle_->Advertise<msgs::Any>("~/" + model_->GetName() + BLADE_FLAPPING_GZTOPIC, 1);

		// For testing only
		control_lift_sub_ = node_handle_->Subscribe("~/" + model_->GetName() + "/rotor_lift_cmd", &GazeboRotorModel::OnLiftCmdMsg, this);
	}
	
	void GazeboRotorModel::OnUpdate(){

		// Update state
		rotor_omega_ = rotor_joint_->GetVelocity(0);
		rotor_ksi_ = shaft_link_->RelativePose().Rot().Yaw();
		blade_flapping_ = blades_[0].flap_joint->Position(0);

		// Publish state
		msgs::Any msg;
		msg = msgs::ConvertAny(rotor_omega_);
		rotor_speed_pub_->Publish(msg);
		msg = msgs::ConvertAny(blade_flapping_);
		blade_flapping_pub_->Publish(msg);

		// Activate test method or run nnormal simulation
		if(test_case_.empty()){
			if(world_->SimTime().Double() > 10){  // wait until gazebo/px4 starts
				RunSimulation();
			}
		} 
		else {
			(this->*testMethods[test_case_])();
		}

		// Update visuals example
		//link_->SetVisualPose(link_visual_id_, ignition::math::Pose3d(ignition::math::Vector3d(0.0, 0.0, 0.0), real_rot));
		//link_->SetVisualPose(link_visual_cyl_id_, ignition::math::Pose3d(ignition::math::Vector3d(0.0, 0.0, 0.05), real_rot));
	}

	void GazeboRotorModel::OnRollCmdMsg(ConstAnyPtr &_msg){
		//gzdbg << "Ctrl: R "  << _msg->double_value() << "\n";
		double value = _msg->double_value();

		if (value > 0){
			cmd_roll_ = value*rotor_roll_max_;
		}
		else {
			cmd_roll_ = value*(-rotor_roll_min_);
		}
		
		physics::JointPtr roll_joint = model_->GetJoint("roll_dummyjoint");
		roll_joint->SetPosition(0,cmd_roll_);
	}

	void GazeboRotorModel::OnPitchCmdMsg(ConstAnyPtr &_msg){
		//gzdbg << "Ctrl: 		P"  << _msg->double_value() << "\n";
		double value = -_msg->double_value(); // inverting ???
		if (value > 0){
			cmd_pitch_ = value*rotor_pitch_max_;
		}
		else {
			cmd_pitch_ = value*(-rotor_pitch_min_);
		}

		physics::JointPtr pitch_joint = model_->GetJoint("pitch_dummyjoint");
		pitch_joint->SetPosition(0,-cmd_pitch_); // inverting axis
	}

//  Testing other channels
/*
	void GazeboRotorModel::On5(ConstAnyPtr &_msg){
		gzdbg << "Ctrl:				5:"  << _msg->double_value() << "\n";
	}

	void GazeboRotorModel::On6(ConstAnyPtr &_msg){
		gzdbg << "Ctrl:						6:"  << _msg->double_value() << "\n";
	}

	void GazeboRotorModel::On7(ConstAnyPtr &_msg){
		gzdbg << "Ctrl:							7:"  << _msg->double_value() << "\n";
	}
*/

	void GazeboRotorModel::OnLiftCmdMsg(ConstAnyPtr &_msg){
		cmd_lift_ = _msg->double_value();
	}


	void GazeboRotorModel::RunSimulation(){

		ignition::math::Vector3d fuselage_roll_axis = parent_link_->WorldPose().Rot().XAxis();
		ignition::math::Vector3d fuselage_pitch_axis = parent_link_->WorldPose().Rot().YAxis();

		for(int i = 0; i<n_blades_; i++){

			ignition::math::Pose3d blade_pose = blades_[i].link->WorldPose();

		 	ignition::math::Quaternion<double> roll_rot = ignition::math::Quaternion<double>(
				fuselage_roll_axis, cmd_roll_);

			ignition::math::Quaternion<double> pitch_rot = ignition::math::Quaternion<double>(
				fuselage_pitch_axis, -cmd_pitch_);

		 	ignition::math::Pose3d blade_cmd_pose = ignition::math::Pose3d(blade_pose.Pos(),
		 			blade_pose.CoordRotationAdd(roll_rot));

		 	//gzdbg << "cmd " << cmd_roll_ << " rax " << fuselage_roll_axis << " cmdpos " << blade_cmd_pose << "\n";

			ignition::math::Pose3d rotor_origin = rotor_joint_->WorldPose();
			ignition::math::Vector3d rotor_origin_vel = rotor_joint_->GetParent()->WorldLinearVel();
			
			// Read directions
			ignition::math::Vector3d blade_centrifugal_direction = roll_rot*pitch_rot*blade_pose.Rot().XAxis();
			ignition::math::Vector3d blade_normal_direction = roll_rot*pitch_rot*blade_pose.Rot().ZAxis();  // no twisting 
			ignition::math::Vector3d blade_radial_direction = roll_rot*pitch_rot*blade_pose.Rot().YAxis(); // acw, "thumb rule"
			
			//gzdbg << "Blade_dirs: C " << blade_centrifugal_direction << " N " << blade_normal_direction << " R " << blade_radial_direction << "\n";
			
			// Update elements 
			double Fz = 0;
			double Fx = 0;
			double L = 0;

			for(int j = 0; j<n_elements_; j++){
				BladeElement* e = &(blades_[i].elements[j]);

				// Calculate local wind at current blade element
				e->abs_pos = rotor_origin.Pos() + e->r*blade_centrifugal_direction;
				e->abs_vel = rotor_origin_vel + e->r*rotor_omega_*blade_radial_direction; 	// currently ignores flapping velocity!
				e->local_wind = world_wind_ - e->abs_vel;									
				// gzdbg << "local wind" << e->local_wind << " \n";

				// Calculate Angle of attack alfa
				double ut = e->local_wind.Dot(-blade_radial_direction); 	// ut speed goes agaist the radial direction
				double up = e->local_wind.Dot(-blade_normal_direction);   	// up speed points down as in Leishman
				double u = sqrt(up*up + ut*ut);   // local wind magnitude, can be simplified to ut
				double phi = up/ut;
				double alfa = profile_pitch_ - phi;	// tan-1 (up/ut) = up/ut for small angles

				//Re, Ma
				//todo dodelat zavislost Cl, Cd

				// Calculate lift and drag coefficients
				double alfa_deg = alfa*180/3.1416;
				double Cl = 0.1*alfa_deg; 
				double Cd = 0.02+(0.01*alfa_deg)*(0.01*alfa_deg);

				// Adjust aerodynamic coefficients if stalls, todo - improve
				Cl = ignition::math::clamp(Cl,-12.0,12.0);
				Cd = ignition::math::clamp(Cd, 0.0, 0.06);

				// Calculate lift and drag forces 
				double dL = 0.5*air_rho_*u*u*profile_chord_*Cl*e->dr;
				double dD = 0.5*air_rho_*u*u*profile_chord_*Cd*e->dr;

				// Project forces to blade coordinates
				Fz += dL*cos(phi) + dD*sin(phi);
				Fx += dL*sin(phi) + dD*cos(phi);

				// Integrate L over elements
				L += dL;

				e->up = up;
				e->ut = ut;
				e->phi = phi;
				e->alfa = alfa;
				e->Cl = Cl;
				e->Cd = Cd;
				e->dL = dL;
			}


			ignition::math::Vector3d F_world_frame = blade_pose.Rot()*ignition::math::Vector3d(0,-Fx,Fz);;
			//ignition::math::Vector3d F_world_frame = blade_pose.Rot()*ignition::math::Vector3d(0,0,25);

			ignition::math::Vector3d F_rotated = pitch_rot*roll_rot*F_world_frame;
			ignition::math::Vector3d F_local =  blade_pose.Rot().Inverse()*F_rotated;

			blades_[i].link->AddLinkForce(F_local, ignition::math::Vector3d(blade_cog_pos_,0,0 ));

			// gzdbg
			// 	<< "FW " << F_world_frame
			// 	//<< " r_A" << fuselage_roll_axis
			// 	//<< " ROLL_rot" << roll_rot
			// 	<< " F_local " << F_local 
			// << "\n";

			// gzdbg << "F: " << F_b << "\n";
			
			// gzdbg 
			// 	<<	" ksi " << rotor_ksi_ 
			// 	<<	" Pitch " << cmd_pitch_ 
			// 	<<  " F_loc " << F_local 
			// 	<< " Fz: " << Fz 
			// 	<< " Fx " << -Fx 
			// 	<< " Fw " << F_world_frame 
			// << "\n";
			

			// Flapping angle
			double Mcf = (blade_mass_*rotor_omega_*rotor_omega_*rotor_radius_*rotor_radius_)/3;
			double flap = (L*0.5-blade_mass_*9.82*0.5)/Mcf;

			// gzdbg << "Mcf" << Mcf << " Fz" << Fz << " Fx " << Fx << " flap " << flap << "\n" ;

		} // /end for blades
			
		// gzdbg 
		// 	<< blades_[0].elements[0].Cd << " "
		// 	<< blades_[0].elements[1].Cd << " "
		// 	<< blades_[0].elements[2].Cd << " "
		// 	<< blades_[0].elements[3].Cd << " "
		// 	<< blades_[0].elements[4].Cd << " "
		// << "\n";
	}


	/* TESTS */

	void GazeboRotorModel::SimpleSpinTest(){
		
		double targetVel = 84; // rad/s ... 800rpm
		double e = targetVel - rotor_omega_;
		double p = 0.5;

		blades_[0].link->AddRelativeTorque(ignition::math::Vector3d(0,0,p*e));
	}

	void GazeboRotorModel::SimpleGroundRideTest(){
		double target = 2.0;
		ignition::math::Pose3d pose = blade_template_link_->WorldCoGPose();
		double e = target-pose.Pos().X();
		double k = 10;

		blades_[0].link->AddRelativeForce(ignition::math::Vector3d(k*e, 0, 0));
		gzdbg << blades_[0].link->RelativeForce() << "\n";
	}

	void GazeboRotorModel::SimpleSpinLiftTest(){
		SimpleSpinTest();
		SimpleLiftTest();
	}

	void GazeboRotorModel::SimpleLiftTest(){
		double lift = cmd_lift_;
		blades_[0].link->AddRelativeForce(ignition::math::Vector3d(0,0,lift));
		gzdbg << " Lift:   " << lift << "\n";
	}

	void GazeboRotorModel::AeroLiftTest(){

		// Set initial shaft velocity
		// if(test_counter_ < 1){
		// 	rotor_joint_->SetVelocity(0,10.0);	e.Rot().Inverse()*ignition::math::Vector3d(0,-Fx,Fz);
			//ignition::math::Vector3d F_world_frame = blade_pos
		// 	test_counter_++;
		// }

		// Move the test stand
		//test1Base->SetLinearVel(ignition::math::Vector3d(0, 0, 0));
		//test1Base->AddRelativeForce(ignition::math::Vector3d(10,0,0));


		if(world_->SimTime().Double() > test_delay_){
			// Setup wind
			double wind_speed = 20;
			ignition::math::Vector3d world_wind_direction = ignition::math::Vector3d(0,0,1).Normalize();
			world_wind_ = wind_speed * world_wind_direction;
			gzdbg << "World wind " << world_wind_ << "\n";

			RunSimulation();
		}

		// Rotate the axis
		// if (rotor_joint_tmp_pitch_ < 0.8){
		// 	rotor_joint_tmp_pitch_ +=0.0001;
		// 	joint_->SetAxis(0, ignition::math::Vector3d(0, rotor_joint_tmp_pitch_, 1).Normalize());
		// 	gzdbg << "rotor_joint_tmp_pitch_ " << rotor_joint_tmp_pitch_ << "\n";
		// }

		// Rotate the shaft pose
		// if(rotor_joint_tmp_pitch_ <0.8){
		// 	rotor_joint_tmp_pitch_ += 0.01;

		// 	ignition::math::Pose3d oldPose = test1Shaft->RelativePose(); 
		// 	ignition::math::Vector3d oldPos = oldPose.Pos();
		// 	ignition::math::Quaternion<double> oldRot = oldPose.Rot();

		// 	ignition::math::Quaternion<double> pitchIncrement = ignition::math::Quaternion<double>(
		// 		oldRot.YAxis(), 0.01);

		// 	ignition::math::Quaternion<double> newRot = oldPose.Rot() * pitchIncrement;
		// 	ignition::math::Pose3d newPose = ignition::math::Pose3d(oldPos, newRot);

		// 	test1Shaft->SetRelativePose(newPose);
		// 	gzdbg << "setting pose " << newPose << "\n";
		// } else {
		// 	gzdbg << test1Shaft->RelativePose() << "\n";
		// }

		// Step set pose
		// if (rotor_joint_tmp_pitch_ < 0.8){
		// 	rotor_joint_tmp_pitch_ +=0.0001;

		// 	ignition::math::Pose3d newPose = (ignition::math::Pose3d(0,0,0,0,rotor_joint_tmp_pitch_,0));
		// 	test1Shaft->SetRelativePose(newPose);
		// 	gzdbg << "setting pose " << newPose << "\n";

		// }
		
		// if(test_counter_ < 3000){
		// 	test1Shaft->AddRelativeTorque(ignition::math::Vector3d(0,0,20));
		// 	test_counter_++;
		// }

	}

	void GazeboRotorModel::AutorotCarTest(){

		if(world_->SimTime().Double() < test_delay_){
			return;
		}

		double wind_speed = 14;
		ignition::math::Vector3d world_wind_direction = ignition::math::Vector3d(0,0,1).Normalize();
		world_wind_ = wind_speed * world_wind_direction;
		gzdbg << "World wind " << world_wind_ << "\n";

		RunSimulation();
	}

	void GazeboRotorModel::AutorotVerticalWindTest(){

		// Run Once
		if(test_counter_ < 1){
			rotor_joint_->SetVelocity(0,60.0);	

			double wind_speed = 20;
			ignition::math::Vector3d world_wind_direction = ignition::math::Vector3d(0,0,1).Normalize();
			world_wind_ = wind_speed * world_wind_direction;
			gzdbg << "World wind " << world_wind_ << "\n";

			test_counter_++;
		}

		if(world_->SimTime().Double() < test_delay_){
			return;
		}

		if(test_counter_ < 2){
			gzdbg << "Simulation activated!" << " Wind " << world_wind_ << "\n";
			test_counter_++;
		}

		RunSimulation();
	}

	void GazeboRotorModel::RotorControlTest(){


		
		if(test_counter_ < 1){
			/* Run once at the start code start */	
			rotor_joint_->SetVelocity(0,0.5);	


			double wind_speed = 0;
			ignition::math::Vector3d world_wind_direction = ignition::math::Vector3d(-0.5,0,1).Normalize();
			world_wind_ = wind_speed * world_wind_direction;
			gzdbg << "World wind " << world_wind_ << "\n";

			/* Run once at the start code end */
			test_counter_++;
		}

		if(world_->SimTime().Double() < test_delay_){
			return;
		}

		if(test_counter_ < 2){
			/* Runonce after delay code */


			/* Run once after delay end */

			gzdbg << "Simulation activated!" << " Wind " << world_wind_ << "\n";
			test_counter_++;
		}

		rotor_joint_->SetVelocity(0,60.0);	
		
		// Prism joint force measure example
		// physics::JointPtr prismJoint = model_->GetJoint("prism");
		// physics::LinkPtr prism_shaft = model_->GetChildLink("prism_shaft");
		// physics::JointWrench forces = prismJoint->GetForceTorque(0);
		// ignition::math::Vector3d pf =  forces.body2Force;
		// // gzdbg << "body2Force " << pf << "\n";

		RunSimulation();
	}


	GZ_REGISTER_MODEL_PLUGIN(GazeboRotorModel)
}