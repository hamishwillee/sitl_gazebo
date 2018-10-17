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

		test_mode_ = none;
		//test_mode_ = simple_spin;
		//test_mode_ = simple_spin_lift;
		test_simtime_latency_ = 4; // seconds before test is activated
	}

	GazeboRotorModel::~GazeboRotorModel(){
		gzdbg << "Rotor model destr \n";
		updateConnection_->~Connection();
	}

	void GazeboRotorModel::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
		
		GZ_ASSERT(_model, "RotorModel _model pointer is NULL");
		GZ_ASSERT(_sdf, "RotorModel _sdf pointer is NULL");

		model_ = _model;
		world_ = model_->GetWorld();

		if (_sdf->HasElement(LINK_NAME_SDF_TAG)){
			link_name_ = _sdf->GetElement(LINK_NAME_SDF_TAG)->Get<std::string>();
 			link_ = model_->GetChildLink(link_name_);
		}
		else {
			gzerr << "sdf tag " << LINK_NAME_SDF_TAG << " not found. \n";
		}

		if (_sdf->HasElement(JOINT_NAME_SDF_TAG)){
			joint_name_ = _sdf->GetElement(JOINT_NAME_SDF_TAG)->Get<std::string>();
			joint_ = model_->GetJoint(joint_name_);
		}
		else {
			gzerr << "sdf tag " << JOINT_NAME_SDF_TAG << " not found. \n";
		}

		GZ_ASSERT(world_, "RotorModel world_ pointer is NULL");
		GZ_ASSERT(link_, "RotorModel link_ pointer is NULL");
		GZ_ASSERT(joint_, "RotorModel joint_ pointer is NULL");

		gzdbg << "Loaded link '" << link_->GetName() << "' \n";
		gzdbg << "Loaded joint '" << joint_->GetName() << "' \n";

		// Create the transport nodes
		node_handle_ = transport::NodePtr(new transport::Node());
		#if GAZEBO_MAJOR_VERSION < 8
			node_handle_->Init(model_->GetWorld()->GetName());
		#else
			node_handle_->Init(model_->GetWorld()->Name());
		#endif

		std::string rollCmdTopic = "~/" + model_->GetName() + ROLL_CMD_GZTOPIC;
		std::string pitchCmdTopic = "~/" + model_->GetName() + PITCH_CMD_GZTOPIC;

		// Subscribe to the topics, and register a callbacks
		this->control_roll_sub_ = node_handle_->Subscribe(rollCmdTopic,
		   &GazeboRotorModel::OnRollCmdMsg, this);
		this->control_pitch_sub_ = node_handle_->Subscribe(pitchCmdTopic,
		   &GazeboRotorModel::OnPitchCmdMsg, this);		

		// Example of visual pose update
		//visual_pub_ = node_handle_->Advertise<msgs::Visual>("~/visual");
		//uint32_t visualID;
		//link_->VisualId("rotor_visual", visualID);
        //link_->SetVisualPose(visualID, ignition::math::Pose3d(0, 0, 0, 1, 1, 1));

		// Bind the onUpdate function
		this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
			std::bind(&GazeboRotorModel::OnUpdate, this));
	}


	void GazeboRotorModel::OnUpdate(){

		// Activate tests after test_simtime_latency_ passed
		if(world_->SimTime().Double() > test_simtime_latency_){
			switch(test_mode_){
				case simple_ground_ride: SimpleGroundRideTest(); break;
				case simple_spin: SimpleSpinTest(); break;
				case simple_spin_lift: SimpleSpinLiftTest(); break;
			}		
		}
	}

	void GazeboRotorModel::OnRollCmdMsg(ConstAnyPtr &_msg){
		//gzdbg << "Ctrl: R"  << _msg->double_value() << "\n";
		cmd_roll_ = _msg->double_value();
	}

	void GazeboRotorModel::OnPitchCmdMsg(ConstAnyPtr &_msg){
		//gzdbg << "Ctrl: 		P"  << _msg->double_value() << "\n";
		cmd_pitch_ = _msg->double_value();
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

	void GazeboRotorModel::SimpleSpinTest(){
		angularVel_ = link_->RelativeAngularVel();
		
		//double targetVel = 84; // rad/s ... 800rpm
		double targetVel = 84;
		double e = targetVel - angularVel_[2];
		double p = 0.5;

		gzdbg << "angVel " << angularVel_ << " | err: " << e <<"\n";
		link_->AddRelativeTorque(ignition::math::Vector3d(0,0,p*e));
	}

	void GazeboRotorModel::SimpleGroundRideTest(){
		double target = 2.0;
		ignition::math::Pose3d pose = link_->WorldCoGPose();
		double e = target-pose.Pos().X();
		double k = 10;

		link_->AddRelativeForce(ignition::math::Vector3d(k*e, 0, 0));
		gzdbg << link_->RelativeForce() << "\n";
	}

	void GazeboRotorModel::SimpleSpinLiftTest(){
		SimpleSpinTest();

		double omegaLiftFactor = 1.0;
		double lift = angularVel_[2]*omegaLiftFactor;
		lift = ignition::math::clamp(lift, 0.0, 100.0);

		link_->AddRelativeForce(ignition::math::Vector3d(0,0,lift));
		gzdbg << "Lift:" << lift << "\n";
	}

	GZ_REGISTER_MODEL_PLUGIN(GazeboRotorModel)
}