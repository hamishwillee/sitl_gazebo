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

		test_mode_ = simple_spin;
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

		if (_sdf->HasElement(LINK_NAME_TAG)){
			link_name_ = _sdf->GetElement(LINK_NAME_TAG)->Get<std::string>();
 			link_ = model_->GetChildLink(link_name_);
		}
		else {
			gzerr << "sdf tag " << LINK_NAME_TAG << " not found. \n";
		}

		if (_sdf->HasElement(JOINT_NAME_TAG)){
			joint_name_ = _sdf->GetElement(JOINT_NAME_TAG)->Get<std::string>();
			joint_ = model_->GetJoint(joint_name_);
		}
		else {
			gzerr << "sdf tag " << JOINT_NAME_TAG << " not found. \n";
		}

		GZ_ASSERT(world_, "RotorModel world_ pointer is NULL");
		GZ_ASSERT(link_, "RotorModel link_ pointer is NULL");
		GZ_ASSERT(joint_, "RotorModel joint_ pointer is NULL");

		gzdbg << "Loaded link '" << link_->GetName() << "' \n";
		gzdbg << "Loaded joint '" << joint_->GetName() << "' \n";

		// Bind the onUpdate function
		this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
			std::bind(&GazeboRotorModel::OnUpdate, this));
	}


	void GazeboRotorModel::OnUpdate(){
		switch(test_mode_){
			case simple_ground_ride: SimpleGroundRideTest(); break;
			case simple_spin: SimpleSpinTest(); break;
		}
	}

	void GazeboRotorModel::SimpleSpinTest(){
		angularVel_ = link_->RelativeAngularVel();
		
		double targetVel = 10; // rad/s
		double e = targetVel - angularVel_[2];
		double p = 0.00002;

		gzdbg << "angularVel " << angularVel_ << "\n";
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

	GZ_REGISTER_MODEL_PLUGIN(GazeboRotorModel)
}