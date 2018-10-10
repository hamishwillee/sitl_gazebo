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

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include "ignition/math/Vector3.hh"

namespace gazebo {

	

	static const std::string LINK_NAME_TAG = "link_name";
	static const std::string JOINT_NAME_TAG = "joint_name";

	class GazeboRotorModel : public ModelPlugin {
	public:

		GazeboRotorModel();
		~GazeboRotorModel();
	
	protected:

		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		virtual void OnUpdate();

	private:

		physics::WorldPtr world_;
		physics::ModelPtr model_;
		physics::LinkPtr link_;
		physics::JointPtr joint_;

		event::ConnectionPtr updateConnection_;

		std::string link_name_;
		std::string joint_name_;

		// Construction/aerodynamic params
		double rotor_radius_;
		double number_of_blades_;
		bool ccw_;

		// State
		double rotor_rpm_;
		ignition::math::Vector3d angularVel_;

		// Testing, choose testmode in constuctor
		enum TestMode { 
			simple_ground_ride, 
			simple_spin
		};

		TestMode test_mode_;
		double test_simtime_latency_;
		void SimpleSpinTest();
		void SimpleGroundRideTest();
	};


}
