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

#ifndef _ROTOR_MODEL_PLUGIN_HH_
#define _ROTOR_MODEL_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>

namespace gazebo {

	static const std::string LINK_NAME_SDF_TAG = "link_name";
	static const std::string JOINT_NAME_SDF_TAG = "joint_name";
	static const std::string ROLL_CMD_GZTOPIC = "/rotor_roll_cmd";
	static const std::string PITCH_CMD_GZTOPIC = "/rotor_pitch_cmd";

	class GazeboRotorModel : public ModelPlugin {
	public:

		GazeboRotorModel();
		~GazeboRotorModel();
	
	protected:

		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		virtual void OnUpdate();

		void OnRollCmdMsg(ConstAnyPtr &_msg);
		void OnPitchCmdMsg(ConstAnyPtr &_msg);

		// Testing other channels
		/*
		void On5(ConstAnyPtr &_msg);
		void On6(ConstAnyPtr &_msg);
		void On7(ConstAnyPtr &_msg);
		*/

	private:

		physics::WorldPtr world_;
		physics::ModelPtr model_;
		physics::LinkPtr link_;
		physics::JointPtr joint_;

		event::ConnectionPtr updateConnection_;

		transport::NodePtr node_handle_;
  		transport::PublisherPtr rotor_pub_;
  		transport::PublisherPtr visual_pub_;
  		transport::SubscriberPtr control_roll_sub_;
  		transport::SubscriberPtr control_pitch_sub_;

////  Testing other channels
/*
		transport::SubscriberPtr control_5_sub_;
		transport::SubscriberPtr control_6_sub_;
		transport::SubscriberPtr control_7_sub_;
*/

		std::string link_name_;
		std::string joint_name_;

		// Construction/aerodynamic params
		double rotor_radius_;
		double number_of_blades_;
		bool ccw_;

		// State
		double rotor_rpm_;
		double cmd_pitch_;
		double cmd_roll_;

		ignition::math::Pose3d pose_;
		ignition::math::Vector3d angularVel_;

		// Testing, choose testmode in constuctor
		enum TestMode {
			none, 
			simple_ground_ride, 
			simple_spin,
			simple_spin_lift
		};

		TestMode test_mode_;
		double test_simtime_latency_;
		void SimpleSpinTest();
		void SimpleGroundRideTest();
		void SimpleSpinLiftTest();
	};
}

#endif // DEFINE _ROTOR_MODEL_PLUGIN_HH_