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

#include "common.h"

namespace gazebo {

		static const std::string SDF_TAG_BLADE_TEMPLATE_LINK = "blade_link";
		static const std::string SDF_TAG_PARENT_LINK = "parent_link";
		static const std::string SDF_TAG_SHAFT_TEMPLATE_LINK = "shaft_link";
		static const std::string SDF_TAG_FLAP_JOINT_TEMPLATE = "flap_joint";

		static const std::string SDF_TAG_ROTOR_POSE = "pose";
		static const std::string SDF_TAG_ROTOR_RADIUS = "radius";
		static const std::string SDF_TAG_ROTOR_PITCH_MAX = "rotor_pitch_max";
		static const std::string SDF_TAG_ROTOR_PITCH_MIN = "rotor_pitch_min";
		static const std::string SDF_TAG_ROTOR_ROLL_MAX = "rotor_roll_max";
		static const std::string SDF_TAG_ROTOR_ROLL_MIN = "rotor_roll_min";

		static const std::string SDF_TAG_BLADE_COUNT = "blade_count";
		static const std::string SDF_TAG_BLADE_REL_START = "blade_rel_start";
		static const std::string SDF_TAG_BLADE_REL_COG = "blade_rel_cog_pos";
		static const std::string SDF_TAG_BLADE_FLAP_ANGLE_MAX = "blade_flap_angle_max";
		static const std::string SDF_TAG_BLADE_FLAP_ANGLE_MIN = "blade_flap_angle_min";

		static const std::string SDF_TAG_PROFILE_PITCH = "profile_pitch";
		static const std::string SDF_TAG_PROFILE_CHORD = "profile_chord";

		static const std::string SDF_TAG_TEST_CASE_DELAY = "test_delay";
		static const std::string SDF_TAG_TEST_CASE = "test_case";

		static const std::string ROTOR_SPEED_GZTOPIC = "/rotor_speed";
		static const std::string BLADE_FLAPPING_GZTOPIC = "/blade_flapping";
		static const std::string ROLL_CMD_GZTOPIC = "/rotor_roll_cmd";
		static const std::string PITCH_CMD_GZTOPIC = "/rotor_pitch_cmd";


	class GazeboRotorModel : public ModelPlugin {


	public:

		GazeboRotorModel();
		~GazeboRotorModel();
	
	protected:

		virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		virtual void OnUpdate();

		void ReadSDFParams(sdf::ElementPtr _sdf);
		void SetupLinks();
		void InitSimulation();
		void InitEventHandlers();
		void RunSimulation();


		void OnRollCmdMsg(ConstAnyPtr &_msg);
		void OnPitchCmdMsg(ConstAnyPtr &_msg);
		void OnLiftCmdMsg(ConstAnyPtr &_msg);

		void MockParams();

		// Testing other channels
		/*
		void On5(ConstAnyPtr &_msg);
		void On6(ConstAnyPtr &_msg);
		void On7(ConstAnyPtr &_msg);
		*/

	private:

		physics::WorldPtr world_;
		physics::ModelPtr model_;
		physics::PhysicsEnginePtr physics_;


		physics::LinkPtr blade_template_link_;
		physics::LinkPtr shaft_template_link_;
		physics::LinkPtr parent_link_;
		physics::LinkPtr shaft_link_;
		physics::JointPtr rotor_joint_;
		physics::JointPtr flap_joint_template_;


		ignition::math::Pose3d rotor_pose_;

		event::ConnectionPtr updateConnection_;

		transport::NodePtr node_handle_;
  		transport::PublisherPtr rotor_speed_pub_;
  		transport::PublisherPtr blade_flapping_pub_;
  		transport::PublisherPtr visual_pub_;
  		transport::SubscriberPtr control_roll_sub_;
  		transport::SubscriberPtr control_pitch_sub_;
  		transport::SubscriberPtr control_lift_sub_;

		// Testing other channels
		/*
		transport::SubscriberPtr control_5_sub_;
		transport::SubscriberPtr control_6_sub_;
		transport::SubscriberPtr control_7_sub_;
		*/

		std::string link_name_;
		std::string joint_name_;
		uint32_t link_visual_id_;
		uint32_t link_visual_cyl_id_;

		// Fields 
		struct BladeElement {
			
			double r;		// offset from 
			double dr;
			double alfa; 	// relative wind
			double theta;   // pitch
			double ut;
			double up;
			double phi;
			double Cl;
			double Cd;
			double dL;

			ignition::math::Vector3d abs_pos;
			ignition::math::Vector3d abs_vel;
			ignition::math::Vector3d abs_wind;
			ignition::math::Vector3d local_wind;
		};


		struct Blade {
			physics::LinkPtr link;
			physics::JointPtr flap_joint;

			double flapping_angle;
			std::vector<BladeElement> elements;
		};
		std::vector<Blade> blades_;

	
		// Construction/aerodynamic params, default values
		double rotor_radius_ = 1.0;
		double rotor_pitch_max_ = 0;
		double rotor_pitch_min_ = 0;
		double rotor_roll_max_ = 0;
		double rotor_roll_min_ = 0;
		int n_blades_ = 2;
		int n_elements_ = 8;
		bool ccw_ = true;
		double air_rho_ = 1.225;
		double profile_chord_ = 0.05;
		double profile_pitch_ = 1.5*3.1416/180;
		double blade_cog_rel_pos_ = 0.66;
		double blade_mass_ = 0.5;
		double blade_rel_start_ = 0;
		double blade_flap_angle_min_ = 0;
		double blade_flap_angle_max_ = 0;

		// Computed parameters (in InitSimulation())
		double blade_cog_pos_;
		double rotor_joint_tmp_pitch_;

		ignition::math::Vector3d world_wind_ = ignition::math::Vector3d(0,0,0);


		// State
		double rotor_omega_;
		double rotor_ksi_; // angular position
		double blade_flapping_;
		double cmd_pitch_;
		double cmd_roll_;
		double cmd_lift_;// for testing
	

		// Testing
		std::string test_case_ = "";
		double test_delay_;
		double test_counter_ = 0;

		void SimpleSpinTest();
		void SimpleGroundRideTest();
		void SimpleSpinLiftTest();
		void SimpleLiftTest();
		void AeroLiftTest();
		void AutorotCarTest();
		void AutorotVerticalWindTest();
		void RotorControlTest();

		typedef void (GazeboRotorModel::*TestFunction)(void);
		std::map<std::string, TestFunction> testMethods {
			{"simple_spin", 				&GazeboRotorModel::SimpleSpinTest},
			{"simple_ground_ride",			&GazeboRotorModel::SimpleGroundRideTest},
			{"lift", 						&GazeboRotorModel::AeroLiftTest},
			{"rotor_speed_car", 			&GazeboRotorModel::AutorotCarTest},
			{"rotor_speed_vertical_wind", 	&GazeboRotorModel::AutorotVerticalWindTest},
			{"rotor_control_test",			&GazeboRotorModel::RotorControlTest}
		};
	};
}

#endif // DEFINE _ROTOR_MODEL_PLUGIN_HH_