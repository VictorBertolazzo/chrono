// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
// Modified: Victor Bertolazzo
// =============================================================================

// Main function for an articulated vehicle (steering applied to the joint
// connecting the front and rear sides).
//
// Driver inputs are decided bmo a flag.
// Keyboard can be used, a DataDriver(throttle,steering,braking), or a PathFollower(path,speed)
//
// The front_side reference frame has Z up, X towards the front of the vehicle,
// and Y pointing to the left.
//
// Rigid Terrain is modeled here, in serial mode Granular Terrain is not taken into account due to computational costs.
//
// FIALA Tire Model is used with rigid terrain, no parameters available for that;
// Rigid Tire hinders steering manouevre with this unusual type of steering joint.
//
// As not usual for a Chrono Vehicle, Driveline is implemented outside the derived class of ChWheeledVehicle, since it is 4WheelDriving but the two axles are located on different bodies:
// both front part(ChWheeledVehicle) and rear part(ChBody) own one axle.
// At the moment the speed and torque are passed with powertrain equally distributing them among the four wheels.Differentials will be thought in future.

// Powertrain is a SimpleMap with only information about fullThrottleMap(not motor idling hence) and six possible gear ratios, 
////	having the real vehicle disconnected FW and RW direction selection and pure gear selection(3 levels).
// The one in the middle refers to idling(gear ration = 1e20).


// WARNING: It needs: path, target_speed, gear_selection related to the same manouevre(Reorganize different manouevres in different folders to make the code thinner)
// =============================================================================

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/generic/Generic_RigidTire.h"
#include "chrono_models/vehicle/generic/Generic_FialaTire.h"

#include "chrono_vehicle/driver/ChPathFollowerDriver.h"


// Chrono utility header files
#include "chrono/utils/ChUtilsGeometry.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/physics/ChBodyEasy.h"

// Articulated Subsystems
#include "subsystems/Articulated_Front.h"
#include "subsystems/Articulated_Rear.h"

#include "subsystems/WL_Driveline4WD.h"
#include "subsystems/WL_SimpleDriveline4WD.h"

#include "subsystems/WL_SimpleMapPowertrain.h"
#include "subsystems/WL_ShaftsPowertrain.h"

#include "subsystems/WL_FialaTire.h"

#include "utilities/UtilityFunctions.h"


using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// =============================================================================

// Initial front_side position
ChVector<> initLoc(0, 0, 1.0);

// Initial front_side orientation
ChQuaternion<> initRot = Q_from_AngZ(0*CH_C_PI / 10);

// Type of tire model (RIGID or FIALA)
TireModelType tire_model = TireModelType::FIALA;

// Rigid terrain dimensions
double terrainHeight = 0;
double terrainLength = 1000.0;  // size in X direction
double terrainWidth = 1000.0;   // size in Y direction

// Simulation step size
double step_size = 0.001;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Input file names for the path-follower driver model
std::string steering_controller_file("generic/driver/SteeringController.json");
std::string speed_controller_file("generic/driver/SpeedController.json");
std::string path_file("paths/straight10km.txt");

// =============================================================================

int main(int argc, char* argv[]) {
	// --------------------------
	// Create the various modules
	// --------------------------

	// Create the front side
	Articulated_Front front_side(false);
	front_side.Initialize(ChCoordsys<>(initLoc, initRot));
	front_side.SetChassisVisualizationType(VisualizationType::PRIMITIVES);
	front_side.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	front_side.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
	front_side.SetWheelVisualizationType(VisualizationType::NONE);

	// Create the rear side
	Articulated_Rear rear_side(std::static_pointer_cast<Articulated_Chassis>(front_side.GetChassis()));
	rear_side.Initialize();
	rear_side.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	rear_side.SetWheelVisualizationType(VisualizationType::NONE);

	// Create the terrain
	RigidTerrain terrain(front_side.GetSystem());
	terrain.SetContactFrictionCoefficient(0.9f);
	terrain.SetContactRestitutionCoefficient(0.01f);
	terrain.SetContactMaterialProperties(2e7f, 0.3f);
	terrain.SetColor(ChColor(0.5f, 0.5f, 1));
	terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
	terrain.Initialize(terrainHeight, terrainLength, terrainWidth);

	// Create and initialize the powertrain system
	// Vehicle is 4WD indeed
	auto driveline = std::make_shared<WL_SimpleDriveline4WD>("driveline");//WL_Driveline4WD works.
	ChSuspensionList suspensions; suspensions.resize(2);
	suspensions[0] = front_side.GetSuspension(0); suspensions[1] =  rear_side.GetSuspension(0);
	std::vector<int> driven_susp_indexes(2);//(ignored)
	driveline->Initialize(front_side.GetChassisBody(), suspensions, driven_susp_indexes);

	// Create and initialize the powertrain system
	// Vehicle is 4WD indeed
	WL_SimpleMapPowertrain powertrain;
	powertrain.Initialize(front_side.GetChassisBody(), driveline->GetDriveshaft());

	// Create the front tires
	std::unique_ptr<ChTire> tire_FL;
	std::unique_ptr<ChTire> tire_FR;
	switch (tire_model) {
	case TireModelType::RIGID:
		tire_FL = std::unique_ptr<ChTire>(new Generic_RigidTire("FL"));
		tire_FR = std::unique_ptr<ChTire>(new Generic_RigidTire("FR"));
		break;
	case TireModelType::FIALA:
		tire_FL = std::unique_ptr<ChTire>(new WL_FialaTire("FL"));
		tire_FR = std::unique_ptr<ChTire>(new WL_FialaTire("FR"));
		break;
	default:
		std::cout << "Tire type not supported!" << std::endl;
		return 1;
	}

	tire_FL->Initialize(front_side.GetWheelBody(FRONT_LEFT), LEFT);
	tire_FR->Initialize(front_side.GetWheelBody(FRONT_RIGHT), RIGHT);
	tire_FL->SetVisualizationType(VisualizationType::PRIMITIVES);
	tire_FR->SetVisualizationType(VisualizationType::PRIMITIVES);

	// Create the rear tires
	std::unique_ptr<ChTire> tire_RL;
	std::unique_ptr<ChTire> tire_RR;
	switch (tire_model) {
	case TireModelType::RIGID:
		tire_RL = std::unique_ptr<ChTire>(new Generic_RigidTire("RL"));
		tire_RR = std::unique_ptr<ChTire>(new Generic_RigidTire("RR"));
		break;
	case TireModelType::FIALA:
		tire_RL = std::unique_ptr<ChTire>(new WL_FialaTire("RL"));
		tire_RR = std::unique_ptr<ChTire>(new WL_FialaTire("RR"));
		break;
	default:
		std::cout << "Tire type not supported!" << std::endl;
		return 1;
	}

	tire_RL->Initialize(rear_side.GetWheelBody(FRONT_LEFT), LEFT);
	tire_RR->Initialize(rear_side.GetWheelBody(FRONT_RIGHT), RIGHT);
	tire_RL->SetVisualizationType(VisualizationType::PRIMITIVES);
	tire_RR->SetVisualizationType(VisualizationType::PRIMITIVES);

	// Initialize Irrlicht app
	ChWheeledVehicleIrrApp app(&front_side, &powertrain, L"Articulated Vehicle Demo");
	app.SetSkyBox();
	app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
	app.SetChaseCamera(trackPoint, 6.0, 0.5);

	app.SetTimestep(step_size);

	app.AssetBindAll();
	app.AssetUpdateAll();

#define USE_PATH_FOLLOWER

#ifdef USE_KEYBOARD
	// Initialize interactive driver
	ChIrrGuiDriver driver(app);
	
	// Set the time response for steering and throttle keyboard inputs.
	// NOTE: this is not exact, since we do not render quite at the specified FPS.
	double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
	double throttle_time = 1.0;  // time to go from 0 to +1
	double braking_time = 0.3;   // time to go from 0 to +1
	driver.SetSteeringDelta(render_step_size / steering_time);
	driver.SetThrottleDelta(render_step_size / throttle_time);
	driver.SetBrakingDelta(render_step_size / braking_time);
#else
    
#ifndef USE_PATH_FOLLOWER

	//ChDataDriver driver(front_side, "C:/Users/victo/Documents/chrono_fork_victor-build/bin/data/WL_Man_NoSteer.dat");
	//ChDataDriver driver(front_side, "C:/Users/victo/Documents/chrono_fork_victor-build/bin/data/WL_Man_RecYaw.dat");
	ChDataDriver driver(front_side, "C:/Users/victo/Documents/chrono_fork_victor-build/bin/data/WL_Man_ddYaw.dat");
#else
	auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
	//auto path = ChBezierCurve::read(vehicle::GetDataFile("paths/WL_Path_First_Segment.dat"));// Uncomment to select real WL path.

	ChPathFollowerDriver driver(front_side, vehicle::GetDataFile(steering_controller_file),
		vehicle::GetDataFile(speed_controller_file), path, "my_path", 3.0);
#endif // !


#endif

	driver.Initialize();


	// Create Selected Gear Time Series-- Test file : WL_SelectedGear.dat
	std::vector<TimeSeries> SelectedGear;
	ReadFile("../data/WL_SelectedGear.dat", SelectedGear);
	auto gear = std::make_shared<ChFunction_Recorder>();
	for (int i = 0; i < SelectedGear.size(); i++){
		gear->AddPoint(SelectedGear[i].mt, SelectedGear[i].mv);
	}

	// Create Desired Speed Time Series -- Test file WL_DesiredSpeed.dat
	std::vector<TimeSeries> DesiredSpeed;
	ReadFile("../data/WL_DesiredSpeedSmoothed.dat", DesiredSpeed);
	auto target_speed = std::make_shared<ChFunction_Recorder>();
	for (int i = 0; i < SelectedGear.size(); i++){
		target_speed->AddPoint(DesiredSpeed[i].mt, DesiredSpeed[i].mv);
	}
	// ---------------
	// Tweak Solver Parameters
	// ---------------
	
	//front_side.GetSystem()->SetSolverType(ChSolver::Type::APGD);

	// ---------------
	// Add an Obstacle to Verify RigidPinnedAxle goodness
	// ---------------

	auto obstacle = std::make_shared<ChBodyEasyBox>(1., 1., 0.125, 1000, true, true);
	//front_side.GetSystem()->AddBody(obstacle);
	obstacle->SetBodyFixed(true);
	obstacle->SetPos(ChVector<>(initLoc.x() + 3., initLoc.y(), 0.));
	obstacle->SetRot(initRot);

	// ---------------
	// SetVerbose
	// ---------------
	app.GetSystem()->GetSolver()->SetVerbose(false);


	// ---------------
	// Simulation loop
	// ---------------

	// Inter-module communication data
	TireForces tire_front_forces(2);
	TireForces tire_rear_forces(2);
	double driveshaft_speed;
	double powertrain_torque;
	double throttle_input;
	double steering_input;
	double braking_input;
	double gear_input;

	// Number of simulation steps between two 3D view render frames
	int render_steps = (int)std::ceil(render_step_size / step_size);

	// Initialize simulation frame counter and simulation time
	int step_number = 0;
	double time = 0;

	while (app.GetDevice()->run()) {
		// Render scene
		if (step_number % render_steps == 0) {
			app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
			app.DrawAll();
			app.EndScene();
		}

		// Collect output data from modules (for inter-module communication)
		throttle_input = driver.GetThrottle();
		steering_input = driver.GetSteering();
		braking_input = driver.GetBraking();
		powertrain_torque = powertrain.GetOutputTorque(); // =0 since ShaftsPowertrain is directly connected to the vehicle's driveline
		gear_input = (int)gear->Get_y(time);

		tire_front_forces[0] = tire_FL->GetTireForce();
		tire_front_forces[1] = tire_FR->GetTireForce();
		tire_rear_forces[0] = tire_RL->GetTireForce();
		tire_rear_forces[1] = tire_RR->GetTireForce();

		driveshaft_speed = driveline->GetDriveshaftSpeed();

		WheelState wheel_FL = front_side.GetWheelState(FRONT_LEFT);
		WheelState wheel_FR = front_side.GetWheelState(FRONT_RIGHT);

		WheelState wheel_RL = rear_side.GetWheelState(FRONT_LEFT);
		WheelState wheel_RR = rear_side.GetWheelState(FRONT_RIGHT);

		// Update modules (process inputs from other modules)
		time = front_side.GetSystem()->GetChTime();

		driver.Synchronize(time);

		terrain.Synchronize(time);

		tire_FL->Synchronize(time, wheel_FL, terrain);
		tire_FR->Synchronize(time, wheel_FR, terrain);
		tire_RL->Synchronize(time, wheel_RL, terrain);
		tire_RR->Synchronize(time, wheel_RR, terrain);

		// Select Direction(it should be called only at changes)
		// I need sth similar to powertrain.SetDriveMode, but my input file has values in range[-1:1:+1],
		// hence a different method should be implemented

		// Select Gear
		powertrain.SetSelectedGear(gear_input);// Should it be a function of time as it is, ge(t), or position,ge(x,y) or speed,ge(v) ?
		powertrain.Synchronize(time, throttle_input, driveshaft_speed);

		driveline->Synchronize(powertrain_torque);//Is the time info needed?

		front_side.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_front_forces);
		rear_side.Synchronize(time, steering_input, braking_input, tire_rear_forces);

#ifdef USE_KEYBOARD
		app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);
#else
		app.Synchronize("Message HUD", steering_input, throttle_input, braking_input);
#endif


#ifdef USE_PATH_FOLLOWER
		driver.SetDesiredSpeed(target_speed->Get_y(time + 0.5));//Change target speed for successive Advance method.In this way speed is a function of time , v(t).
																// needed to translate in a function of position v(x,y), this is a path follower not a traj follower.
														  // "look ahead" speed
#endif
		// Advance simulation for one timestep for all modules
		driver.Advance(step_size);

		terrain.Advance(step_size);

		tire_FL->Advance(step_size);
		tire_FR->Advance(step_size);
		tire_RL->Advance(step_size);
		tire_RR->Advance(step_size);

		powertrain.Advance(step_size);

		front_side.Advance(step_size);

		app.Advance(step_size);

		// Increment frame number
		step_number++;
	}

	return 0;
}
