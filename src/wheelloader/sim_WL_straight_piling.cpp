// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Victor Bertolazzo
// =============================================================================
//
// ChronoParallel program simulating a piling manouevre . 
// The WHOLE loader vehicle is created.
//
// 
// The global reference frame has Z up.
//
// If available, OpenGL is used for run-time rendering. Otherwise, the
// simulation is carried out for a pre-defined duration and output files are
// generated for post-processing with POV-Ray.
// =============================================================================

// STL Headers
#include <iostream>
#include <memory>
#include <ctime>

// Chrono::Engine header files
#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChStream.h"

// Chrono utilities header files
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

// Chrono collision header files
#include "chrono/collision/ChCCollisionUtils.h"

// Chrono::Parallel header files
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/collision/ChBroadphaseUtils.h"

// Chrono::Vehicle header files
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChDataDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
	// Driver header files
#include "chrono_vehicle/driver/ChPathFollowerDriver.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

// Custom functions 
#include "utilities/ZBarMechanism.h"


// Articulated Subsystems
#include "subsystems/Articulated_Front.h"
#include "subsystems/Articulated_Rear.h"

#include "subsystems/WL_Driveline4WD.h"
#include "subsystems/WL_SimpleDriveline4WD.h"

#include "subsystems/WL_SimpleMapPowertrain.h"
#include "subsystems/WL_ShaftsPowertrain.h"

#include "subsystems/WL_FialaTire.h"
#include "chrono_models/vehicle/generic/Generic_RigidTire.h"


using namespace chrono;
using namespace chrono::collision;
using namespace chrono::vehicle;
using namespace chrono::vehicle::generic;

// =============================================================================
//	USER SETTINGS
// =============================================================================

// Enumerator used for creating the closing caps of the bucket.
enum BucketSide { LEFT, RIGHT };
// Enumerator used for choosing which tyoe of underlying terrain holds.
enum TerrainType { RIGID_TERRAIN, GRANULAR_TERRAIN };
// Type of terrain
TerrainType terrain_type = RIGID_TERRAIN;

// =============================================================================
// Parameters for granular material
// =============================================================================
int Id_g = 100;
double r_g = 1e-2;
double rho_g = 2500;
double coh_pressure = 3e4;
float mu_g = 0.9f;

double vol_g = (4.0 / 3) * CH_C_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * r_g * r_g * ChVector<>(1, 1, 1);
double coh_force = CH_C_PI * r_g * r_g * coh_pressure;

// =============================================================================
// Specification of the terrain model
// =============================================================================
double terrainHeight = 0;
double terrainLength = 1000.0;  // size in X direction
double terrainWidth = 1000.0;   // size in Y direction

// =============================================================================
// Specification of the vehicle model
// =============================================================================

// Initial vehicle position and orientation
ChVector<> initLoc(0., 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Type of tire model (RIGID or FIALA)
TireModelType tire_model = TireModelType::FIALA;

// =============================================================================
//	SIMULATION PARAMETERS
// =============================================================================

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Total simulation duration.
double time_end = 500;

// Solver parameters
double time_step = 1e-3;
double tolerance = 1e-5;

uint max_iteration_bilateral = 1000;
uint max_iteration_normal = 0;
uint max_iteration_sliding = 100;
uint max_iteration_spinning = 0;

float contact_recovery_speed = 1;

// =============================================================================
//	OUTPUT
// =============================================================================

// Boolean choice of povray output.
bool povray_output = true;
// Output and Povray Directories strings.
const std::string out_dir = "../WL_NSC";
const std::string pov_dir = out_dir + "/POVRAY";
// Frame per Second value: povray output setting.
int out_fps = 60;

// =============================================================================
//	PATH FOLLOWER FILES
// =============================================================================

// Input file names for the path-follower driver model
std::string steering_controller_file("generic/driver/SteeringController.json");
std::string speed_controller_file("generic/driver/SpeedController.json");
std::string path_file("paths/straight10km.txt");

// -----------------------------------------------------------------------------
// Generate postprocessing output with current system state.
// -----------------------------------------------------------------------------
void OutputPOVRayData(ChSystemParallel* sys, int out_frame, double time) {
	char filename[100];
	sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), out_frame);
	utils::WriteShapesPovray(sys, filename);
	std::cout << "time = " << time << std::flush << std::endl;
}

// =============================================================================
//	ADDITIONAL FEATURES
// =============================================================================
// Time Series structure: utility to grab two columns file values(t_k,f(t_k)) extensively used in this project.
struct TimeSeries {
	TimeSeries() {}
	TimeSeries(float t, float v)
		: mt(t), mv(v) {}
	float mt; float mv;
};
// Read Pressure function : converts a text file in a time series vector. 
void ReadPressureFile(const std::string& filename, std::vector<TimeSeries>& profile) {
	std::ifstream ifile(filename.c_str());
	std::string line;

	while (std::getline(ifile, line)) {
		std::istringstream iss(line);
		float ttime, vvalue;
		iss >> ttime >> vvalue;
		if (iss.fail())
			break;
		profile.push_back(TimeSeries(ttime, vvalue));
	}
	ifile.close();



}
// Impose forward velocity profile, and set displacement law consequently
void SetSpeedProfile(std::shared_ptr<ChLinkLinActuator> act, std::vector<TimeSeries> speed){
	auto chass_traj = std::make_shared<ChFunction_Recorder>();

	chass_traj->AddPoint(0., 0.);
	for (int i = 1; i < speed.size(); i++){
		double pos = chass_traj->Get_y(speed[i - 1].mt) + speed[i].mv * (speed[i].mt - speed[i - 1].mt);
		chass_traj->AddPoint(speed[i].mt, pos);
		//target_speed->AddPoint(DesiredSpeed[i].mt, DesiredSpeed[i].mv);
	}

	act->Set_dist_funct(chass_traj);

}
// =============================================================================
//	PLAYERS CREATION FUNCTIONS
// =============================================================================
std::shared_ptr<ChBody> CreateGround(ChSystem* system){
	auto ground = std::shared_ptr<ChBody>(system->NewBody());
	system->Add(ground);
	ground->SetBodyFixed(true);
	ground->SetIdentifier(-1);
	ground->SetName("ground");
	ground->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddBoxGeometry(ground.get(), ChVector<>(10., 10., 3.0), ChVector<>(0, 0, -3.0),
		ChQuaternion<>(1, 0, 0, 0), true);
	ground->GetCollisionModel()->BuildModel();
	ground->SetCollide(true);

	return ground;
};
utils::Generator CreateSandpile(ChSystem* system, std::shared_ptr<ChMaterialSurface> material_terrain, ChVector<> hdims){
	// Create a particle generator and a mixture entirely made out of spheres
	utils::Generator gen(system);
	std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1);
	m1->setDefaultMaterial(material_terrain);
	m1->setDefaultDensity(rho_g);
	m1->setDefaultSize(r_g);
	gen.setBodyIdentifier(Id_g);
	double r = r_g * 1.01;
	ChVector<> center(7.5, 0, 2 * r);
	double num_layers = 50;
	for (int il = 0; il < num_layers; il++) {
		gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
		center.z() += 2 * r;
		hdims.x() -= 2 * r;
		hdims.y() -= 2 * r;
		std::cout << center.z() << std::endl;
		if (center.z() > 1.){ break; }
	}
	return gen;
};
MyWheelLoader* CreateLoader(ChSystem* system){
	MyWheelLoader* mywl = new MyWheelLoader(*system);
	return mywl;
};
// =============================================================================
//	OTHER SETTING FUNCTIONS
// =============================================================================
void OutputCSVdata(ChSystemParallel* sys, int out_frame, double time, utils::CSV_writer csv){

	csv.write_to_file(out_dir + "/output.dat");

};
void SetSolverParameters(ChSystemParallel* system){
	// Set number of threads
	int max_threads = CHOMPfunctions::GetNumProcs();
	if (threads > max_threads)
		threads = max_threads;
	system->SetParallelThreadNumber(threads);
	CHOMPfunctions::SetNumThreads(threads);
	std::cout << "Using " << threads << " threads" << std::endl;

	system->GetSettings()->perform_thread_tuning = thread_tuning;

	// Set solver parameters
	system->GetSettings()->solver.use_full_inertia_tensor = false;
	system->GetSettings()->solver.tolerance = tolerance;
	system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
	system->GetSettings()->solver.max_iteration_bilateral = max_iteration_bilateral;
	system->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
	system->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
	system->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
	system->GetSettings()->solver.alpha = 0;
	system->GetSettings()->solver.contact_recovery_speed = -1;
	system->GetSettings()->solver.bilateral_clamp_speed = 1e8;
	system->GetSettings()->solver.solver_type = SolverType::BB;
	// Set collision parameters
	system->GetSettings()->collision.collision_envelope = 0.1 * r_g;
	system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
	system->GetSettings()->collision.collision_envelope = 0.001;


};
void SetBroadphaseParameters(ChSystemParallel* system, double num_particles, vec3 hdims){
	// Set broadphase parameters
	system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);
	int factor = 2;
	int binsX = (int)std::ceil(hdims.x / r_g) / factor;
	int binsY = (int)std::ceil(hdims.y / r_g) / factor;
	int binsZ = 1;
	system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
	vec3 bins = collision::function_Compute_Grid_Resolution((int)num_particles / 8, real3(hdims.x, hdims.y, hdims.z), .1);
	std::cout << "broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;

}
void UpdateMaterialProperties(std::shared_ptr<ChMaterialSurfaceNSC> mat_ter){
	mat_ter->SetFriction(mu_g);
	mat_ter->SetRestitution(0.);
	mat_ter->SetCohesion((float)coh_force);
	mat_ter->SetSpinningFriction((float)r_g / 100);
	mat_ter->SetRollingFriction((float)r_g / 100);

}
// =============================================================================
//	IMPOSED MOTION FUNCTIONS
// =============================================================================
void SetPistonsMovement(ChSystem* system, MyWheelLoader* mywl){
	std::vector<TimeSeries> ReadTiltDisplacement;
	ReadPressureFile("../data/TiltDisplacement.dat", ReadTiltDisplacement);
	auto tdisplacement = std::make_shared<ChFunction_Recorder>();
	for (int i = 0; i < ReadTiltDisplacement.size(); i++){
		tdisplacement->AddPoint(ReadTiltDisplacement[i].mt, 0.1*ReadTiltDisplacement[i].mv);// 2 pistons, data in [bar]
	}

	mywl->SetPistonTiltImposedMotion(tdisplacement);




	std::vector<TimeSeries> ReadLiftDisplacement;
	ReadPressureFile("../data/LiftDisplacement.dat", ReadLiftDisplacement);
	auto ldisplacement = std::make_shared<ChFunction_Recorder>();
	for (int i = 0; i < ReadLiftDisplacement.size(); i++){
		ldisplacement->AddPoint(ReadLiftDisplacement[i].mt, 0.1*ReadLiftDisplacement[i].mv);// 2 pistons, data in [bar]
	}

	mywl->SetPistonLiftImposedMotion(ldisplacement);
}

// =============================================================================
// =============================================================================
//	MAIN
// =============================================================================
// =============================================================================
int main(int argc, char* argv[]){

	// --------------------------
	// Create output directories.
	// --------------------------
	if (povray_output) {
		if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
			std::cout << "Error creating directory " << out_dir << std::endl;
			return 1;
		}
		if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
			std::cout << "Error creating directory " << pov_dir << std::endl;
			return 1;
		}
	}

	// --------------------------
	// Initialize csv output file.
	// --------------------------
	utils::CSV_writer csv("\t");
	csv.stream().setf(std::ios::scientific | std::ios::showpos);
	csv.stream().precision(6);

	// --------------------------
	// Create system and set specific solver settings
	// --------------------------
	ChSystemParallelNSC* system = new ChSystemParallelNSC;
	system->Set_G_acc(ChVector<>(0, 0, -9.81));
	SetSolverParameters(system);

	// --------------------------
	// Create material and set its settings
	// --------------------------
	auto material_terrain = std::make_shared<ChMaterialSurfaceNSC>();
	UpdateMaterialProperties(material_terrain);

	// --------------------------
	// --------------------------
	// Create the ground(terrain)
			//auto ground = CreateGround(system);
	// Create the sandpile(spheres pyramid)--Calculate computational time to build it.
	ChVector<> hdims(2., 2., 0.);
	int start_s = clock();
			//auto sandpile = CreateSandpile(system, material_terrain, hdims);
	int stop_s = clock();
	std::cout << "Sandpile creation computational time: " << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << std::endl;
			//std::cout << "Number of created particles: " << sandpile.getTotalNumBodies() << std::endl;
	//Create the loader(mechanism only, with a fake chassis)
			//MyWheelLoader* loader = CreateLoader(system);
	// Create the vehicle(the whole wheel loader except the mechanism)
	// --------------------------
	// --------------------------
	// Create the front side
	Articulated_Front front_side(system);// it avoids to create two concurrent systems.
	front_side.Initialize(ChCoordsys<>(initLoc, initRot));
	front_side.SetChassisVisualizationType(VisualizationType::MESH);
	front_side.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	front_side.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
	front_side.SetWheelVisualizationType(VisualizationType::NONE);

	// Create the rear side
	Articulated_Rear rear_side(std::static_pointer_cast<Articulated_Chassis>(front_side.GetChassis()));
	rear_side.Initialize();
	rear_side.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
	rear_side.SetWheelVisualizationType(VisualizationType::NONE);

	// Create the terrain
	RigidTerrain terrain(system);
	terrain.SetContactFrictionCoefficient(0.9f);
	terrain.SetContactRestitutionCoefficient(0.01f);
	terrain.SetContactMaterialProperties(2e7f, 0.3f);
	terrain.SetColor(ChColor(0.5f, 0.5f, 0.5f));
	terrain.SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
	terrain.Initialize(terrainHeight, terrainLength, terrainWidth);
	//terrain.GetGroundBody()->GetCollisionModel()->ClearModel();
	//auto ground = std::shared_ptr<ChBody>(terrain.GetGroundBody());
	//utils::AddBoxGeometry(ground.get(), ChVector<>(terrainLength, terrainWidth, .25), ChVector<>(0, 0, -.25),ChQuaternion<>(1, 0, 0, 0), true);
	//ground->GetCollisionModel()->BuildModel();

	// Create and initialize the powertrain system
	// Vehicle is 4WD indeed
	auto driveline = std::make_shared<WL_SimpleDriveline4WD>("WL_driveline");//WL_Driveline4WD works.
	ChSuspensionList suspensions; suspensions.resize(2);
	suspensions[0] = front_side.GetSuspension(0); suspensions[1] = rear_side.GetSuspension(0);
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

	tire_FL->Initialize(front_side.GetWheelBody(FRONT_LEFT), VehicleSide::LEFT);
	tire_FR->Initialize(front_side.GetWheelBody(FRONT_RIGHT), VehicleSide::RIGHT);
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

	tire_RL->Initialize(rear_side.GetWheelBody(FRONT_LEFT), VehicleSide::LEFT);
	tire_RR->Initialize(rear_side.GetWheelBody(FRONT_RIGHT), VehicleSide::RIGHT);
	tire_RL->SetVisualizationType(VisualizationType::PRIMITIVES);
	tire_RR->SetVisualizationType(VisualizationType::PRIMITIVES);
	// --------------------------
	// Attach the mechanism to the vehicle.
	// --------------------------

	// --------------------------
	// Set Chassis and Piston motion law.
	// --------------------------
		//SetPistonsMovement(system, loader);

	// --------------------------
	// Set Path Follower settings.
	// --------------------------
	auto path = ChBezierCurve::read(vehicle::GetDataFile(path_file));
	//auto path = ChBezierCurve::read(vehicle::GetDataFile("paths/WL_Path_First_Segment.dat"));// Uncomment to select real WL path.
	ChPathFollowerDriver driver(front_side, vehicle::GetDataFile(steering_controller_file),	vehicle::GetDataFile(speed_controller_file), path, "my_path", 3.0);
	//ChDataDriver driver(front_side, "C:/Users/victo/Documents/chrono_fork_victor-build/bin/data/WL_Man_NoSteer.dat");
	driver.Initialize();

	// --------------------------
	// Create Selected Gear Time Series-- Test file : WL_SelectedGear.dat
	// --------------------------
	std::vector<TimeSeries> SelectedGear;
	ReadPressureFile("../data/WL_SelectedGear.dat", SelectedGear);
	auto gear = std::make_shared<ChFunction_Recorder>();
	for (int i = 0; i < SelectedGear.size(); i++){
		gear->AddPoint(SelectedGear[i].mt, SelectedGear[i].mv);
	}

	// --------------------------
	// Create Desired Speed Time Series -- Test file WL_DesiredSpeed.dat
	// --------------------------
	std::vector<TimeSeries> DesiredSpeed;
	ReadPressureFile("../data/WL_DesiredSpeedSmoothed.dat", DesiredSpeed);
	auto target_speed = std::make_shared<ChFunction_Recorder>();
	for (int i = 0; i < DesiredSpeed.size(); i++){
		target_speed->AddPoint(DesiredSpeed[i].mt, DesiredSpeed[i].mv);
	}

	// --------------------------
	// Initialize OpenGL window..
	// --------------------------
#ifdef CHRONO_OPENGL
	opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
	gl_window.Initialize(1280, 720, "Wheel Loader-Piling Manouevre", system);
	gl_window.SetCamera(ChVector<>(0, -15, 0), ChVector<>(0.0, 0, 0), ChVector<>(0, 0, 1));
	gl_window.SetRenderMode(opengl::WIREFRAME);
#endif



	// --------------------------
	// Run simulation for specified time
	// --------------------------
	int num_steps = std::ceil(time_end / time_step);
	int out_steps = std::ceil((1 / time_step) / out_fps);
	int out_frame = 0;
	double time = 0;

	// --------------------------
	// Inter-module communication data variables.
	// --------------------------
	TireForces tire_front_forces(2);
	TireForces tire_rear_forces(2);
	double driveshaft_speed;
	double powertrain_torque;
	double throttle_input;
	double steering_input;
	double braking_input;
	double gear_input;
	// --------------------------
	// Output Loader data variables.
	// --------------------------
	double pos_lift; double pos_tilt;
	double ang_lift;double ang_tilt;
	double fp_lift; double fp_tilt;


	// --------------------------
	// Print System Hierarchy.
	// --------------------------
	system->ShowHierarchy(GetLog());

	// --------------------------
	// Simulation Loop--The Loop is the standard for Chrono::Vehicle demos.
	// --------------------------
	// --------------------------
	// --------------------------
	// --------------------------
	for (int i = 0; i < num_steps; i++) {

		// --------------------------
		// Collect output data from modules (for inter-module communication)
		// --------------------------
		throttle_input = driver.GetThrottle();
		steering_input = driver.GetSteering();
		braking_input = driver.GetBraking();
		powertrain_torque = powertrain.GetOutputTorque();
			gear_input = (int)gear->Get_y(time);

		tire_front_forces[0] = tire_FL->GetTireForce();tire_front_forces[1] = tire_FR->GetTireForce();
		tire_rear_forces[0] = tire_RL->GetTireForce();tire_rear_forces[1] = tire_RR->GetTireForce();

		driveshaft_speed = driveline->GetDriveshaftSpeed();

		WheelState wheel_FL = front_side.GetWheelState(FRONT_LEFT);WheelState wheel_FR = front_side.GetWheelState(FRONT_RIGHT);
		WheelState wheel_RL = rear_side.GetWheelState(FRONT_LEFT);WheelState wheel_RR = rear_side.GetWheelState(FRONT_RIGHT);

		// --------------------------
		// Collect output data from wheel loader module.
		// --------------------------

		//pos_lift = loader->lin_ch2lift->Get_dist_funct()->Get_y(time) + loader->lin_ch2lift->Get_lin_offset();
		//pos_tilt = loader->lin_lift2rod->Get_dist_funct()->Get_y(time) + loader->lin_ch2lift->Get_lin_offset();

		//ang_lift = loader->GetLiftArmPitchAngle();
		//ang_tilt = loader->GetTiltBucketPitchAngle();

		//fp_lift = loader->GetReactLiftForce();
		//fp_tilt = loader->GetReactTiltForce();
		//
		// --------------------------
		// Update modules (process inputs from other modules)--Synchronize method call.
		// --------------------------
		time = front_side.GetSystem()->GetChTime();
		driver.Synchronize(time);
		terrain.Synchronize(time);
		tire_FL->Synchronize(time, wheel_FL, terrain);tire_FR->Synchronize(time, wheel_FR, terrain);
		tire_RL->Synchronize(time, wheel_RL, terrain);tire_RR->Synchronize(time, wheel_RR, terrain);

			// Select Direction(it should be called only at changes)

			// Select Gear
			powertrain.SetSelectedGear(gear_input);// Should it be a function of time as it is, ge(t), or position,ge(x,y) or speed,ge(v) ?
		powertrain.Synchronize(time, throttle_input, driveshaft_speed);

		driveline->Synchronize(powertrain_torque);//Is the time info needed?no

		front_side.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_front_forces);// powertrain_torque will be ignored since driveline is outside of ArticulatedVehicle(function overriding)
		rear_side.Synchronize(time, steering_input, braking_input, tire_rear_forces);
			driver.SetDesiredSpeed(target_speed->Get_y(time + 0.5));//Change target speed for successive Advance method.In this way speed is a function of time , v(t).// needed to translate in a function of position v(x,y), this is a path follower not a traj follower.// "look ahead" speed

		// --------------------------
		// Advance simulation for one timestep for all modules
		// --------------------------
		driver.Advance(time_step);
		terrain.Advance(time_step);
		tire_FL->Advance(time_step); tire_FR->Advance(time_step);
		tire_RL->Advance(time_step); tire_RR->Advance(time_step);
		powertrain.Advance(time_step);

		// --------------------------
		// Perform time step integration.
		// --------------------------

		//system->DoStepDynamics(time_step);
		front_side.Advance(time_step);// This call contains "system->DoStepDynamics(time_step)"

		// --------------------------
		// Save output data from modules on CSV writer.
		// --------------------------
		csv << pos_lift << pos_tilt;
		csv << ang_lift << ang_tilt;
		csv << fp_lift << fp_tilt;
		csv << std::endl;


		// --------------------------
		// Write output info on CSV file.
		// --------------------------
		if (i % out_steps == 0) {
			OutputPOVRayData(system, out_frame, time);
			out_frame++;

			csv.write_to_file(out_dir + "/output.dat");
		}
		time += time_step;

// Render the frame
#ifdef CHRONO_OPENGL
		if (gl_window.Active())
		{gl_window.Render();}
		else
			break;
#endif
	}

	return 0;
}