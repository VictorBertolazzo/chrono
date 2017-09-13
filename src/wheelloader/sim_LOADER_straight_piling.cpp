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
// Only the loader mechanism is created.
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

// Custom functions 
#include "utilities/ZBarMechanism.h"


using namespace chrono;
// =============================================================================
//	USER SETTINGS
// =============================================================================

// Enumerator used for creating the closing caps of the bucket.
enum BucketSide { LEFT, RIGHT };
// Enumerator used for choosing which tyoe of underlying terrain holds.
enum TerrainType { RIGID_TERRAIN, GRANULAR_TERRAIN};
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
// Specification of the vehicle model
// =============================================================================

// Initial vehicle position and orientation
ChVector<> initLoc(0., 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// =============================================================================
//	SIMULATION PARAMETERS
// =============================================================================

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 20;

// Perform dynamic tuning of number of threads?
bool thread_tuning = false;

// Total simulation duration.
double time_end = 1;

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
const std::string out_dir = "../LOADER_NSC";
const std::string pov_dir = out_dir + "/POVRAY";
// Frame per Second value: povray output setting.
int out_fps = 60;

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
	ChVector<> center(0, 0, 2 * r);
	double num_layers = 10;
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
void OutputCSVdata(){};
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
	vec3 bins = collision::function_Compute_Grid_Resolution((int)num_particles / 8, real3(hdims.x,hdims.y,hdims.z), .1);
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
void SetChassisMovement(ChSystem* system, MyWheelLoader* mywl, std::shared_ptr<ChBody> ground, ChVector<> COG_chassis){

	ChQuaternion<> z2x;
	z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));

	
	auto prism_fix2ch = std::make_shared<ChLinkLockPrismatic>();
	prism_fix2ch->SetName("prismatic_ground2chassis");
	prism_fix2ch->Initialize(mywl->chassis, ground, ChCoordsys<>(COG_chassis, z2x));
	system->AddLink(prism_fix2ch);
	auto lin_fix2ch = std::make_shared<ChLinkLinActuator>();
	prism_fix2ch->SetName("linear_ground2chassis");
	lin_fix2ch->Initialize(mywl->chassis, ground, false, ChCoordsys<>(COG_chassis, z2x), ChCoordsys<>(COG_chassis, z2x));//m2 is the master
	lin_fix2ch->Set_lin_offset(Vlength(VNULL));
	system->AddLink(lin_fix2ch);

	std::vector<TimeSeries> DesiredSpeed;
	ReadPressureFile("../data/WL_DesiredSpeedSmoothed.dat", DesiredSpeed);
	SetSpeedProfile(lin_fix2ch, DesiredSpeed);
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
	// Create system and set specific solver settings
	// --------------------------
	ChSystemParallelNSC* system = new ChSystemParallelNSC;
	SetSolverParameters(system);
	
	// --------------------------
	// Create material and set its settings
	// --------------------------
	auto material_terrain = std::make_shared<ChMaterialSurfaceNSC>();
	UpdateMaterialProperties(material_terrain);
	
	// --------------------------
	// --------------------------
	// Create the ground(terrain)
	auto ground = CreateGround(system);
	// Create the sandpile(spheres pyramid)--Calculate computational time to build it.
	ChVector<> hdims(5.,5.,0.);
	auto sandpile = CreateSandpile(system, material_terrain, hdims);
	//Create the loader(mechanism only, with a fake chassis)
	MyWheelLoader* loader = CreateLoader(system);
	// --------------------------
	// --------------------------

	// --------------------------
	// Set Chassis and Piston motion law.
	// --------------------------
	SetPistonsMovement(system, loader);
	SetChassisMovement(system, loader, ground, ChVector<>(.0, .0, 1.575));

	return 0;
}