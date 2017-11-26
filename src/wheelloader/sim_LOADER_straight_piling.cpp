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

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

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
double r_g = 5e-2;
double rho_g = 2500;
double coh_pressure = 0;
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
double time_end = 8.;
// Heap Height.
double height = 4.0;

// Solver parameters
double time_step = 1e-3;
double tolerance = 1e-5;

uint max_iteration_bilateral = 50;
uint max_iteration_normal = 5;
uint max_iteration_sliding = 50;
uint max_iteration_spinning = 50;

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
	// Side Wall
	utils::AddBoxGeometry(ground.get(), ChVector<>(0.375, 4.0, 2.0),
		ChVector<>(12.5, 0, 2.0 - .25), ChQuaternion<>(1, 0, 0, 0), true);//side wall
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
	ChVector<> center(8.0, 0, 2 * r);
	double num_layers = 50;
	for (int il = 0; il < num_layers; il++) {
		gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
		center.z() += 2 * r;
		hdims.x() -= 2 * r;
		hdims.y() -= 2 * r;
		center.x() += 2 * r;
		std::cout << center.z() << std::endl;
		if (center.z() > height){ break; }
	}
	return gen;
};
//MyWheelLoader* CreateLoader(ChSystem* system){
//	MyWheelLoader* mywl = new MyWheelLoader(*system);
//	return mywl;
//};


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
	int binsZ = (int)std::ceil(height / r_g) / factor;
	system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);
	vec3 bins = collision::function_Compute_Grid_Resolution((int)num_particles / 8, real3(hdims.x,hdims.y,hdims.z), .1);
	std::cout << "broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;
	//system->GetSettings()->collision.bins_per_axis = vec3(bins.x, bins.y, bins.z);


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
		tdisplacement->AddPoint(ReadTiltDisplacement[i].mt, ReadTiltDisplacement[i].mv);// 2 pistons, data in [bar]
	}

	mywl->SetPistonTiltImposedMotion(tdisplacement);




	std::vector<TimeSeries> ReadLiftDisplacement;
	ReadPressureFile("../data/LiftDisplacement.dat", ReadLiftDisplacement);
	auto ldisplacement = std::make_shared<ChFunction_Recorder>();
	for (int i = 0; i < ReadLiftDisplacement.size(); i++){
		ldisplacement->AddPoint(ReadLiftDisplacement[i].mt, ReadLiftDisplacement[i].mv);// 2 pistons, data in [bar]
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
	ReadPressureFile("../data/loadingSpeed.dat", DesiredSpeed);
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
	auto ground = CreateGround(system);
	// Create the sandpile(spheres pyramid)--Calculate computational time to build it.
	ChVector<> hdims(4., 4., 0.);
	int start_s = clock();
		auto sandpile = CreateSandpile(system, material_terrain, hdims);
	int stop_s = clock();
	std::cout << "Sandpile creation computational time: " << (stop_s - start_s) / double(CLOCKS_PER_SEC) * 1000 << std::endl;
		std::cout << "Number of created particles: " << sandpile.getTotalNumBodies() << std::endl;
	
	// Setting broadphase grid partition.
		SetBroadphaseParameters(system, sandpile.getTotalNumBodies(), vec3(hdims.x(), hdims.y(), hdims.z()));

	//Create the loader(mechanism only, with a fake chassis)
		MyWheelLoader* loader = new MyWheelLoader(*system, ChCoordsys<>(ChVector<>(0.,0.,0.), QUNIT));	// --------------------------
	// --------------------------

	// --------------------------
	// Set Chassis and Piston motion law.
	// --------------------------
	SetPistonsMovement(system, loader);
	SetChassisMovement(system, loader, ground, ChVector<>(.0, .0, 1.575));

	// --------------------------
	// Initialize OpenGL window..
	// --------------------------
#ifdef CHRONO_OPENGL
	opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
	gl_window.Initialize(1280, 720, "Loader-Piling", system);
	gl_window.SetCamera(ChVector<>(0, -10, 0), ChVector<>(7.5, 0, 0), ChVector<>(0, 0, 1));
	gl_window.SetRenderMode(opengl::WIREFRAME);
#endif



	// Run simulation for specified time
	int num_steps = std::ceil(time_end / time_step);
	int out_steps = std::ceil((1 / time_step) / out_fps);
	int out_frame = 0;
	double time = 0;


	// Simulation Loop
	for (int i = 0; i < num_steps; i++) {

		// Collect output data from modules.
		ChVector<> pos_CG = loader->chassis->GetPos();
		ChVector<> vel_CG = loader->chassis->GetPos_dt();
		vel_CG = loader->chassis->GetCoord().TransformDirectionParentToLocal(vel_CG);

		double pos_lift = loader->lin_ch2lift->Get_dist_funct()->Get_y(time) + loader->lin_ch2lift->Get_lin_offset();
		double pos_tilt = loader->lin_lift2rod->Get_dist_funct()->Get_y(time) + loader->lin_ch2lift->Get_lin_offset();

		double ang_lift = loader->GetLiftArmPitchAngle();
		double ang_tilt = loader->GetTiltBucketPitchAngle();

		double fp_lift = loader->GetReactLiftForce();
		double fp_tilt = loader->GetReactTiltForce();

		// Save output data from modules on CSV writer.I don't save time since time step is fixed.
		csv << pos_CG.x() << pos_CG.y() << pos_CG.z();
		csv << vel_CG.x() << vel_CG.y() << vel_CG.z();
		csv << pos_lift << pos_tilt;
		csv << ang_lift << ang_tilt;
		csv << fp_lift << fp_tilt;
		csv << std::endl;


		if (i % out_steps == 0) {
			OutputPOVRayData(system, out_frame, time);
			out_frame++;

			csv.write_to_file(out_dir + "/output.dat");

		}
		system->DoStepDynamics(time_step);
		//std::cout << "Time : " << time << std::endl;
		time += time_step;
		


#ifdef CHRONO_OPENGL
		if (gl_window.Active())
		{
			gl_window.Render();
			//std::cin.get();
		}
		else
			break;
#endif
	}

	return 0;
}