// Victor Bertolazzo
// Building a sandpile and performing piling operation with different particles size.
// ================================================================================================================

#include <cmath>
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <valarray>
#include <vector>

#include "chrono/ChConfig.h"
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChTimer.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono/collision/ChCCollisionUtils.h"

#include "chrono_parallel/collision/ChBroadphaseUtils.h"
#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"

#include "chrono_postprocess/ChGnuPlot.h"




#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif

#include "utilities/UtilityFunctions.h"

enum TestType {LAYER, FUNNEL, DROP, CASCADE};
TestType workcase = TestType::LAYER;
using namespace chrono;
using namespace postprocess;

// Utility Functions
enum BucketSide { LEFT, RIGHT };
struct Points {
	Points() {}
	Points(float x, float y, float z)
		: mx(x), my(y), mz(z) {}
	float mx; float my; float mz;
};

// --------------------------------------------------------------------------
void ReadFile(const std::string& filename, std::vector<Points>& profile) {
	std::ifstream ifile(filename.c_str());
	std::string line;

	while (std::getline(ifile, line)) {
		std::istringstream iss(line);
		float xpos, ypos, zpos;
		iss >> xpos >> ypos >> zpos;
		if (iss.fail())
			break;
		profile.push_back(Points(xpos, ypos, zpos));
	}
	ifile.close();



}
void AddBucketHull(std::vector<Points> p_ext, std::vector<Points> p_int, std::shared_ptr<ChBody> bucket) {

	for (int iter = 0; iter < p_ext.size() - 1; iter++) {
		std::vector<ChVector<double>> cloud;
		double width = 1.3;// or halve an input

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my - width, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my - width, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter + 1].mx, p_ext[iter + 1].my - width, p_ext[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter].mx, p_ext[iter].my - width, p_ext[iter].mz));

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter + 1].mx, p_ext[iter + 1].my + width, p_ext[iter + 1].mz));
		cloud.push_back(ChVector<>(p_ext[iter].mx, p_ext[iter].my + width, p_ext[iter].mz));



		bucket->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
		//bucket->GetCollisionModel()->BuildModel();

		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		//bucket->AddAsset(shape);

		//bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}

}
void AddCapsHulls(std::vector<Points> p_int, BucketSide side, std::shared_ptr<ChBody> bucket) {


	for (int iter = 0; iter < p_int.size() - 1; iter++) {

		std::vector<ChVector<double>> cloud;
		double width;
		switch (side)
		{
		case LEFT:
			width = +1.3;
			break;
		case RIGHT:
			width = -1.3;
			break;
		default:
			width = .0;
			break;
		}


		double th = .05;

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width - th, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width - th, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(.70, width - th, .25));

		cloud.push_back(ChVector<>(p_int[iter].mx, p_int[iter].my + width + th, p_int[iter].mz));
		cloud.push_back(ChVector<>(p_int[iter + 1].mx, p_int[iter + 1].my + width + th, p_int[iter + 1].mz));
		cloud.push_back(ChVector<>(.70, width + th, .25));

		bucket->GetCollisionModel()->AddConvexHull(cloud, ChVector<>(0, 0, 0), QUNIT);
		//bucket->GetCollisionModel()->BuildModel();

		auto shape = std::make_shared<ChTriangleMeshShape>();
		collision::ChConvexHullLibraryWrapper lh;
		lh.ComputeHull(cloud, shape->GetMesh());
		//bucket->AddAsset(shape);

		//bucket->AddAsset(std::make_shared<ChColorAsset>(0.5f, 0.0f, 0.0f));
	}
}

const std::string out_dir = "../";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string flatten = out_dir + "/funnel_DEMP_c10";
const std::string flatten_track = "funnel_DEMP_c10";

int out_fps = 60;

using std::cout;
using std::endl;

int num_threads = 40;
ChMaterialSurface::ContactMethod method = ChMaterialSurface::NSC;
// PovRay Output
bool povray_output = false;
// Material
bool use_mat_properties = true;
// Render 
bool render = false;
// Tracking Granule
bool track_granule = false;
// Roughness
bool roughness = false;
// Broad vs Narr
bool broad_narr = true;
// Tracking Flattening
bool track_flatten = true;
// --------------------------------------------------------------------------
double radius_g = 0.05;
// --------------------------------------------------------------------------
double r = 1.01 * radius_g;

// Container dimensions
double hdimX = 5.;
double hdimY = 5.;
double hdimZ = 0.5;
double hthick = 0.25;

// Granular material properties
int Id_g = 10000;
double rho_g = 2500;
double vol_g = (4.0 / 3) * CH_C_PI * radius_g * radius_g * radius_g;
double mass_g = rho_g * vol_g;
ChVector<> inertia_g = 0.4 * mass_g * radius_g * radius_g * ChVector<>(1, 1, 1);
int num_layers = 40;

// Terrain contact properties---Default Ones are commented out.
float friction_terrain = 1.0f;// 
float restitution_terrain = 0.0f;
float coh_pressure_terrain = 10.f;// 0e3f;
float coh_force_terrain = (float)(CH_C_PI * radius_g * radius_g) * coh_pressure_terrain;
float rolling_friction = 0.1 * radius_g;//increasing rolling_friction results in lowering time_step, WHY?

//// Number of bins for broad-phase
int factor = 2;
int binsX = 10;
int binsY = 10;
int binsZ = 10;

// -------------------------
double Ra_d = 5.0*radius_g;//Distance from centers of particles.
double Ra_r = 3.0*radius_g;//Default Size of particles.

const std::string san_dir = out_dir + "/SCALABILITY";


// ---------------------------FUNCTIONS--------------------------------
    // All functions are in UtilityFunctions.h file.
// ---------------------------FUNCTIONS--------------------------------
int main(int argc, char** argv) {



	if (ChFileutils::MakeDirectory(san_dir.c_str()) < 0) {
		std::cout << "Error creating directory " << san_dir << std::endl;
		return 1;
	}

	///////////////////////////////////////////////////////////////Constructor Utilities
	std::vector<Points> p_ext;
	std::vector<Points> p_int;
	const std::string out_dir = "../";
	const std::string& ext_profile = out_dir + "data/ext_profile.txt";
	const std::string& int_profile = out_dir + "data/int_profile.txt";
	ReadFile(ext_profile, p_ext);
	ReadFile(int_profile, p_int);
	///////////////////////////////////////////////////////////////Constructor Utilities

	uint max_iteration_normal = 50;
	uint max_iteration_sliding = 50;
	uint max_iteration_spinning = 50;
	uint max_iteration_bilateral = 50;

	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}
	std::cout << "Requested number of threads: " << num_threads << std::endl;
	// Model parameters
	std::cout << "Broad-phase bins: " << binsX << " x " << binsY << " x " << binsZ << std::endl;
	// Create the parallel system
	chrono::ChSystemParallel* system;
	// Create system and set method-specific solver settings
	switch (method) {
	case ChMaterialSurface::NSC: {
		ChSystemParallelNSC* sys = new ChSystemParallelNSC;
		sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
		sys->GetSettings()->solver.max_iteration_normal = max_iteration_normal;
		sys->GetSettings()->solver.max_iteration_sliding = max_iteration_sliding;
		sys->GetSettings()->solver.max_iteration_spinning = max_iteration_spinning;
		sys->GetSettings()->solver.alpha = 0;
		sys->GetSettings()->solver.contact_recovery_speed = .1;
		sys->GetSettings()->collision.collision_envelope = 0.05 * radius_g;
		sys->ChangeSolverType(SolverType::APGD);
		system = sys;

		break;
	}
	}

	system->Set_G_acc(ChVector<>(0, 0, -9.81));
	system->GetSettings()->perform_thread_tuning = false;
	system->GetSettings()->solver.use_full_inertia_tensor = false;
	system->GetSettings()->solver.tolerance = 10.;//pumping it to tol=10, it achieves max_iter when new spawned particles
	// collide against the ones on the floor.And then keep oscillating.
	system->GetSettings()->solver.max_iteration_bilateral = 100;
	system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
	// Set number of bins manually
	system->GetSettings()->collision.bins_per_axis = vec3(binsX, binsY, binsZ);

	// Set number of threads
	system->SetParallelThreadNumber(num_threads);
	CHOMPfunctions::SetNumThreads(num_threads);

	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

	// -----------------------------------------------------------------------------------
	// ---------------------------------Create terrain bodies-----------------------------
	// -----------------------------------------------------------------------------------

	// Create contact material for terrain
	std::shared_ptr<ChMaterialSurface> material_terrain;

	switch (method) {
	case ChMaterialSurface::NSC: {
		auto mat_ter = std::make_shared<ChMaterialSurfaceNSC>();
		mat_ter->SetFriction(friction_terrain);
		mat_ter->SetRestitution(restitution_terrain);
		mat_ter->SetCohesion(coh_force_terrain);
		mat_ter->SetSpinningFriction(rolling_friction);
		mat_ter->SetRollingFriction(rolling_friction);

		material_terrain = mat_ter;

		break;
	}
	}
	// -----------------------------------------------------------------------------------
	// ---------------------------------Create terrain bodies-----------------------------
	// -----------------------------------------------------------------------------------

	// Create contact material for container, tube, funnel...
	std::shared_ptr<ChMaterialSurface> material_body;

	switch (method) {
	case ChMaterialSurface::NSC: {
		auto mat_ter = std::make_shared<ChMaterialSurfaceNSC>();
		mat_ter->SetFriction(friction_terrain);
		mat_ter->SetRestitution(restitution_terrain);
		mat_ter->SetCohesion(0.0);
		mat_ter->SetSpinningFriction(rolling_friction);
		mat_ter->SetRollingFriction(rolling_friction);

		material_body = mat_ter;

		break;
	}
	}


	// Create container body
	auto container = std::shared_ptr<ChBody>(system->NewBody());
	system->AddBody(container);
	container->SetIdentifier(-1);
	container->SetMass(1);
	container->SetBodyFixed(true);
	container->SetCollide(true);
	container->SetMaterialSurface(material_terrain);

	container->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hdimY, hthick), ChVector<>(0, 0, -hthick),
		ChQuaternion<>(1, 0, 0, 0), true);
	// Front box
	utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
		ChVector<>(hdimX + hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	// Rear box
	utils::AddBoxGeometry(container.get(), ChVector<>(hthick, hdimY, hdimZ + hthick),
		ChVector<>(-hdimX - hthick, 0, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	// Left box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
		ChVector<>(0, hdimY + hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	// Right box
	utils::AddBoxGeometry(container.get(), ChVector<>(hdimX, hthick, hdimZ + hthick),
		ChVector<>(0, -hdimY - hthick, hdimZ - hthick), ChQuaternion<>(1, 0, 0, 0), false);
	container->GetCollisionModel()->BuildModel();

	if (roughness) {
		// Adding a "roughness" to the terrain, consisting of sphere/capsule/ellipsoid grid--ENABLING AT THE TOP OF THE FILE.

		for (int ix = -40; ix < 40; ix++) {
			for (int iy = -40; iy < 40; iy++) {
				ChVector<> pos(ix * Ra_d, iy * Ra_d, 0.0);
				utils::AddSphereGeometry(container.get(), Ra_r, pos);
			}
		}
		container->GetCollisionModel()->BuildModel();
	}

	// ----------------------------------------------------------------------------------------------------------------------- //
	// ------------------------------------------------Create particles------------------------------------------------------- //
	// ----------------------------------------------------------------------------------------------------------------------- //

	// // 
	// Create a particle generator and a mixture entirely made out of spheres
	utils::Generator gen(system);
	std::shared_ptr<utils::MixtureIngredient> m1 = gen.AddMixtureIngredient(utils::SPHERE, 1);
	m1->setDefaultMaterial(material_terrain);
	m1->setDefaultDensity(rho_g);
	m1->setDefaultSize(radius_g);
	gen.setBodyIdentifier(Id_g);



	double time_end = 4.50;
	double time_step = 1e-3;

	switch (workcase) {
	case TestType::LAYER: {

		// Create particles in layers until reaching the desired number of particles
		ChVector<> hdims(2.0, 2.0, 0);
		ChVector<> center(0, 0, 2 * r);

		for (int il = 0; il < num_layers; il++) {
			std::cout << "h = " << center.z() << std::endl;
			gen.createObjectsBox(utils::POISSON_DISK, 2 * r, center, hdims);
			center.z() += 2 * r;
			// shrink uniformly the upper layer
			hdims.x() -= 2 * r;
			hdims.y() -= 2 * r;
			if (center.z() > 1.){ break; }
		}

		
		break;
	}
	}

						  // ----------------------------------------------------------------------------------------------------------------------- //
						  // ------------------------------------------------Create Bucket------------------------------------------------------- //
						  // ----------------------------------------------------------------------------------------------------------------------- //

						  //	// BUCKET
						  auto bucket = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
						  system->AddBody(bucket);
						  bucket->SetName("Bucket");
						  bucket->SetMass(1305.0);//confirmed data
						  bucket->SetInertiaXX(ChVector<>(200, 800, 200));//not confirmed data
						  bucket->SetFrame_REF_to_abs(ChFrame<>(ChVector<>(-3.0, 0., 2 * r), QUNIT));
						  bucket->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(.2, 0., 2 * r), QUNIT));
						  // Create contact geometry.
						  bucket->SetCollide(true);
						  bucket->GetCollisionModel()->ClearModel();
						  AddBucketHull(p_ext, p_int, bucket);
						  AddCapsHulls(p_int, BucketSide::LEFT, bucket);
						  AddCapsHulls(p_int, BucketSide::RIGHT, bucket);
#ifdef USE_PENALTY//temporary workaround
						  bucket->SetMaterialSurface(materialDEM);
#else
						  bucket->SetMaterialSurface(material_body);
#endif
						  bucket->GetCollisionModel()->BuildModel();
						  geometry::ChTriangleMeshConnected bucket_mesh;
						  bucket_mesh.LoadWavefrontMesh(out_dir + "data/bucket_mod.obj", false, false);
						  auto bucket_mesh_shape = std::make_shared<ChTriangleMeshShape>();
						  bucket_mesh_shape->SetMesh(bucket_mesh);
						  bucket_mesh_shape->SetName("bucket");
						  bucket->AddAsset(bucket_mesh_shape);


						  ChQuaternion<> z2x;
						  z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));


						  auto prism_fix2ch = std::make_shared<ChLinkLockPrismatic>();
						  prism_fix2ch->SetName("prismatic_ground2chassis");
						  prism_fix2ch->Initialize(bucket, container, ChCoordsys<>(bucket->GetPos(), z2x));
						  system->AddLink(prism_fix2ch);
						  auto lin_fix2ch = std::make_shared<ChLinkLinActuator>();
						  lin_fix2ch->SetName("linear_ground2chassis");
						  lin_fix2ch->Initialize(bucket, container, false, ChCoordsys<>(bucket->GetPos(), z2x), ChCoordsys<>(bucket->GetPos(), z2x));//m2 is the master
						  lin_fix2ch->Set_lin_offset(Vlength(VNULL));
						  system->AddLink(lin_fix2ch);
						  auto speed_function = std::make_shared<ChFunction_Ramp>();
						  speed_function->Set_ang(1);
						  lin_fix2ch->Set_dist_funct(speed_function);


						  unsigned int num_particles = gen.getTotalNumBodies();
						  std::cout << "Generated particles:  " << num_particles << std::endl;

						  // BRAODPHASE UTILS--
						  vec3 bins = collision::function_Compute_Grid_Resolution((int)num_particles/8, real3(hdimX, hdimY, hdimZ), .1);
						  // BRAODPHASE UTILS--
						  // Set bins from utility function
						  //system->GetSettings()->collision.bins_per_axis = bins;


#ifdef CHRONO_OPENGL
						  // -------------------------------
						  // Create the visualization window
						  // -------------------------------

						  if (render) {
							  opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
							  gl_window.Initialize(1280, 720, "Settling test", system);
							  gl_window.SetCamera(ChVector<>(0, -5., 0), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
							  gl_window.SetRenderMode(opengl::WIREFRAME);
						  }
#endif

						  // ---------------
						  // Simulate system
						  // ---------------


						  double cum_sim_time = 0;
						  double cum_broad_time = 0;
						  double cum_narrow_time = 0;
						  double cum_solver_time = 0;
						  double cum_update_time = 0;

						  TimingHeader();


						  int out_steps = std::ceil((1.0 / time_step) / out_fps);

						  int sim_frame = 0;
						  int out_frame = 0;
						  int next_out_frame = 0;

						  ChFunction_Recorder mfun;
						  ChFunction_Recorder zfun;
						  ChFunction_Recorder bnfun;

						  double avkinenergy = 0.;

						  ChVector<> bcforce ;
						  std::vector<std::shared_ptr<ChForce> > bclist;

						  // Initialize csv output file.
						  // --------------------------
						  utils::CSV_writer csv("\t");
						  csv.stream().setf(std::ios::scientific | std::ios::showpos);
						  csv.stream().precision(6);

						  while (system->GetChTime() < time_end) {

							  system->DoStepDynamics(time_step);

							  cum_sim_time += system->GetTimerStep();
							  cum_broad_time += system->GetTimerCollisionBroad();
							  cum_narrow_time += system->GetTimerCollisionNarrow();
							  cum_solver_time += system->GetTimerSolver();
							  cum_update_time += system->GetTimerUpdate();

							  bcforce = lin_fix2ch->Get_react_force();
							  csv << system->GetChTime() << bcforce.x() << bcforce.y() << bcforce.z()<< std::endl;
							  if (sim_frame % 50 == 0) {
								  csv.write_to_file(san_dir + "/output_react_5cm.dat");
							  }

							  // VISUALIZATION-OPTIONAL
#ifdef CHRONO_OPENGL
							  if (render) {
								  opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
								  if (gl_window.Active()) {
									  gl_window.Render();
								  }
								  else {
									  return 1;
								  }
							  }
#endif
							  sim_frame++;
							  //TimingOutput(system);

						  }


						  // AFTER SIMULATION STUFFS

						  std::cout << std::endl;
						  std::cout << "Simulation time: " << cum_sim_time << std::endl;
						  std::cout << "    Broadphase:  " << cum_broad_time << std::endl;
						  std::cout << "    Narrowphase: " << cum_narrow_time << std::endl;
						  std::cout << "    Solver:      " << cum_solver_time << std::endl;
						  std::cout << "    Update:      " << cum_update_time << std::endl;
						  std::cout << std::endl;

						  return 0;
	}
