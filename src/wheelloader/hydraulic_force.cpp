// This example receives in input the actuator data and elaborates them feeding the latter.
// Moreover, it overloads the classical ChLinkLinActuator class in a way to accept nor only displacement but also
//			force inputs.
// It must be merged with test_HYDR_actuator.cpp
// Victor Bertolazzo
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

#include "chrono_parallel/physics/ChSystemParallel.h"
#include "chrono_parallel/solver/ChIterativeSolverParallel.h"


#include "chrono/motion_functions/ChFunction_Recorder.h"
#include "chrono/motion_functions/ChFunction_Integrate.h"
#include "chrono/motion_functions/ChFunction_Base.h"

#include "chrono/core/ChLog.h"
#include "chrono/core/ChVectorDynamic.h"
#include "chrono/motion_functions/ChFunction_Sine.h"


#include "chrono_postprocess/ChGnuPlot.h"
#include "chrono/assets/ChPointPointDrawing.h"


#include "chrono/physics/ChLinkLinActuator.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
using namespace chrono;
using namespace postprocess;
#include "chrono/physics/ChLinkMarkers.h"

#include "utilities/UtilityFunctions.h"
// --------------------------------------------------------------------------
using std::cout;
using std::endl;
// -------------------ENUMERATOR ----------------------------
enum FunctionSettingMode {INFILE , INPLACE};
FunctionSettingMode mode = INPLACE;
bool render = true;


int main(int argc, char** argv) {
	int num_threads = 4;


	// Get number of threads from arguments (if specified)
	if (argc > 1) {
		num_threads = std::stoi(argv[1]);
	}

	std::cout << "Requested number of threads: " << num_threads << std::endl;

	// Preliminary Settings, catch the input file or write a sine function

	std::vector<TimeSeries> input;
	const std::string& act_input = "../data/actuator_input.txt";


	
	
	//----------SYSTEM------------
	chrono::ChSystemParallel* system;

#ifdef USE_PENALTY
	ChSystemParallelSMC* sys = new ChSystemParallelSMC;
	sys->GetSettings()->solver.contact_force_model = ChSystemDEM::Hertz;
	sys->GetSettings()->solver.tangential_displ_mode = ChSystemDEM::TangentialDisplacementModel::OneStep;
	sys->GetSettings()->solver.use_material_properties = use_mat_properties;
	system = sys;
#else
	ChSystemParallelNSC* sys = new ChSystemParallelNSC;
	sys->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
	sys->GetSettings()->solver.max_iteration_normal = 0;
	sys->GetSettings()->solver.max_iteration_sliding = 200;
	sys->GetSettings()->solver.max_iteration_spinning = 0;
	sys->GetSettings()->solver.alpha = 0;
	sys->GetSettings()->solver.contact_recovery_speed = -1;
	sys->GetSettings()->collision.collision_envelope = 0.01;
	sys->ChangeSolverType(SolverType::APGD);
	system = sys;

#endif // USE_PENALTY

	system->Set_G_acc(ChVector<>(0, 0, -9.81));
	system->GetSettings()->perform_thread_tuning = false;
	system->GetSettings()->solver.use_full_inertia_tensor = false;
	system->GetSettings()->solver.tolerance = 0.1;
	system->GetSettings()->solver.max_iteration_bilateral = 100;
	system->GetSettings()->collision.narrowphase_algorithm = NarrowPhaseType::NARROWPHASE_HYBRID_MPR;
	system->GetSettings()->collision.bins_per_axis = vec3(10, 10, 10);

	// Rendering Settings
#ifdef CHRONO_OPENGL
	if (render) {
		opengl::ChOpenGLWindow& gl_window = opengl::ChOpenGLWindow::getInstance();
		gl_window.Initialize(1280, 720, "Settling test", system);
		gl_window.SetCamera(ChVector<>(9., 0., 4.), ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 0.05f);
		gl_window.SetRenderMode(opengl::WIREFRAME);
	}
#endif // CHRONO_OPENGL

	// Set number of threads
	system->SetParallelThreadNumber(20);
	CHOMPfunctions::SetNumThreads(20);
	// Sanity check: print number of threads in a parallel region
#pragma omp parallel
#pragma omp master 
	{ std::cout << "Actual number of OpenMP threads: " << omp_get_num_threads() << std::endl; }

	//---------------------------------------------------
	double mass = 1.0;
	ChVector<> inertiaXX(1, 1, 1);
	ChVector<> gravity(0, 0, -9.80665);
	ChQuaternion<> rot(Q_from_AngY(CH_C_PI / 2));
	ChVector<> axis = rot.GetZaxis();

	// Create the ground body.
	std::shared_ptr<ChBody> ground(system->NewBody());

	system->AddBody(ground);
	ground->SetBodyFixed(true);

	auto box_g = std::make_shared<ChBoxShape>();
	box_g->GetBoxGeometry().SetLengths(ChVector<>(0.1, 0.1, 5));
	box_g->Pos = 2.5 * axis;
	box_g->Rot = rot;
	ground->AddAsset(box_g);

	// Create the plate body.

	std::shared_ptr<ChBody> plate(system->NewBody());

	system->AddBody(plate);
	plate->SetPos(ChVector<>(0, 0, 0));
	plate->SetRot(rot);
	plate->SetMass(mass);
	plate->SetInertiaXX(inertiaXX);

	auto box_p = std::make_shared<ChBoxShape>();
	box_p->GetBoxGeometry().SetLengths(ChVector<>(1, 1, 0.2));
	plate->AddAsset(box_p);


	auto prismatic = std::make_shared<ChLinkLockPrismatic>();
	prismatic->Initialize(plate, ground, ChCoordsys<>(ChVector<>(0, 0, 0), rot));
	system->AddLink(prismatic);

	

//#define USE_DISPLACEMENT
	// Choose between the two type of connections: check distances and orientations
#ifdef USE_DISPLACEMENT
	//Setup the function--Old

	auto pres_function = std::make_shared<ChFunction_Recorder>();//shared_ptr gives memory acces violation in AddPoint member
	if (mode == INFILE){
		ReadFile(act_input, input);

		std::cout << input[4].mv << std::endl;

		for (int i = 0; i < input.size(); i++){
			pres_function->AddPoint(input[i].mt, 10 * input[i].mv);
		}
	}
	auto linAB = std::make_shared<ChLinkLinActuator>(); 
	ChVector<> pt1 = ChVector<>(0, 0, 0);
	ChVector<> pt2 = axis;
	linAB->Initialize(ground, plate, false, ChCoordsys<>(pt1, rot), ChCoordsys<>(pt2, rot));
	linAB->Set_lin_offset(5.);//safe
	system->AddLink(linAB);
	linAB->Set_dist_funct(pres_function);
	//	// Asset for the linear actuator--NOT WORKING WITH OPENGL
	//	auto bp_asset = std::make_shared<ChPointPointSegment>();				//asset
	//	linAB->AddAsset(bp_asset);
	//
#else
	ChFunction_Recorder pressure;

	if (mode == INFILE){
		ReadFile(act_input, input);
		for (int i = 0; i < input.size(); i++){
			// scalar gain to be observable in simulation
			pressure.AddPoint(input[i].mt, 1e6*input[i].mv);
		}
		pressure.AddPoint(input.back().mt + 1., 0.);
	}
	else if (mode == INPLACE){
		pressure.AddPoint(0., 10);
	    
	}
	else { std::cout << "No valid input mode" << std::endl; }

	// Gnuplot--IT DOESN'T WORK WITH ChFunction_Sine or others.
	ChGnuPlot mplot("__tmp_gnuplot_4.gpl");
	mplot.SetGrid();
	mplot.Plot(pressure, "Pressure Function", " with lines lt -1 lc rgb'#00AAEE' ");

	myHYDRforce force;
	auto linAB = std::make_shared<myHYDRactuator>();
	// ChLinkMarkers child, force applied on slave m1
	linAB->Initialize(plate, ground, ChCoordsys<>(ChVector<>(0, 0, 0), rot));
	linAB->Set_HYDRforce(&force);
	linAB->Set_PressureH(&pressure);
	system->AddLink(linAB);
	// Attach a visualization asset.
	linAB->AddAsset(std::make_shared<ChPointPointSpring>(0.05, 80, 15));	
#endif


	// Simulation
	ChFunction_Recorder rforce;
	ChFunction_Recorder pforce;
	ChFunction_Recorder bxpos;
	ChFunction_Recorder bypos;
	ChFunction_Recorder bzpos;

	while (system->GetChTime() < 5.) {

		system->DoStepDynamics(.001);
		rforce.AddPoint(system->GetChTime(), linAB->Get_react_force().x());
		pforce.AddPoint(system->GetChTime(), linAB->GetC_force().z());
		bxpos.AddPoint(system->GetChTime(), plate->GetPos().x());
		bypos.AddPoint(system->GetChTime(), plate->GetPos().y());
		bzpos.AddPoint(system->GetChTime(), plate->GetPos().z());

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
	}
	// Gnuplot--IT DOESN'T WORK WITH ChFunction_Sine or others.
	ChGnuPlot fplot("__tmp_gnuplot_5.gpl");
	fplot.SetGrid();
	fplot.Plot(pforce, "pneumatic Force", " with lines lt -1 lc rgb'#00AAEE' ");

	ChGnuPlot rplot("__tmp_gnuplot_6.gpl");
	rplot.SetGrid();
	rplot.Plot(rforce, "pneumatic reaction", " with lines lt -1 lc rgb'#00AAEE' ");

	ChGnuPlot xplot("__tmp_gnuplot_7.gpl");
	xplot.SetGrid();
	xplot.Plot(bxpos, "x Position", " with lines lt -1 lc rgb'#00AAEE' ");
	ChGnuPlot yplot("__tmp_gnuplot_8.gpl");
	yplot.SetGrid();
	yplot.Plot(bypos, "y Position", " with lines lt -1 lc rgb'#00AAEE' ");
	ChGnuPlot zplot("__tmp_gnuplot_9.gpl");
	zplot.SetGrid();
	zplot.Plot(bzpos, "z Position", " with lines lt -1 lc rgb'#00AAEE' ");


	return 0;
}