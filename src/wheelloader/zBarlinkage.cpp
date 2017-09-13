// Sequential zBarlinkage mechanism of a front wheel loader
// Victor Bertolazzo

// The mechanism consists of : lift arm, rod arm, link arm, bucket
// They are attached to a fictitious chassis via : revjoint btw chassis and lift arm, linear actuator btw chassis and lift arm,linear actuator btw chassis and rod arm
// The mechanism is essentially 2D even if modeled in 3D: Chrono solver takes care of redundant constraints.
// The piston movement is modeled as a linear actuator at the moment: it could be driven either via displacement-imposed(reverse dynamics happens) and force-imposed.
// This simple test takes care only of the displacement-imposed actuation.
// Only the bucket body has collision shapes, visualization shapes of the other bodies are kept simplified for a better understanding of the mechanism.


// Irrlicht
#include "chrono/assets/ChTriangleMeshShape.h"
#include "chrono/collision/ChCCollisionUtils.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkLinActuator.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChPointPointDrawing.h"
// Particle Generation
#include "chrono/utils/ChUtilsGenerators.h"
// C++ libraries
#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>
#ifdef CHRONO_IRRLICHT
#include <irrlicht.h>
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"
#endif
// OpenGL
#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChOpenGLWindow.h"
#endif
// Custom functions 
#include "utilities/ZBarMechanism.h"
// Chrono
#include "chrono/physics/ChSystemNSC.h"


// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::collision;

// Use the main namespaces of Irrlicht
#ifdef CHRONO_IRRLICHT
using namespace chrono::irrlicht;
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;
#endif

// -------------------TIME SERIES STRUCTURE----------------------------
// Time Series structure: utility to grab two columns file values(t_k,f(t_k)) extensively used in this project.
struct TimeSeries {
	TimeSeries() {}
	TimeSeries(float t, float v)
		: mt(t), mv(v) {}
	float mt; float mv;
};
// -------------------READ PRESSURE FILE FUNCTION----------------------------
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
// -------------------SET SPEED PROFILE FUNCTION----------------------------
// Impose forward velocity profile, and set displacement law consequently
void SetSpeedProfile(std::shared_ptr<ChLinkLinActuator> act, std::vector<TimeSeries> speed){
	auto chass_traj = std::make_shared<ChFunction_Recorder>();

	chass_traj->AddPoint(0., 0.);
		for (int i = 1; i < speed.size(); i++){
			double pos = chass_traj->Get_y(speed[i-1].mt) + speed[i].mv * (	speed[i].mt - speed[i-1].mt	);
				chass_traj->AddPoint(speed[i].mt, pos);
			//target_speed->AddPoint(DesiredSpeed[i].mt, DesiredSpeed[i].mv);
		}

		act->Set_dist_funct(chass_traj);

}

// Enumerator used for creating the closing caps of the bucket
enum BucketSide { LEFT, RIGHT };
int main(int argc, char* argv[]) {

	// Create a material (will be used by both objects)
	auto material = std::make_shared<ChMaterialSurfaceNSC>();
	material->SetRestitution(0.1f);
	material->SetFriction(0.4f);
	// Create a material (will be used by both objects)
	auto materialDEM = std::make_shared<ChMaterialSurfaceSMC>();
	materialDEM->SetYoungModulus(1.0e7f);
	materialDEM->SetRestitution(0.1f);
	materialDEM->SetFriction(0.4f);
	materialDEM->SetAdhesion(0);  // Magnitude of the adhesion in Constant adhesion model

	// 0. Set the path to the Chrono data folder

	// 1. Create the system: it's creating with a boring method due previous bugs
	ChSystem* system;
#ifdef USE_PENALTY
	ChSystemSMC* sys = new ChSystemSMC;
	ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::SMC;
	system = sys;
#else
	ChSystemNSC* sys = new ChSystemNSC;
	ChMaterialSurface::ContactMethod contact_method = ChMaterialSurface::NSC;
	system = sys;
#endif

	// Set the gravity acceleration
	system->Set_G_acc(ChVector<>(.0,.0,-9.806));



	//////////////////////////////////////////////////////--------------LIST OF THE BODIES-------------------////////////////////////////////////////////
	// GROUND
//	auto ground = std::shared_ptr<ChBody>(system->NewBody());
	auto ground = std::make_shared<ChBody>(contact_method);
	system->Add(ground);
	ground->SetBodyFixed(true);
	ground->SetIdentifier(-1);
	ground->SetName("ground");
	//ground->SetMaterialSurface(mat_g);
	ground->GetCollisionModel()->ClearModel();
	// Bottom box
	utils::AddBoxGeometry(ground.get(), ChVector<>(10., 10., 3.0), ChVector<>(0, 0, -3.0),
		ChQuaternion<>(1, 0, 0, 0), false);
	ground->GetCollisionModel()->BuildModel();
	ground->SetCollide(true);


  //  // measures are in [m]
		ChVector<> COG_chassis(0, 0, 1.575);							// somewhere

		ChQuaternion<> z2y;
		ChQuaternion<> z2x;
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
		

		// 2 Create the rigid bodies : THE MECHANISM BODIES WILL BE SETUP IN A SEPARATE FUNCTION

    MyWheelLoader* mywl = new MyWheelLoader(*system); 
	double total_mass = mywl->GetMass();
	GetLog() << "Total Mass " << total_mass  << "\n";

			// Inplace : CHASSIS-GROUND prismatic+linactuator
						auto prism_fix2ch = std::make_shared<ChLinkLockPrismatic>();
						prism_fix2ch->SetName("prismatic_ground2chassis");
						prism_fix2ch->Initialize(mywl->chassis, ground, ChCoordsys<>(COG_chassis, z2x));
						system->AddLink(prism_fix2ch);
						auto lin_fix2ch = std::make_shared<ChLinkLinActuator>();
						prism_fix2ch->SetName("linear_ground2chassis");
						lin_fix2ch->Initialize(mywl->chassis, ground, false, ChCoordsys<>(COG_chassis, z2x), ChCoordsys<>(COG_chassis, z2x));//m2 is the master
						lin_fix2ch->Set_lin_offset(Vlength(VNULL));
						system->AddLink(lin_fix2ch);

						// Impose motion to the actuator -- Test file WL_DesiredSpeed.dat(Change file!)
						std::vector<TimeSeries> DesiredSpeed;
						ReadPressureFile("../data/WL_DesiredSpeedSmoothed.dat", DesiredSpeed);
						SetSpeedProfile(lin_fix2ch, DesiredSpeed);


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

// 4. Write the system hierarchy to the console (default log output destination)
//system->ShowHierarchy(GetLog());

// 5. Prepare visualization with Irrlicht
//    Note that Irrlicht uses left-handed frames with Y up.
	
	
#ifdef CHRONO_IRRLICHT
	// Create the Irrlicht application and set-up the camera.
	ChIrrApp * application = new ChIrrApp(
		system,                               // pointer to the mechanical system
		L"WL Z Bar Mechanism",                // title of the Irrlicht window
		core::dimension2d<u32>(800, 600),      // window dimension (width x height)
		false,                                 // use full screen?
		true);                                 // enable shadows?
	application->AddTypicalLogo();
	application->AddTypicalSky();
	application->AddTypicalLights();
	application->AddTypicalCamera(core::vector3df(3, +8, 0), core::vector3df(0,0,0)); //'camera' location            // "look at" location
											   // Let the Irrlicht application convert the visualization assets.
	//application->GetSceneManager()->getActiveCamera()->bindTargetAndRotation(true);
	//application->GetSceneManager()->getActiveCamera()->setRotation(irr::core::vector3df(0,0,0));

	//// Apply Camera Rotation
		//irr::scene::ICameraSceneNode* camera = application->GetSceneManager()->addCameraSceneNode(application->GetSceneManager()->getRootSceneNode(), core::vector3df(+2.5, +4., 0), core::vector3df(2., 0, 0));
		//camera->setUpVector(core::vector3df(0, 0, 1));

	//application->GetSceneManager()->getActiveCamera()->setUpVector(core::vector3df(0, 0, 1));
	application->AssetBindAll();
	application->AssetUpdateAll();
    
	application->SetTimestep(0.01);
	application->SetTryRealtime(true);

	while (application->GetDevice()->run()) {

		// Irrlicht must prepare frame to draw
		application->BeginScene();
		// Irrlicht application draws all 3D objects and all GUI items
		application->DrawAll();
		// Draw an XZ grid at the global origin to add in visualization.
		ChIrrTools::drawGrid(
			application->GetVideoDriver(), 1, 1, 20, 20,
			ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
			video::SColor(255, 80, 100, 100), true);

		// Advance the simulation time step
		application->DoStep();

ChVector<> pist = 		mywl->lin_ch2lift->Get_react_force();
double pist_g = mywl->GetReactLiftForce();

ChVector<> pistt = mywl->lin_lift2rod->Get_react_force();
double pistt_g = mywl->GetReactTiltForce();

double 	angl = 	mywl->rev_ch2lift->GetRelAngle();
double angl_g = mywl->GetLiftArmPitchAngle();

double angll = mywl->rev_lift2bucket->GetRelAngle();
double angll_g = mywl->GetTiltBucketPitchAngle();
// Irrlicht must finish drawing the frame
		application->EndScene();
	}



#endif
    return 0;
}
