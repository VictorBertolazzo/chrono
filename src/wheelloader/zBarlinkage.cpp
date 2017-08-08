// Sequential zBarlinkage mechanism of a front wheel loader
// Victor Bertolazzo

// The mechanism consists of : lift arm, rod arm, link arm, bucket
// They are attached to a fictitious chassis via : revjoint btw chassis and lift arm, linear actuator btw chassis and lift arm
// The mechanism is essentially 2D even if modeled in 3D: Chrono solver takes care of redundant constraints.
// The piston movement is modeled as a linear actuator at the moment: it could be driven either via displacement-imposed(reverse dynamics happens) and force-imposed.
// This simple test takes care only of the displacement-imposed actuattion.
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
#include "chrono_irrlicht/ChBodySceneNodeTools.h"
#include "chrono_irrlicht/ChIrrApp.h"
// Particle Generation
#include "chrono/utils/ChUtilsGenerators.h"
// C++ libraries
#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <irrlicht.h>
// OpenGL
#include "chrono_opengl/ChOpenGLWindow.h"
// Custom functions 
#include "utilities/ZBarMechanism.h"
// Chrono
#include "chrono/physics/ChSystemNSC.h"


// Use the namespaces of Chrono
using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::collision;

// Use the main namespaces of Irrlicht
using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

// Define Flag in order to decide which contact method algorithm use:NSC(DVI) or SMC(DEM)
//#define USE_PENALTY

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

  //  // measures are in [m]
		ChVector<> COG_chassis(0, 0, 1.575);							// somewhere

		ChQuaternion<> z2y;
		ChQuaternion<> z2x;
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
		
		ChQuaternion<> test_quat;		test_quat.Q_from_AngAxis(-CH_C_1_PI/2,VECT_Y);

		// 2 Create the rigid bodies : THE MECHANISM BODIES WILL BE SETUP IN A SEPARATE FUNCTION
#ifdef USE_CLASS
		// LIFT
//		auto lift = std::shared_ptr<ChBody>(system->NewBody());
		auto lift = std::make_shared<ChBody>(contact_method);
		system->Add(lift);
		lift->SetBodyFixed(false);
		lift->SetName("lift arm");
		lift->SetPos(COG_lift);
		ChVector<> u1 = (COG_lift - POS_ch2lift).GetNormalized();				//GetNormalized() yields a versor
		ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();						//overkill
		ChMatrix33<> rot1;														//no control on orthogonality
		rot1.Set_A_axis(u1, VECT_Y, w1);
		lift->SetRot(rot1);
		lift->SetMass(993.5);
		lift->SetInertiaXX(ChVector<>(110.2, 1986.1, 1919.3));
		lift->SetInertiaXY(ChVector<>(0.,0.,339.6));

		// visualization:
				auto lift_asset = std::make_shared<ChCylinderShape>();
				lift_asset->GetCylinderGeometry().rad = .025;
				lift_asset->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_ch2lift;
				lift_asset->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
				auto lift_asset1 = std::make_shared<ChCylinderShape>();
				lift_asset1->GetCylinderGeometry().rad = .025;
				lift_asset1->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
				lift_asset1->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2bucket;
				auto lift_asset2 = std::make_shared<ChCylinderShape>();
				lift_asset2->GetCylinderGeometry().rad = .025;
				lift_asset2->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * POS_ch2lift;
				lift_asset2->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * INS_ch2lift;
				auto lift_asset3 = std::make_shared<ChCylinderShape>();
				lift_asset3->GetCylinderGeometry().rad = .025;
				lift_asset3->GetCylinderGeometry().p1 = lift->GetFrame_COG_to_abs().GetInverse() * INS_ch2lift;
				lift_asset3->GetCylinderGeometry().p2 = lift->GetFrame_COG_to_abs().GetInverse() * POS_lift2bucket;

				lift->AddAsset(lift_asset);
				lift->AddAsset(lift_asset1);
				lift->AddAsset(lift_asset2);
				lift->AddAsset(lift_asset3);
				auto col_l = std::make_shared<ChColorAsset>();
				col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
				lift->AddAsset(col_l);	
		// ROD
//		auto rod = std::shared_ptr<ChBody>(system->NewBody());
		auto rod = std::make_shared<ChBody>(contact_method);
		system->Add(rod);
		rod->SetName("rod arm");
		rod->SetIdentifier(3);
		rod->SetPos(COG_rod);
		ChVector<> u3 = (POS_rod2link - POS_lift2rod).GetNormalized();
		ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();						//overkill
		ChMatrix33<> rot3;
		rot3.Set_A_axis(u3, VECT_Y, w3);
		rod->SetRot(rot3);
		rod->SetMass(381.5);
		rod->SetInertiaXX(ChVector<>(11.7, 33.4, 29.5));
		rod->SetInertiaXY(ChVector<>(0., 0., -12.1));
		// No collision properties:
		// visualization properties:
						auto rod_asset = std::make_shared<ChCylinderShape>();
						rod_asset->GetCylinderGeometry().rad = .025;
						rod_asset->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2lever;
						rod_asset->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
						auto rod_asset1 = std::make_shared<ChCylinderShape>();
						rod_asset1->GetCylinderGeometry().rad = .025;
						rod_asset1->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
						rod_asset1->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
						rod->AddAsset(rod_asset);
						rod->AddAsset(rod_asset1);
						auto col_r = std::make_shared<ChColorAsset>();
						col_r->SetColor(ChColor(0.0f, 0.0f, 0.2f));
						rod->AddAsset(col_r);
		// LINK
//		auto link = std::shared_ptr<ChBody>(system->NewBody());
		auto link = std::make_shared<ChBody>(contact_method);
		system->Add(link);
		link->SetName("link arm");
		link->SetIdentifier(3);
		link->SetPos(COG_link);
		ChVector<> u4 = (POS_link2bucket - POS_rod2link).GetNormalized();
		ChVector<> w4 = Vcross(u4, VECT_Y).GetNormalized();							//overkill
		ChMatrix33<> rot4;
		rot4.Set_A_axis(u4, VECT_Y, w4);
		link->SetRot(rot4);
		link->SetMass(277.2);
		link->SetInertiaXX(ChVector<>(3.2, 11.1, 13.6));
		link->SetInertiaXY(ChVector<>(0.0, 0.0, -.04));
						// No collision properties:
						// visualization properties:
						auto link_asset = std::make_shared<ChCylinderShape>();
						link_asset->GetCylinderGeometry().rad = .025;
						link_asset->GetCylinderGeometry().p1 = link->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
						link_asset->GetCylinderGeometry().p2 = link->GetFrame_COG_to_abs().GetInverse() * POS_link2bucket;
						link->AddAsset(link_asset);
						auto col_l1 = std::make_shared<ChColorAsset>();
						col_l1->SetColor(ChColor(0.0f, 0.2f, 0.2f));
						link->AddAsset(col_l1);
		// BUCKET
//		auto bucket = std::shared_ptr<ChBodyAuxRef>(system->NewBodyAuxRef());
		auto bucket = std::make_shared<ChBodyAuxRef>(contact_method);
		system->AddBody(bucket);
		bucket->SetName("benna");
		bucket->SetIdentifier(4);
		bucket->SetMass(200.0);														//not confirmed data
		bucket->SetInertiaXX(ChVector<>(20, 50, 20));								//not confirmed data
		bucket->SetPos(POS_lift2bucket);
		bucket->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(.35, .0, .2), QUNIT));		//tentative:NOT A KNOWN FEATURES 
				// Create contact geometry.
		bucket->SetCollide(true);
		bucket->GetCollisionModel()->ClearModel();
		AddBucketHull(p_ext, p_int, bucket);
		AddCapsHulls(p_int, BucketSide::LEFT, bucket);
		AddCapsHulls(p_int, BucketSide::RIGHT, bucket);
		bucket->GetCollisionModel()->BuildModel();
#ifdef USE_PENALTY
		bucket->SetMaterialSurface(materialDEM);
#else
		bucket->SetMaterialSurface(material_test);
#endif
		geometry::ChTriangleMeshConnected bucket_mesh;
		bucket_mesh.LoadWavefrontMesh(out_dir + "data/bucket_mod.obj", false, false);
		auto bucket_mesh_shape = std::make_shared<ChTriangleMeshShape>();
		bucket_mesh_shape->SetMesh(bucket_mesh);
		bucket->AddAsset(bucket_mesh_shape);

		
		// CHASSIS
//		auto chassis = std::shared_ptr<ChBody>(system->NewBody());
		auto chassis = std::make_shared<ChBody>(contact_method);
		system->AddBody(chassis);
		chassis->SetBodyFixed(false);
		chassis->SetName("chassis");
		chassis->SetIdentifier(0);
		chassis->SetMass(2000.0);
		chassis->SetPos(COG_chassis);
		chassis->SetInertiaXX(ChVector<>(500., 1000., 500.));
						// collision properties:
						chassis->GetCollisionModel()->ClearModel();
						chassis->GetCollisionModel()->AddSphere(.05, VNULL);
						chassis->GetCollisionModel()->BuildModel();
						chassis->SetCollide(true);
						// visualization properties
						auto chassis_asset = std::make_shared<ChSphereShape>();				//asset
						chassis_asset->GetSphereGeometry().rad = .05;						//asset
						chassis->AddAsset(chassis_asset);

		
						

						

// 3. Add joint constraints:REVOLUTE JOINTS+LINEAR ACTUATOR AT THE MOMENT
						// Please note the rotations due to the fact that:
						//													- rev joint rotates around its frame z-axis
						//													- lin actuator moves along its frame z-axis


			// LIFT-ROD spring(it simulates the actuator)--LEFT HERE FOR HISTORY BUT REPLACED BY LIN ACT
						auto springdamper_ground_ball = std::make_shared<ChLinkSpring>();
						springdamper_ground_ball->Initialize(rod,lift,false,POS_lift2lever,PIS_lift2lever,true,.00);
						springdamper_ground_ball->Set_SpringK(50.0);
						springdamper_ground_ball->Set_SpringR(0.0);
						auto lin_lift2rod = std::make_shared<ChLinkLinActuator>();
						ChVector<> u11 = (POS_lift2lever - PIS_lift2lever).GetNormalized();			//GetNormalized()
						ChVector<> w11 = Vcross(u11, VECT_Y).GetNormalized();						//overkill
						ChMatrix33<> rot11;															//no control on orthogonality
						rot11.Set_A_axis(u11, VECT_Y, w11);
						lin_lift2rod->SetName("linear_chassis2lift");
						lin_lift2rod->Initialize(rod, lift, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
						lin_lift2rod->Set_lin_offset(Vlength(POS_lift2lever - PIS_lift2lever));
//	//	ASSET FOR THE LINEAR ACTUATOR
						auto bp_asset = std::make_shared<ChPointPointSegment>();					//asset
						lin_lift2rod->AddAsset(bp_asset);
//	//
						auto legge1 = std::make_shared<ChFunction_Ramp>();					
						legge1->Set_ang(1.0);
						auto legge2 = std::make_shared<ChFunction_Const>();
						auto legge3 = std::make_shared<ChFunction_Ramp>();
						legge3->Set_ang(-1.0);
						auto tilt_law = std::make_shared<ChFunction_Sequence>();
						tilt_law->InsertFunct(legge1, 1.0, 1, true);
						tilt_law->InsertFunct(legge2, 1.0, 1., true);
						tilt_law->InsertFunct(legge3, 1.0, 1., true);
						auto tilt_law_seq = std::make_shared<ChFunction_Repeat>();
						tilt_law_seq->Set_fa(tilt_law);
						tilt_law_seq->Set_window_length(3.0);
						tilt_law_seq->Set_window_start(0.0);
						tilt_law_seq->Set_window_phase(3.0);
						// test_law
						auto tilt_law_test = std::make_shared<ChFunction_Const>();
						tilt_law_test->Set_yconst(Vlength(VNULL));
						// end test_law
						lin_lift2rod->Set_dist_funct(tilt_law);
						//system->AddLink(springdamper_ground_ball);
						system->Add(lin_lift2rod);

			// LIFT-ROD inthe middle revjoint 
						auto rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
						rev_lift2rod->SetName("revolute_lift2rod");
						rev_lift2rod->Initialize(rod, lift, ChCoordsys<>(POS_lift2rod, z2y >> rot3.Get_A_quaternion()));
						system->AddLink(rev_lift2rod);

			// ROD-LINK revjoint
						auto rev_rod2link = std::make_shared<ChLinkLockRevolute>();
						rev_rod2link->SetName("revolute_rod2link");
						ChMatrix33<> rotb44;
						rev_rod2link->Initialize(link, rod, ChCoordsys<>(POS_rod2link, z2y >> rotb44.Get_A_quaternion()));
						system->AddLink(rev_rod2link);

			// LIFT-BUCKET revjoint
						auto rev_lift2bucket = std::make_shared<ChLinkLockRevolute>();
						rev_lift2bucket->SetName("revolute_lift2bucket");
						ChMatrix33<> rotb1;
						rev_lift2bucket->Initialize(bucket, lift, ChCoordsys<>( POS_lift2bucket, z2y >> rotb1.Get_A_quaternion()));
						system->AddLink(rev_lift2bucket);

			// LINK-BUCKET revjoint
						auto rev_link2bucket = std::make_shared<ChLinkLockRevolute>();
						rev_link2bucket->SetName("revolute_link2bucket");
						ChMatrix33<> rotb2;
						rev_link2bucket->Initialize(bucket, link, ChCoordsys<>(POS_link2bucket, z2y >> rotb2.Get_A_quaternion()));//Does it make sense?
						system->AddLink(rev_link2bucket);
			
			// CHASSIS-LIFT revjoint
						auto rev_ch2lift = std::make_shared<ChLinkLockRevolute>();
						rev_ch2lift->SetName("revolute_chassis2lift");
						rev_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(POS_ch2lift, z2y >> rot1.Get_A_quaternion()), ChCoordsys<>(POS_ch2lift, z2y >> rot1.Get_A_quaternion()));
						system->AddLink(rev_ch2lift);

			// CHASSIS-LIFT linear actuator
						auto lin_ch2lift = std::make_shared<ChLinkLinActuator>();
						ChVector<> u22 = (INS_ch2lift - PIS_ch2lift).GetNormalized();//Normalize,not GetNormalize
						ChVector<> w22 = Vcross(u22, VECT_Y).GetNormalized();//overkill
						ChMatrix33<> rot22;//no control on orthogonality
						rot22.Set_A_axis(u22, VECT_Y, w22);
						lin_ch2lift->SetName("linear_chassis2lift");
						lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(INS_ch2lift, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, z2x >> rot11.Get_A_quaternion()));//m2 is the master
						lin_ch2lift->Set_lin_offset(Vlength(INS_ch2lift - PIS_ch2lift));
						auto lift_law = std::make_shared<ChFunction_Sequence>();
						lift_law->InsertFunct(legge1, 1.0, 1, true);
						lift_law->InsertFunct(legge2, 1.0, 1., true);
						lift_law->InsertFunct(legge3, 1.0, 1., true);
						auto lift_law_seq = std::make_shared<ChFunction_Repeat>();
						lift_law_seq->Set_fa(lift_law);
						lift_law_seq->Set_window_length(3.0);
						lift_law_seq->Set_window_start(0.0);
						lift_law_seq->Set_window_phase(3.0);
						auto lift_law_sine = std::make_shared<ChFunction_Sine>();
						lift_law_sine->Set_w(1.0472);
						lift_law_sine->Set_amp(.5*Vlength(INS_ch2lift - PIS_ch2lift));
//	//	ASSET FOR LINEAR ACTUATOR
						lin_ch2lift->AddAsset(std::make_shared<ChPointPointSegment>());
						// test_law
						auto lift_law_test = std::make_shared<ChFunction_Const>();
						lift_law_test->Set_yconst(Vlength(VNULL));
						// end test_law
						lin_ch2lift->Set_dist_funct(lift_law_sine);
						system->AddLink(lin_ch2lift);
#else				

    //CreateMechanism(*system, container);// Previous function.Obsolete.
    MyWheelLoader* mywl = new MyWheelLoader(*system); 
//    system->ShowHierarchy(GetLog());
	double total_mass = mywl->GetMass();
	GetLog() << "Total Mass " << total_mass  << "\n";
	
			// CHASSIS-GROUND prismatic+hydractuator

			// CHASSIS-GROUND prismatic+linactuator
						auto prism_fix2ch = std::make_shared<ChLinkLockPrismatic>();
						prism_fix2ch->SetName("prismatic_ground2chassis");
						prism_fix2ch->Initialize(mywl->chassis, ground, ChCoordsys<>(COG_chassis, z2x));
						system->AddLink(prism_fix2ch);
						auto lin_fix2ch = std::make_shared<ChLinkLinActuator>();
						prism_fix2ch->SetName("linear_ground2chassis");
						lin_fix2ch->Initialize(mywl->chassis, ground, false, ChCoordsys<>(COG_chassis, z2x), ChCoordsys<>(COG_chassis, z2x));//m2 is the master
						lin_fix2ch->Set_lin_offset(Vlength(VNULL));
						system->AddLink(lin_fix2ch);
						auto chassis_law = std::make_shared<ChFunction_Ramp>();
						chassis_law->Set_ang(.00);//it'll act as the chassis speed
						lin_fix2ch->Set_dist_funct(chassis_law);
						
				
			//// CHASSIS-GROUND prismatic+linactuator
						//auto prism_fix2ch = std::make_shared<ChLinkLockPrismatic>();
						//prism_fix2ch->SetName("prismatic_ground2chassis");
						//prism_fix2ch->Initialize(mywl->chassis, ground, ChCoordsys<>(mywl->chassis->GetPos(), mywl->z2x));
						//system->AddLink(prism_fix2ch);

						//auto lin_fix2ch = std::make_shared<ChLinkLinActuator>();
						//prism_fix2ch->SetName("linear_ground2chassis");
						//ChQuaternion<> prismrot;
						//prismrot.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
						//lin_fix2ch->Initialize(mywl->chassis, ground, false, ChCoordsys<>(mywl->chassis->GetPos(), prismrot), ChCoordsys<>(ChVector<>(-10.0, 0., 0.), prismrot));//m2 is the master
						//lin_fix2ch->Set_lin_offset(Vlength(ChVector<>(-10., 0., 0.) - mywl->chassis->GetPos()));// A random far point is chosen s.t. in "reverse gear" motion linear displacement does not turn to negative(i.e. bad behaviour)
						//system->AddLink(lin_fix2ch);

#endif

// 4. Write the system hierarchy to the console (default log output destination)
//system->ShowHierarchy(GetLog());

// 5. Prepare visualization with Irrlicht
//    Note that Irrlicht uses left-handed frames with Y up.
	
	
	
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
		// Apply Camera Rotation
		irr::scene::ICameraSceneNode* camera = application->GetSceneManager()->addCameraSceneNode(application->GetSceneManager()->getRootSceneNode(), core::vector3df(+2.5, +4., 0), core::vector3df(2., 0, 0));
		camera->setUpVector(core::vector3df(0, 0, 1));

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


		// Irrlicht must finish drawing the frame
		application->EndScene();
	}




    return 0;
}
