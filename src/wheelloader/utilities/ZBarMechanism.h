using namespace chrono;
using namespace chrono::collision;

#include "chrono/physics/ChLinkMarkers.h"
// Actuator test times : VALUES
double ta1 = 2.00; double ta2 = 11.00; double ta3 = 15.00; double ta4 = 5.00; double ta5 = 12.00;
// Actuator test times : TESTING IDEA
// Tilt : constant until ta4, then negative displacement(piling) until ta5, at the end constant til ta3;
// Lift : constant until ta1, then positive displacement(piling) til ta2, at the constant til ta3;
// chassis : FWD motion til ta1, stop until ta2, then RWD til ta3;
// ------------------CLASSES----------------------------------------------
#include "Pneumatics.h"
#include "chrono/physics/ChLinkSpringCB.h"

double spring_coef = 50;
double damping_coef = 1;


// ------------------CLASSES----------------------------------------------
class MyWheelLoader {
    public:
	// Data
		// Functor class implementing the force for a ChLinkSpringCB link.
		class MySpringForce : public ChLinkSpringCB::ForceFunctor {
			double operator()(double time,          // current time
				double rest_length,   // undeformed length
				double length,        // current length
				double vel,           // current velocity (positive when extending)
				ChLinkSpringCB* link  // back-pointer to associated link
				) override {
				double force = -spring_coef * (length - rest_length) - damping_coef * vel;
				return force;
			}
		};

	// Handles
	//std::shared_ptr<ChBody> lift;
	std::shared_ptr<ChBodyAuxRef> lift;
	std::shared_ptr<ChBody> rod;
	std::shared_ptr<ChBody> link;
	std::shared_ptr<ChBodyAuxRef> bucket;
	std::shared_ptr<ChBody> chassis;

	std::shared_ptr<ChLinkLockRevolute> rev_lift2rod;
	std::shared_ptr<ChLinkLockRevolute> rev_rod2link;
	std::shared_ptr<ChLinkLockRevolute> rev_lift2bucket;
	std::shared_ptr<ChLinkLockRevolute> rev_link2bucket;
	std::shared_ptr<ChLinkLockRevolute> rev_ch2lift;
	// USE_PNEUMATICS flag.
	std::shared_ptr<myHYDRactuator> lin_ch2lift;		
	std::shared_ptr<myHYDRactuator> lin_lift2rod;	
	
	//std::shared_ptr<ChLinkLinActuator> lin_ch2lift;
	//std::shared_ptr<ChLinkLinActuator> lin_lift2rod;

	ChQuaternion<> z2y;
	ChQuaternion<> z2x;
	ChQuaternion<> y2x;

	// Utility Functions
	enum BucketSide { LEFT, RIGHT };
	struct Points {
		Points() {}
		Points(float x, float y, float z)
			: mx(x), my(y), mz(z) {}
		float mx; float my; float mz;
	};
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
			double width = 1.0;// or halve an input

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
				width = +1.;
				break;
			case RIGHT:
				width = -1.0;
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
	// -------------------TIME SERIES STRUCTURE----------------------------
	struct TimeSeries {
		TimeSeries() {}
		TimeSeries(float t, float v)
			: mt(t), mv(v) {}
		float mt; float mv;
	};
	// -------------------READ PRESSURE FILE FUNCTION----------------------------
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
	// Constructor	

	MyWheelLoader(ChSystem& system){


		///////////////////////////////////////////////////////////////Constructor Utilities
		std::vector<Points> p_ext;
		std::vector<Points> p_int;
		const std::string out_dir = "../";
		const std::string& ext_profile = out_dir + "data/ext_profile.txt";
		const std::string& int_profile = out_dir + "data/int_profile.txt";
		ReadFile(ext_profile, p_ext);
		ReadFile(int_profile, p_int);
		// Create a material (will be used by both objects)
		auto materialDVI = std::make_shared<ChMaterialSurfaceNSC>();
		materialDVI->SetRestitution(0.1f);
		materialDVI->SetFriction(0.4f);
		// Create a material (will be used by both objects)
		auto materialDEM = std::make_shared<ChMaterialSurfaceSMC>();
		materialDEM->SetYoungModulus(1.0e7f);
		materialDEM->SetRestitution(0.1f);
		materialDEM->SetFriction(0.4f);
		materialDEM->SetAdhesion(0);  								// Magnitude of the adhesion in Constant adhesion model

		////////////////////////////////////////////////////////Links Position Data

		ChVector<> POS_ch2lift(0., 0, 2.115);						//Rev chassis->lift--POINT [B]
		ChVector<> PIS_ch2lift(0., 0, 1.675);						//Act chassis->lift--POINT [A]
		ChVector<> PIS_lift2lever(.320, 0, 2.027);					//Act lift->lever--POINT [C]

		ChVector<> INS_ch2lift(1.13, 0, 1.515);						// Insertion of boom piston over lift body--POINT [D]
		ChVector<> POS_lift2bucket(2.26, .0, .832);					//chassis piston(LIFT ARM) insertion abs frame--POINT [G]
		ChVector<> POS_lift2rod(1.60, .0, 1.80);					//rev joint(LIFT ARM) abs frame--POINT [F]
		  // suppose rod arm oriented parallel to vertical axis(assumption, useful to define initial positions)
		ChVector<> POS_lift2lever(POS_lift2rod.x() -.2, 0, POS_lift2rod.z() + (1.267-.770));//end position actuator lift->lever--POINT [E]

		ChVector<> POS_rod2link(POS_lift2rod.x() - .2, 0, POS_lift2rod.z() - (.770));		//Act lift->lever--POINT [H]
			// same trick for link arm, parallel to ground
		ChVector<> POS_link2bucket(POS_rod2link.x() + .718, .0, POS_rod2link.z());	//chassis piston(BUCKET LEVER) insertion abs frame--POINT [L]
		////////////////////////////////////////////////////////COG Position Data
		ChVector<> COG_chassis(POS_ch2lift); // somewhere not defined
		ChVector<> COG_lift(ChVector<>(2.6/2,0.,.518/3));
		ChVector<> COG_rod(POS_lift2rod);
		ChVector<> COG_link(POS_rod2link);
		ChVector<> COG_bucket(POS_lift2bucket); // not easy definition

		/// USEFUL Quaternions
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));
		y2x.Q_from_AngAxis(CH_C_PI_2, VECT_Z);

		////////////////////////--------------Create rigid bodies-------------------------------////////////////////////////////////////////
		///////////////////////-----------------------------------------------------------------////////////////////////////////////////////
		// LIFT
		lift = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());//switched to ChBodyAuxRef
		system.Add(lift);
		lift->SetBodyFixed(false);
		lift->SetName("lift arm");
		lift->SetPos(POS_ch2lift);// switched to ChBodyAuxRef;
// not working //		lift->SetFrame_REF_to_COG(ChFrame<>(COG_lift,ChMatrix33<>(1,0,0,0,1,0,0,0,1)).GetInverse());
		ChVector<> u1 = (POS_lift2bucket - POS_ch2lift).GetNormalized();//switched to ChBodyAuxRef
		ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rot1;//no control on orthogonality
		rot1.Set_A_axis(u1, VECT_Y, w1);
		lift->SetRot(rot1);	//		lift->SetFrame_COG_to_REF(ChFrame<>(lift->GetFrame_REF_to_abs().GetInverse() * COG_lift, QUNIT));//switched to ChBodyAuxRef
		lift->SetMass(928.0);
		lift->SetInertiaXX(ChVector<>(110.2, 1986.1, 1919.3));
		lift->SetInertiaXY(ChVector<>(0., 0., 339.6));

		// primitive visualization:
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
		lift->AddAsset(lift_asset);lift->AddAsset(lift_asset1);lift->AddAsset(lift_asset2);lift->AddAsset(lift_asset3);
		auto col_l = std::make_shared<ChColorAsset>();
		col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
		lift->AddAsset(col_l);
		geometry::ChTriangleMeshConnected lift_mesh;
		lift_mesh.LoadWavefrontMesh(out_dir + "data/boom_mod.obj", false, false);
		auto lift_mesh_shape = std::make_shared<ChTriangleMeshShape>();
		lift_mesh_shape->SetMesh(lift_mesh);
		lift_mesh_shape->SetName("boom");
		//lift->AddAsset(lift_mesh_shape);

		// collision model, in order to get natural link limits(abandoned).
		////lift->SetCollide(true);
		////lift->GetCollisionModel()->ClearModel();
		////ChVector<> ucyl1 = (POS_lift2bucket - POS_ch2lift).GetNormalized();ChVector<> wcyl1 = Vcross(ucyl1, VECT_Y).GetNormalized();//overkill
		////ChMatrix33<> rotcyl1; rotcyl1.Set_A_axis(ucyl1, VECT_Y, wcyl1);
		////utils::AddCylinderGeometry(lift.get(), .025, Vlength(POS_ch2lift - POS_lift2rod), POS_ch2lift, rotcyl1.Get_A_quaternion() , true);
		////lift->GetCollisionModel()->BuildModel();


		// ROD
		rod = std::shared_ptr<ChBody>(system.NewBody());
		system.Add(rod);
		rod->SetName("rod arm");
		rod->SetIdentifier(3);
		//rod->SetPos(POS_lift2rod);//COG_rod
		rod->SetPos(COG_rod);//
		ChVector<> u3 = (POS_rod2link - POS_lift2rod).GetNormalized();
		ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rot3;
		rot3.Set_A_axis(u3, VECT_Y, w3);
		rod->SetRot(rot3);
		//	rod->SetFrame_COG_to_REF(ChFrame<>(rod->GetFrame_REF_to_abs().GetInverse() * COG_rod, QUNIT));//switched to ChBodyAuxRef
		rod->SetMass(318.0);
		rod->SetInertiaXX(ChVector<>(11.7, 33.4, 29.5));
		rod->SetInertiaXY(ChVector<>(0., 0., -12.1));
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
		geometry::ChTriangleMeshConnected rocker_mesh;
		rocker_mesh.LoadWavefrontMesh(out_dir + "data/rockerarm_mod.obj", false, false);// file not present
		auto rocker_mesh_shape = std::make_shared<ChTriangleMeshShape>();
		rocker_mesh_shape->SetMesh(rocker_mesh);
		rocker_mesh_shape->SetName("boom");
		//rod->AddAsset(rocker_mesh_shape);//temporary
		// collision model

		//	// LINK
		link = std::shared_ptr<ChBody>(system.NewBody());
		system.Add(link);
		link->SetName("link arm");
		link->SetIdentifier(3);
		link->SetPos(COG_link);
		ChVector<> u4 = (POS_link2bucket - POS_rod2link).GetNormalized();
		ChVector<> w4 = Vcross(u4, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rot4;
		rot4.Set_A_axis(u4, VECT_Y, w4);
		link->SetRot(rot4);
		link->SetMass(56.0);
		link->SetInertiaXX(ChVector<>(3.2, 11.1, 13.6));
		link->SetInertiaXY(ChVector<>(0.0, 0.0, -.04));
		// visualization properties:
		auto link_asset = std::make_shared<ChCylinderShape>();
		link_asset->GetCylinderGeometry().rad = .025;
		link_asset->GetCylinderGeometry().p1 = link->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
		link_asset->GetCylinderGeometry().p2 = link->GetFrame_COG_to_abs().GetInverse() * POS_link2bucket;
		link->AddAsset(link_asset);
		auto col_l1 = std::make_shared<ChColorAsset>();
		col_l1->SetColor(ChColor(0.0f, 0.2f, 0.2f));
		link->AddAsset(col_l1);
		// collision model, in order to get natural link limits(abandoned).
		link->SetCollide(true);
		link->GetCollisionModel()->ClearModel();
		//link->GetCollisionModel()->AddCylinder(.025, .025, Vlength(POS_rod2link - POS_link2bucket), link->GetFrame_COG_to_abs().GetInverse() * POS_rod2link, y2x >> rot4.Get_A_quaternion());
		utils::AddCylinderGeometry(link.get(), .025, Vlength(POS_rod2link - POS_link2bucket)/2, link->GetFrame_COG_to_abs().GetInverse() * (POS_rod2link/2+POS_link2bucket/2), y2x >> rot4.Get_A_quaternion(), true);
		link->GetCollisionModel()->BuildModel();

		//	// BUCKET
		bucket = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
		system.AddBody(bucket);
		bucket->SetName("benna");
		bucket->SetIdentifier(4);
		bucket->SetMass(1305.0);//confirmed data
		bucket->SetInertiaXX(ChVector<>(200, 800, 200));//not confirmed data
		bucket->SetPos(POS_lift2bucket);
		//bucket->SetFrame_COG_to_REF(ChFrame<> (bucket->GetFrame_REF_to_abs().GetInverse() * COG_bucket,QUNIT));
		bucket->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(.05, .0, .02), QUNIT));//tentative
		// Create contact geometry.
		bucket->SetCollide(true);
		bucket->GetCollisionModel()->ClearModel();
		AddBucketHull(p_ext, p_int, bucket);
		AddCapsHulls(p_int, BucketSide::LEFT, bucket);
		AddCapsHulls(p_int, BucketSide::RIGHT, bucket);
#ifdef USE_PENALTY//temporary workaround
		bucket->SetMaterialSurface(materialDEM);
#else
		bucket->SetMaterialSurface(materialDVI);
#endif
		bucket->GetCollisionModel()->BuildModel();
		geometry::ChTriangleMeshConnected bucket_mesh;
		bucket_mesh.LoadWavefrontMesh(out_dir + "data/bucket_mod.obj", false, false);
		auto bucket_mesh_shape = std::make_shared<ChTriangleMeshShape>();
		bucket_mesh_shape->SetMesh(bucket_mesh);
		bucket_mesh_shape->SetName("bucket");
		bucket->AddAsset(bucket_mesh_shape);

		// CHASSIS
		chassis = std::shared_ptr<ChBody>(system.NewBody());
		system.AddBody(chassis);
		chassis->SetBodyFixed(true);//temporary
		chassis->SetName("chassis");
		chassis->SetIdentifier(0);
		chassis->SetMass(2000.0);
		chassis->SetPos(COG_chassis);
		chassis->SetPos_dt(ChVector<>(.0, .0, .0));// u=2.25m/s
		chassis->SetInertiaXX(ChVector<>(500., 1000., 500.));
		// visualization properties
		auto chassis_asset = std::make_shared<ChSphereShape>();//asset
		chassis_asset->GetSphereGeometry().rad = .05;//asset
		chassis->AddAsset(chassis_asset);

		



		/////////////////////////////////////------------------Add joint constraints------------------////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Revolute Joint between the lift body and the rod, located near the geometric baricenter(int(r dA)/int(dA)) of the rod LIFT-ROD. 
		// // It is the second of the three joints that interest the rod body(also known as rocker arm).
		rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
		rev_lift2rod->SetName("revolute_lift2rod");
		rev_lift2rod->Initialize(rod, lift, ChCoordsys<>(POS_lift2rod, z2y >> rot3.Get_A_quaternion()));		// z-dir default rev axis is rotated on y-dir
		system.AddLink(rev_lift2rod);

		// Revolute Joint between the rod body and link, last of the three joints interesting the rod , rear joint of the two concerning the link.
		rev_rod2link = std::make_shared<ChLinkLockRevolute>();
		rev_rod2link->SetName("revolute_rod2link");
		ChMatrix33<> rotb44; rotb44.Set_A_axis(VECT_X,VECT_Y,VECT_Z);
		rev_rod2link->Initialize(link, rod, ChCoordsys<>(POS_rod2link, z2y >> rotb44.Get_A_quaternion()));		// z-dir default rev axis is rotated on y-dir 
		system.AddLink(rev_rod2link);
		// LIFT-BUCKET revjoint
		rev_lift2bucket = std::make_shared<ChLinkLockRevolute>();
		rev_lift2bucket->SetName("revolute_lift2bucket");
		ChMatrix33<> rotb1; rotb1.Set_A_axis(VECT_X, VECT_Y, VECT_Z);
		rev_lift2bucket->Initialize(bucket, lift, ChCoordsys<>(POS_lift2bucket, z2y >> rotb1.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir
		system.AddLink(rev_lift2bucket);
		// LINK-BUCKET revjoint
		rev_link2bucket = std::make_shared<ChLinkLockRevolute>();
		rev_link2bucket->SetName("revolute_link2bucket");
		ChMatrix33<> rotb2; rotb2.Set_A_axis(VECT_X, VECT_Y, VECT_Z);
		rev_link2bucket->Initialize(bucket, link, ChCoordsys<>(POS_link2bucket, z2y >> rotb2.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir
		system.AddLink(rev_link2bucket);

		// CHASSIS-LIFT revjoint
		rev_ch2lift = std::make_shared<ChLinkLockRevolute>();
		rev_ch2lift->SetName("revolute_chassis2lift");
		rev_ch2lift->Initialize(lift, chassis, ChCoordsys<>(POS_ch2lift, z2y >> rot1.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir
		system.AddLink(rev_ch2lift);


		//-----------------------------------------------------------------------------------------------------//


		// Linear Actuator between the lift body and the so called rod(also known as rocker arm)
		ChVector<> u11 = (POS_lift2lever - PIS_lift2lever).GetNormalized();		//GetNormalized() yields a unit vector(versor)
		ChVector<> w11 = Vcross(u11, VECT_Y).GetNormalized();					//overkill
		ChMatrix33<> rot11;														//no control on orthogonality, IT'S UP TO THE USER'
		rot11.Set_A_axis(u11, VECT_Y, w11);
#define USE_PNEUMATIC
#ifdef USE_PNEUMATIC
		// Using prismatic connection oriented as pneumatic actuator, the system does not move(hyper-constrained).

		//auto prism_lin_lift2rod = std::make_shared<ChLinkLockPrismatic>();
		//prism_lin_lift2rod->Initialize(rod, chassis, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
		//system.AddLink(prism_lin_lift2rod);


		auto tforce = std::make_shared<myHYDRforce>();
		//ChFunction_Recorder pressure; pressure.AddPoint(0., 1);
		auto thpressure = std::make_shared<ChFunction_Recorder>(); thpressure->AddPoint(0., 3.8167e5);
		auto trpressure = std::make_shared<ChFunction_Recorder>(); trpressure->AddPoint(0., 3.8167e5);
		
		// Using only the pneumatic actuator, weight of the arms makes them oscillating like a pendulum, hence real pressures are not enough to lift the system.
		lin_lift2rod = std::make_shared<myHYDRactuator>();
		lin_lift2rod->SetName("linear_lift2rod");// ChLinkMarkers child, force applied on slave m1
		lin_lift2rod->Initialize(rod, chassis, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
		lin_lift2rod->Set_HYDRforce(tforce);
		lin_lift2rod->Set_PressureH(thpressure);
		lin_lift2rod->Set_PressureR(trpressure);
		lin_lift2rod->SetAreaH(CH_C_1_PI / 4 * pow(.150, 2));// Head Side, Diameter: 150mm
		lin_lift2rod->SetAreaR(CH_C_1_PI / 4 * ( pow(.150, 2) - pow(.080,2)));// Piston Rod, Diameter: 80mm
		// Attach a visualization asset.
		lin_lift2rod->AddAsset(std::make_shared<ChPointPointSpring>(0.05, 80, 15));
#else
		lin_lift2rod = std::make_shared<ChLinkLinActuator>();
		//		lin_lift2rod = std::unique_ptr<ChLinkMarkers>(new ChLinkLinActuator());
		lin_lift2rod->SetName("linear_lift2rod");
		lin_lift2rod->Initialize(rod, lift, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
		lin_lift2rod->Set_lin_offset(Vlength(POS_lift2lever - PIS_lift2lever));

		// A test law for the actuator--> it'll be substitued by an accessor method->no more inplace law definiton in the future.
		// temporary piston law for lift2rod actuator -> it'll be set constant by default and changeable by accessor
		// Note : it is as displacement law
		auto legget1 = std::make_shared<ChFunction_Const>();
		auto legget2 = std::make_shared<ChFunction_Ramp>(); legget2->Set_ang(-.008);
		auto legget3 = std::make_shared<ChFunction_Const>();
		auto tilt_law_test = std::make_shared<ChFunction_Sequence>(); tilt_law_test->InsertFunct(legget1, ta4, 1, true); tilt_law_test->InsertFunct(legget2, ta5 - ta4, 1., true); tilt_law_test->InsertFunct(legget3, ta3 - ta5, 1., true);
		//	// Former test function
		auto legge1 = std::make_shared<ChFunction_Ramp>();			legge1->Set_ang(1.0);
		auto legge2 = std::make_shared<ChFunction_Const>();
		auto legge3 = std::make_shared<ChFunction_Ramp>();			legge3->Set_ang(-1.0);
		auto tilt_law = std::make_shared<ChFunction_Sequence>();
		tilt_law->InsertFunct(legge1, 1.0, 1, true); tilt_law->InsertFunct(legge2, 1.0, 1., true); tilt_law->InsertFunct(legge3, 1.0, 1., true);
		auto tilt_law_seq = std::make_shared<ChFunction_Repeat>();
		tilt_law_seq->Set_fa(tilt_law);
		tilt_law_seq->Set_window_length(3.0);
		tilt_law_seq->Set_window_start(0.0);
		tilt_law_seq->Set_window_phase(3.0);
		// test_law
		auto tilt_law_testing = std::make_shared<ChFunction_Const>();
		tilt_law_testing->Set_yconst(Vlength(VNULL));
		// end test_law
		
		lin_lift2rod->Set_dist_funct(tilt_law_test);
		// Asset for the linear actuator
		auto bp_asset = std::make_shared<ChPointPointSegment>();				//asset
		lin_lift2rod->AddAsset(bp_asset);

#endif
		system.AddLink(lin_lift2rod);



		// CHASSIS-LIFT linear actuator--Lift Piston
		ChVector<> u22 = (INS_ch2lift - PIS_ch2lift).GetNormalized();					//GetNormalized()
		ChVector<> w22 = Vcross(u22, VECT_Y).GetNormalized();							//overkill
		ChMatrix33<> rot22;																//no control on orthogonality, IT'S UP TO THE USER
		rot22.Set_A_axis(u22, VECT_Y, w22);

#ifdef USE_PNEUMATIC
		// Using prismatic connection oriented as pneumatic actuator, the system does not move(hyper-constrained).

		//auto prism_lin_ch2lift = std::make_shared<ChLinkLockPrismatic>();
		//prism_lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(INS_ch2lift, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, z2x >> rot22.Get_A_quaternion()));//m2 is the master
		//system.AddLink(prism_lin_ch2lift);

		std::vector<TimeSeries> ReadHeadPressure;
		ReadPressureFile("../data/HeadLiftPressure.dat",ReadHeadPressure);
		auto lhpressure = std::make_shared<ChFunction_Recorder>();
		for (int i = 0; i < ReadHeadPressure.size(); i++){
			lhpressure->AddPoint(ReadHeadPressure[i].mt, 2e5*ReadHeadPressure[i].mv);//2 pistons,data in [bar]
		}
		std::vector<TimeSeries> ReadRodPressure;
		ReadPressureFile("../data/RodLiftPressure.dat", ReadRodPressure);
		auto lrpressure = std::make_shared<ChFunction_Recorder>();
		for (int i = 0; i < ReadRodPressure.size(); i++){
			lrpressure->AddPoint(ReadRodPressure[i].mt, 2e5*ReadRodPressure[i].mv);// 2 pistons, data in [bar]
		}

		auto lforce = std::make_shared<myHYDRforce>();
		
		
		// Using only the pneumatic actuator, weight of the arms makes them oscillating like a pendulum, hence real pressures are not enough to lift the system.
		lin_ch2lift = std::make_shared<myHYDRactuator>();
		lin_ch2lift->SetName("linear_chassis2lift");// ChLinkMarkers child, force applied on slave m1
		lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(INS_ch2lift, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, z2x >> rot22.Get_A_quaternion()));//m2 is the master
		lin_ch2lift->SetAreaH(CH_C_1_PI / 4 * pow(.130,2));// Head Side, Diameter: 130mm
		lin_ch2lift->SetAreaR(CH_C_1_PI / 4 * ( pow(.130, 2) - pow(.080,2)));// Piston Rod, Diameter: 80mm
		lin_ch2lift->Set_HYDRforce(lforce);
		ChForce force;
		lin_ch2lift->Set_PressureH(lhpressure);
		lin_ch2lift->Set_PressureR(lrpressure);
		// Attach a visualization asset.
		lin_ch2lift->AddAsset(std::make_shared<ChPointPointSpring>(0.05, 80, 15));
#else
		lin_ch2lift = std::make_shared<ChLinkLinActuator>();
		lin_ch2lift->SetName("linear_chassis2lift");
		lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(INS_ch2lift, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, z2x >> rot11.Get_A_quaternion()));//m2 is the master
		lin_ch2lift->Set_lin_offset(Vlength(INS_ch2lift - PIS_ch2lift));
		// temporary piston law for chassis2lift actuator -> it'll be set constant by default and changeable by accessor
		// Note : it is as displacement law
		auto leggel1 = std::make_shared<ChFunction_Const>();
		auto leggel2 = std::make_shared<ChFunction_Ramp>(); leggel2->Set_ang(+.05);
		auto leggel3 = std::make_shared<ChFunction_Const>();
		auto lift_law = std::make_shared<ChFunction_Sequence>(); lift_law->InsertFunct(leggel1, ta1, 1, true); lift_law->InsertFunct(leggel2, ta2 - ta1, 1., true); lift_law->InsertFunct(leggel3, ta3 - ta2, 1., true);
		auto lift_law_test = std::make_shared<ChFunction_Sequence>();
		lift_law_test->InsertFunct(legge1, 1.0, 1, true);
		lift_law_test->InsertFunct(legge2, 1.0, 1., true);
		lift_law_test->InsertFunct(legge3, 1.0, 1., true);
		auto lift_law_seq = std::make_shared<ChFunction_Repeat>();
		lift_law_seq->Set_fa(lift_law_test);
		lift_law_seq->Set_window_length(3.0);
		lift_law_seq->Set_window_start(0.0);
		lift_law_seq->Set_window_phase(3.0);
		auto lift_law_sine = std::make_shared<ChFunction_Sine>();
		lift_law_sine->Set_w(1.0472);
		lift_law_sine->Set_amp(.5*Vlength(INS_ch2lift - PIS_ch2lift));
		//	//	ASSET FOR LINEAR ACTUATOR
		lin_ch2lift->AddAsset(std::make_shared<ChPointPointSegment>());
		// test_law
		auto lift_law_testing = std::make_shared<ChFunction_Const>();
		lift_law_testing->Set_yconst(Vlength(VNULL));
		// end test_law

		lin_ch2lift->Set_dist_funct(lift_law_sine);
#endif

		system.AddLink(lin_ch2lift);
	}
	// Destructor
	~MyWheelLoader(){}
	// Getter
	void PrintValue(){
		GetLog() << "This is a PrintValue() class method: " << chassis->GetPos().z() << "\n";
	}
	double GetMass(){
	return	lift->GetMass() + link->GetMass() + rod->GetMass() + bucket->GetMass();
	}
	// Setter

};

