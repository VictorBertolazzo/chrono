using namespace chrono;
using namespace chrono::collision;

#include "chrono/physics/ChLinkMarkers.h"
#include "chrono/assets/ChPointPointDrawing.h"

// Actuator test times : VALUES
double ta1 = 2.00; double ta2 = 11.00; double ta3 = 15.00; double ta4 = 5.00; double ta5 = 12.00;
// Actuator test times : TESTING IDEA
// Tilt : constant until ta4, then negative displacement(piling) until ta5, at the end constant til ta3;
// Lift : constant until ta1, then positive displacement(piling) til ta2, at the constant til ta3;
// chassis : FWD motion til ta1, stop until ta2, then RWD til ta3;
// ------------------CLASSES----------------------------------------------



//#define USE_PNEUMATIC 


#ifdef USE_PNEUMATIC
#include "Pneumatics.h"
#endif

#include "chrono/physics/ChLinkSpringCB.h"

double spring_coef = 50;
double damping_coef = 1;


// ------------------CLASSES----------------------------------------------
class MyWheelLoader {
    public:
	// Handles to the bodies
	std::shared_ptr<ChBodyAuxRef> lift;
	std::shared_ptr<ChBodyAuxRef> rod;
	std::shared_ptr<ChBodyAuxRef> link;
	std::shared_ptr<ChBodyAuxRef> bucket;
	std::shared_ptr<ChBody> chassis;

	// Handles to the links
	std::shared_ptr<ChLinkLockRevolute> rev_lift2rod;
	std::shared_ptr<ChLinkLockRevolute> rev_rod2link;
	std::shared_ptr<ChLinkLockRevolute> rev_lift2bucket;
	std::shared_ptr<ChLinkLockRevolute> rev_link2bucket;
	std::shared_ptr<ChLinkLockRevolute> rev_ch2lift;


#ifdef USE_PNEUMATIC
	std::shared_ptr<myHYDRactuator> lin_ch2lift;		
	std::shared_ptr<myHYDRactuator> lin_lift2rod;	
#else
	std::shared_ptr<ChLinkLinActuator> lin_ch2lift;
	std::shared_ptr<ChLinkLinActuator> lin_lift2rod;
#endif

	// Quaternions Initialization
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

	// AddBucketHull function: it takes two different hull points set in order to approximate the shape of the bucket(without the lateral caps) into an assembly of prismatic convex shapes.
	//							At each iteration the "cloud" vector has 8 points, defining the vertexes of the prism.
	//							AddConvexHull function converts this cloud into a collision shape, while ChConvexHullLibraryWrapper allows it to be visualized.
	//							No BuildModel method is here, since it must be called outside in the body initialization steps where this function is used.
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
	// AddCapsHull function : it receives only one profile from the bucket point of view(the inner), and build up a point cloud of 6 elements, 
	//						  in order to create a series of tetrahedrons which will close the bucket side.
	//							AddConvexHull function converts this cloud into a collision shape, while ChConvexHullLibraryWrapper allows it to be visualized.
	//							No BuildModel method is here, since it must be called outside in the body initialization steps where this function is used.
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
	
	
	// Constructor	

	MyWheelLoader(ChSystem& system, ChCoordsys<> location){


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


		//
		ChFrame<> loader_to_abs(location);
		// useless
		//loader_to_abs.ConcatenatePreTransformation(chassis->GetFrame_REF_to_abs());
		////////////////////////////////////////////////////////Links Position Data--Letter of the points refer to Schematics.vsd file.

		ChVector<> POS_ch2lift(0., 0, 2.115);														//Rev chassis->lift--POINT [B]
		ChVector<> PIS_ch2lift(0., 0, 1.675);														//Act chassis->lift--POINT [A]
		ChVector<> PIS_lift2lever(.320, 0, 2.027);													//Act lift->lever--POINT [C]

		ChVector<> INS_ch2lift(1.13, 0, 1.515);														// Insertion of boom piston over lift body--POINT [D]
		ChVector<> POS_lift2bucket(2.26, .0, .832);													//Defined in abs frame		//rev joint --POINT [G]

		ChVector<> POS_lift2bucket_lift(2.6, 0., 0.);												//Defined in LIFT frame		//rev joint --POINT [G]

		ChVector<> POS_lift2rod(1.60, .0, 1.80);													//Defined in abs frame		//rev joint(LIFT ARM) abs frame--POINT [F]
		ChVector<> POS_lift2rod_lift(1.550, .0, .518);												//Defined in LIFT frame		//rev joint(LIFT ARM) abs frame--POINT [F]

		// suppose rod arm oriented parallel to vertical axis(assumption, useful to define initial positions)
		ChVector<> POS_lift2lever(POS_lift2rod.x() -.2, 0, POS_lift2rod.z() + (1.267-.770));		// Defined in ABS frame		//end position actuator lift->lever--POINT [E]
		ChVector<> POS_lift2lever_rod(-.497, 0,-.20 );												// Defined in ROD frame		//end position actuator lift->lever--POINT [E]

		ChVector<> POS_rod2link(POS_lift2rod.x() - .2, 0, POS_lift2rod.z() - (.770));				//Defined in ABS frame		//Act lift->lever--POINT [H]
		ChVector<> POS_rod2link_rod(.770, 0, -.0);													// Defined in ROD frame		//Act lift->lever--POINT [H]

		// same trick for link arm, parallel to ground
		ChVector<> POS_link2bucket(POS_rod2link.x() + .718, .0, POS_rod2link.z());					// Defined in ABS frame		//chassis piston(BUCKET LEVER) insertion abs frame--POINT [L]
		ChVector<> POS_link2bucket_link(.718, .0, 0.);												// Defined in LINK frame	//chassis piston(BUCKET LEVER) insertion abs frame--POINT [L]


		////////////////////////////////////////////////////////COG Position Data-Relative Reference Frames Coordinates
		ChVector<> COG_chassis(POS_ch2lift);							// not important
		ChVector<> COG_lift(ChVector<>(2.6/2,0.,.518/3));				// Lift body com-relative frame
		ChVector<> COG_rod(ChVector<>(1.267 / 2, 0., 0.));				// Rod body com-relative frame
		ChVector<> COG_link(ChVector<>(.718/2,0.,0.));					// Link body com-relative frame
		ChVector<> COG_bucket(ChVector<>(.15,0.,.5));					// (not previously known location)


		///// Change from relative to absolute ref frame
		POS_ch2lift = loader_to_abs.TransformLocalToParent(POS_ch2lift);
		PIS_ch2lift = loader_to_abs.TransformLocalToParent(PIS_ch2lift);
		PIS_lift2lever = loader_to_abs.TransformLocalToParent(PIS_lift2lever);
		INS_ch2lift = loader_to_abs.TransformLocalToParent(INS_ch2lift);
		POS_lift2bucket = loader_to_abs.TransformLocalToParent(POS_lift2bucket);
		//POS_lift2bucket_lift = loader_to_abs.TransformLocalToParent(POS_lift2bucket_lift);
		POS_lift2rod = loader_to_abs.TransformLocalToParent(POS_lift2rod);
		//POS_lift2rod_lift = loader_to_abs.TransformLocalToParent(POS_lift2rod_lift);
		POS_lift2lever = loader_to_abs.TransformLocalToParent(POS_lift2lever);
		//POS_lift2lever_rod = loader_to_abs.TransformLocalToParent(POS_lift2lever_rod);
		POS_rod2link = loader_to_abs.TransformLocalToParent(POS_rod2link);
		//POS_rod2link_rod = loader_to_abs.TransformLocalToParent(POS_rod2link_rod);
		POS_link2bucket = loader_to_abs.TransformLocalToParent(POS_link2bucket);
		//POS_link2bucket_link = loader_to_abs.TransformLocalToParent(POS_link2bucket_link);
		COG_chassis = loader_to_abs.TransformLocalToParent(COG_chassis);
		COG_lift = loader_to_abs.TransformLocalToParent(COG_lift);
		COG_rod = loader_to_abs.TransformLocalToParent(COG_rod);
		COG_link = loader_to_abs.TransformLocalToParent(COG_link);
		COG_bucket = loader_to_abs.TransformLocalToParent(COG_bucket);

		/// USEFUL Quaternions-- not referenced to the abs frame yet(QUNIT argument at this time)
		z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));			// It rotates -90° about x axis--used in revolute-like joint definition
		z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));			// It rotates 90° about y axis--used in prismatic-like joint definition
		y2x.Q_from_AngAxis(CH_C_PI_2, VECT_Z);							// It rotates 90° about z axis

		////////////////////////--------------Create rigid bodies-------------------------------////////////////////////////////////////////
		///////////////////////-----------------------------------------------------------------////////////////////////////////////////////

		// LIFT BODY
		// It is also called the boom. It's the biggest and heaviest among the attachment arms, and connects to the front part by means of a revolute joint 
		//                                    and a piston actuator(in reality two, but they're parallel considering x-y perspective, hence summarized in once in x-z plane.).
		// It links with bucket with a revolute joint and with so-called ROD BODY(also rocker) with another revolute.
		lift = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
		system.Add(lift);
		lift->SetBodyFixed(false);
		lift->SetName("lift arm");
		lift->SetIdentifier(1);

		// Coordinates Definition
		ChVector<> u1 = (POS_lift2bucket - POS_ch2lift).GetNormalized();
		ChVector<> w1 = Vcross(u1, VECT_Y).GetNormalized();
		ChMatrix33<> rot1;
		rot1.Set_A_axis(u1, VECT_Y, w1);
		lift->SetFrame_REF_to_abs(ChFrame<>(POS_ch2lift, rot1));
		lift->SetFrame_COG_to_REF(ChFrame<>(COG_lift, QUNIT));
		
		// Mass Properties
		lift->SetMass(928.0 + 2 * 136.0);
		lift->SetInertiaXX(ChVector<>(110.2, 1986.1, 1919.3));
		lift->SetInertiaXY(ChVector<>(0., 0., 339.6));

		// Primitive visualization:
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
		//lift->AddAsset(lift_asset);lift->AddAsset(lift_asset1);lift->AddAsset(lift_asset2);lift->AddAsset(lift_asset3);
		
		// Visualization Color
		auto col_l = std::make_shared<ChColorAsset>();
		col_l->SetColor(ChColor(0.2f, 0.0f, 0.0f));
		lift->AddAsset(col_l);
		
		// Mesh Visualization
		geometry::ChTriangleMeshConnected lift_mesh;
		lift_mesh.LoadWavefrontMesh(out_dir + "data/ZK-L550-boom.obj", false, false);
		auto lift_mesh_shape = std::make_shared<ChTriangleMeshShape>();
		lift_mesh_shape->SetMesh(lift_mesh);
		lift_mesh_shape->SetName("boom");
		lift->AddAsset(lift_mesh_shape);

		// Collision Model().
		lift->SetCollide(true);
		lift->GetCollisionModel()->ClearModel();
		ChVector<> ucyl1 = (POS_lift2bucket - POS_ch2lift).GetNormalized();ChVector<> wcyl1 = Vcross(ucyl1, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rotcyl1; rotcyl1.Set_A_axis(VECT_X, VECT_Y, VECT_Z);
		utils::AddCylinderGeometry(lift.get(), .025, Vlength(POS_ch2lift - POS_lift2bucket) / 2 - .2, POS_lift2bucket_lift/2 + ChVector<>(0.,.6,0.), y2x >> rotcyl1.Get_A_quaternion(), false);
		utils::AddCylinderGeometry(lift.get(), .025, Vlength(POS_ch2lift - POS_lift2bucket) / 2 - .2, POS_lift2bucket_lift / 2 + ChVector<>(0., -.6, 0.), y2x >> rotcyl1.Get_A_quaternion(), false);
		lift->GetCollisionModel()->BuildModel();


		// ROD BODY
		// Sometimes it is called rocker or linkage body.
		// It is the body deputized to transmit the tilting movement to the bucket.
		// It connects with the front part of the vehicle by means of a piston actuator, to the so-called LIFT BODY(boom) with a revolute.
		// Another revolute links the body to the LINK BODY(connecting rod)
		rod = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
		system.Add(rod);
		rod->SetName("rod arm");
		rod->SetIdentifier(2);
		// Coordinates Definition
		ChVector<> u3 = (POS_rod2link - POS_lift2rod).GetNormalized();//absolute coords
		ChVector<> w3 = Vcross(u3, VECT_Y).GetNormalized();//overkill
		ChMatrix33<> rot3;
		rot3.Set_A_axis(u3, VECT_Y, w3);

		ChQuaternion<> q13; q13.Q_from_AngAxis(80*CH_C_DEG_TO_RAD,VECT_Y);

		ChQuaternion<> q3 = rot1.Get_A_quaternion() >> q13;
		//// Definition in ABS frame
		//rod->SetFrame_REF_to_abs(ChFrame<>(POS_lift2rod,rot3));
		//rod->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0.,0.,0.), QUNIT));

		// Definition in LIFT frame
		std::cout << "ABS Frame: " << POS_lift2rod.x() << "\t" << POS_lift2rod.z() << std::endl;
		std::cout << "LIFT Frame: " << (lift->GetFrame_REF_to_abs()*POS_lift2rod_lift).x() <<"\t" << (lift->GetFrame_REF_to_abs()*POS_lift2rod_lift).z() << std::endl;
		rod->SetFrame_REF_to_abs(ChFrame<>(lift->GetFrame_REF_to_abs()*POS_lift2rod_lift, q3));// use rot3 for abs, q3 for relative
		rod->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(0., 0., 0.), QUNIT));

		// Mass Properties
		rod->SetMass(318.0+127.0);
		rod->SetInertiaXX(ChVector<>(11.7, 33.4, 29.5));
		rod->SetInertiaXY(ChVector<>(0., 0., -12.1));
		
		// Primitive Visualization :
		auto rod_asset = std::make_shared<ChCylinderShape>();
		rod_asset->GetCylinderGeometry().rad = .025;
		rod_asset->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2lever;
		rod_asset->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
		auto rod_asset1 = std::make_shared<ChCylinderShape>();
		rod_asset1->GetCylinderGeometry().rad = .025;
		rod_asset1->GetCylinderGeometry().p1 = rod->GetFrame_COG_to_abs().GetInverse() * POS_lift2rod;
		rod_asset1->GetCylinderGeometry().p2 = rod->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
		//rod->AddAsset(rod_asset);rod->AddAsset(rod_asset1);

		// Visualization Color
		auto col_r = std::make_shared<ChColorAsset>();
		col_r->SetColor(ChColor(0.0f, 0.0f, 0.2f));
		rod->AddAsset(col_r);

		// Mesh Visualization
		geometry::ChTriangleMeshConnected rocker_mesh;
		rocker_mesh.LoadWavefrontMesh(out_dir + "data/ZK-L550-linkage.obj", false, false);// 
		auto rocker_mesh_shape = std::make_shared<ChTriangleMeshShape>();
		rocker_mesh_shape->SetMesh(rocker_mesh);
		rocker_mesh_shape->SetName("linkage");
		rod->AddAsset(rocker_mesh_shape);
		// Collision model

		// LINK BODY
		// It is also called connecting rod.
		// Its extremities connect with ROD BODY and BUCKET, both revolute joints with.
		link = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
		system.Add(link);
		link->SetName("link arm");
		link->SetIdentifier(3);
		// Coordinates Definition
		ChVector<> u4 = (POS_link2bucket - POS_rod2link).GetNormalized();
		ChVector<> w4 = Vcross(u4, VECT_Y).GetNormalized();
		ChMatrix33<> rot4;
		rot4.Set_A_axis(u4, VECT_Y, w4);

		ChQuaternion<> q34; q34.Q_from_AngAxis(-115 * CH_C_DEG_TO_RAD, VECT_Y);
		ChQuaternion<> q4 = rot3.Get_A_quaternion() >> q34;

		//// Definition in ABS frame
		//link->SetFrame_REF_to_abs(ChFrame<>(POS_rod2link, rot4));
		//link->SetFrame_COG_to_REF(ChFrame<>(COG_link, QUNIT));

		// Definition in ROD frame
		std::cout << "ABS Frame: " << POS_rod2link.x() << "\t" << POS_rod2link.z() << std::endl;
		std::cout << "ROD Frame: " << (rod->GetFrame_REF_to_abs()* POS_rod2link_rod).x() << "\t" << (rod->GetFrame_REF_to_abs()* POS_rod2link_rod).z() << std::endl;
		link->SetFrame_REF_to_abs(ChFrame<>(rod->GetFrame_REF_to_abs()* POS_rod2link_rod, q4));// rot4 absolute coords, q4 relative coords (formulation)
		link->SetFrame_COG_to_REF(ChFrame<>(COG_link, QUNIT));
		std::cout << "ABS Frame: " << POS_link2bucket.x() << "\t" << POS_link2bucket.z() << std::endl;
		std::cout << "LINK Frame: " << (link->GetFrame_REF_to_abs()* POS_link2bucket_link).x() << "\t" << (link->GetFrame_REF_to_abs()* POS_link2bucket_link).z() << std::endl;

		// Mass Properties
		link->SetMass(56.0);
		link->SetInertiaXX(ChVector<>(3.2, 11.1, 13.6));
		link->SetInertiaXY(ChVector<>(0.0, 0.0, -.04));
		// Primitive Visualization :
		auto link_asset = std::make_shared<ChCylinderShape>();
		link_asset->GetCylinderGeometry().rad = .025;
		link_asset->GetCylinderGeometry().p1 = link->GetFrame_COG_to_abs().GetInverse() * POS_rod2link;
		link_asset->GetCylinderGeometry().p2 = link->GetFrame_COG_to_abs().GetInverse() * POS_link2bucket;
		//link->AddAsset(link_asset);

		// Visualization Color
		auto col_l1 = std::make_shared<ChColorAsset>();
		col_l1->SetColor(ChColor(0.0f, 0.2f, 0.2f));
		link->AddAsset(col_l1);
		
		// Mesh Visualization
		geometry::ChTriangleMeshConnected connectingrod_mesh;
		connectingrod_mesh.LoadWavefrontMesh(out_dir + "data/ZK-L550-connectingrod.obj", false, false);// 
		auto connectingrod_mesh_shape = std::make_shared<ChTriangleMeshShape>();
		connectingrod_mesh_shape->SetMesh(connectingrod_mesh);
		connectingrod_mesh_shape->SetName("connectingrod");
		link->AddAsset(connectingrod_mesh_shape);
		
		// Collision model(WIP).
		link->SetCollide(true);
		link->GetCollisionModel()->ClearModel();
		utils::AddCylinderGeometry(link.get(), .025, Vlength(rod->GetFrame_REF_to_abs()* POS_rod2link_rod - link->GetFrame_REF_to_abs()* POS_link2bucket_link) / 2, link->GetFrame_COG_to_abs().GetInverse() * (link->GetFrame_REF_to_abs()* POS_link2bucket_link), y2x >> q4, false);// rel
		link->GetCollisionModel()->SetFamily(3);
		//link>SetFamilyMaskNoCollisionWithFamily(1);
		link->GetCollisionModel()->BuildModel();


		//	// BUCKET
		// It's the body handles the material.
		// It has two revolutes(in reality three), one with LINK BODY and the other with LIFT BODY(in reality two, but they're parallel considering x-y perspective, hence summarized in once in x-z plane.)
		bucket = std::shared_ptr<ChBodyAuxRef>(system.NewBodyAuxRef());
		system.AddBody(bucket);
		bucket->SetName("benna");
		bucket->SetIdentifier(4);
		bucket->SetMass(1305.0);//confirmed data
		bucket->SetInertiaXX(ChVector<>(613+866, 433+866, 613+866));//not confirmed data
		bucket->SetInertiaXY(ChVector<>(0, 0 , -433));//not confirmed data
		//bucket->SetPos(POS_lift2bucket);
		//bucket->SetFrame_COG_to_REF(ChFrame<>(ChVector<>(.1, .0, .1), QUNIT));
		
		bucket->SetFrame_REF_to_abs(ChFrame<>(POS_lift2bucket, QUNIT));
		bucket->SetFrame_COG_to_REF(ChFrame<>(COG_bucket, QUNIT));

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

		// Mesh Visualization
		geometry::ChTriangleMeshConnected bucket_mesh;
		bucket_mesh.LoadWavefrontMesh(out_dir + "data/bucket_mod.obj", false, false);
		auto bucket_mesh_shape = std::make_shared<ChTriangleMeshShape>();
		bucket_mesh_shape->SetMesh(bucket_mesh);
		bucket_mesh_shape->SetName("bucket");
		bucket->AddAsset(bucket_mesh_shape);


		// CHASSIS
		// It's a fake body representing the rest of the vehicle in first simulations.
		chassis = std::shared_ptr<ChBody>(system.NewBody());
		system.AddBody(chassis);
		//chassis->SetBodyFixed(true);//temporary
		chassis->SetName("chassis");
		chassis->SetIdentifier(0);
		chassis->SetMass(20.0);
		chassis->SetPos(COG_chassis);
		chassis->SetPos_dt(ChVector<>(.0, .0, .0));// u=2.25m/s
		chassis->SetInertiaXX(ChVector<>(500., 1000., 500.));
		// visualization properties
		auto chassis_asset = std::make_shared<ChSphereShape>();//asset
		chassis_asset->GetSphereGeometry().rad = .05;//asset
		//chassis->AddAsset(chassis_asset);

		



		/////////////////////////////////////------------------Add joint constraints------------------////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		// Revolute Joint between the lift body and the rod, located near the geometric baricenter(int(r dA)/int(dA)) of the rod LIFT-ROD. 
		// // It is the second of the three joints that interest the rod body(also known as rocker arm).
		rev_lift2rod = std::make_shared<ChLinkLockRevolute>();
		rev_lift2rod->SetName("revolute_lift2rod");
		rev_lift2rod->Initialize(rod, lift, ChCoordsys<>(lift->GetFrame_REF_to_abs()*POS_lift2rod_lift, z2y >> rot3.Get_A_quaternion()));		// z-dir default rev axis is rotated on y-dir
		system.AddLink(rev_lift2rod);

		// Revolute Joint between the rod body and link, last of the three joints interesting the rod , rear joint of the two concerning the link.
		rev_rod2link = std::make_shared<ChLinkLockRevolute>();
		rev_rod2link->SetName("revolute_rod2link");
		ChMatrix33<> rotb44; rotb44.Set_A_axis(VECT_X,VECT_Y,VECT_Z);
		rev_rod2link->Initialize(link, rod, ChCoordsys<>(rod->GetFrame_REF_to_abs()* POS_rod2link_rod, z2y >> rotb44.Get_A_quaternion()));		// z-dir default rev axis is rotated on y-dir 
		system.AddLink(rev_rod2link);

		// Definition of the two rev joints on the bucket: their distance is logged out in next line(it should be .437m)
		// At the moment, both ref frames have same initial orientation taking the connecting dir btw the two revs as the z-axis 
		std::cout << " Bucket revs distance : " << Vlength(POS_lift2bucket - link->GetFrame_REF_to_abs()* POS_link2bucket_link) << std::endl;
		// Coordinates Definition
		ChVector<> wb1 = (link->GetFrame_REF_to_abs()* POS_link2bucket_link - POS_lift2bucket).GetNormalized();
		ChVector<> ub1 = Vcross(VECT_Y,wb1).GetNormalized();
		ChMatrix33<> rotb1;
		rotb1.Set_A_axis(ub1, VECT_Y, wb1);
		// LIFT-BUCKET revjoint
		rev_lift2bucket = std::make_shared<ChLinkLockRevolute>();
		rev_lift2bucket->SetName("revolute_lift2bucket");
		rev_lift2bucket->Initialize(bucket, lift, false, ChCoordsys<>(POS_lift2bucket, z2y >> QUNIT), ChCoordsys<>(POS_lift2bucket, z2y >> rotb1.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir	// m2 yields the rotation in the getter
		system.AddLink(rev_lift2bucket);

		// LINK-BUCKET revjoint
		rev_link2bucket = std::make_shared<ChLinkLockRevolute>();
		rev_link2bucket->SetName("revolute_link2bucket");
		rev_link2bucket->Initialize(bucket, link, false, ChCoordsys<>(link->GetFrame_REF_to_abs()* POS_link2bucket_link, z2y >> QUNIT), ChCoordsys<>(link->GetFrame_REF_to_abs()* POS_link2bucket_link, z2y >> rotb1.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir
		system.AddLink(rev_link2bucket);
		// End of Definition of the two rev joints on the bucket.
		// End.

		// CHASSIS-LIFT revjoint
		rev_ch2lift = std::make_shared<ChLinkLockRevolute>();
		rev_ch2lift->SetName("revolute_chassis2lift");
		rev_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(POS_ch2lift, z2y >> QUNIT), ChCoordsys<>(POS_ch2lift, z2y >> rot1.Get_A_quaternion()));	// z-dir default rev axis is rotated on y-dir;// m2 yields the rotation in the getter																																							
		system.AddLink(rev_ch2lift);


		//-----------------------------------------------------------------------------------------------------//
		//-----------------------------------------------------------------------------------------------------//
		//-----------------------------------------------------------------------------------------------------//
		//-----------------------------------------------------------------------------------------------------//


		// Linear Actuator between the lift body and the so called rod(also known as rocker arm)
		ChVector<> u11 = (rod->GetFrame_REF_to_abs()*POS_lift2lever_rod - PIS_lift2lever).GetNormalized();		//GetNormalized() yields a unit vector(versor)
		ChVector<> w11 = Vcross(u11, VECT_Y).GetNormalized();					//overkill
		ChMatrix33<> rot11;														//no control on orthogonality, IT'S UP TO THE USER'
		rot11.Set_A_axis(u11, VECT_Y, w11);


		lin_lift2rod = std::make_shared<ChLinkLinActuator>();
		//		lin_lift2rod = std::unique_ptr<ChLinkMarkers>(new ChLinkLinActuator());
		lin_lift2rod->SetName("linear_lift2rod");
		lin_lift2rod->Initialize(rod, lift, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
			//lin_lift2rod->Set_lin_offset(Vlength(POS_lift2lever - PIS_lift2lever));
			lin_lift2rod->Set_lin_offset(0.0);
		std::cout << "Tilt offset : " << lin_lift2rod->Get_lin_offset() << std::endl;

		
		// Line brought outside in main.cpp
		//lin_lift2rod->Set_dist_funct(tdisplacement);

		// Asset for the linear actuator
		//lin_lift2rod->AddAsset(std::make_shared<ChPointPointSegment>());

		system.AddLink(lin_lift2rod);
		//-----------------------------------------------------------------------------------------------------//
		//-----------------------------------------------------------------------------------------------------//
		//-----------------------------------------------------------------------------------------------------//
		//-----------------------------------------------------------------------------------------------------//



		// CHASSIS-LIFT linear actuator--Lift Piston
		ChVector<> u22 = (INS_ch2lift - PIS_ch2lift).GetNormalized();					//GetNormalized()
		ChVector<> w22 = Vcross(u22, VECT_Y).GetNormalized();							//overkill
		ChMatrix33<> rot22;																//no control on orthogonality, IT'S UP TO THE USER
		rot22.Set_A_axis(u22, VECT_Y, w22);


		lin_ch2lift = std::make_shared<ChLinkLinActuator>();
		lin_ch2lift->SetName("linear_chassis2lift");
		lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(INS_ch2lift, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, z2x >> rot22.Get_A_quaternion()));//m2 is the master
			//lin_ch2lift->Set_lin_offset(Vlength(INS_ch2lift - PIS_ch2lift));
			lin_ch2lift->Set_lin_offset(0.0);
		std::cout << "Lift offset : " << lin_ch2lift->Get_lin_offset() << std::endl;
		//	//	ASSET FOR LINEAR ACTUATOR
		//lin_ch2lift->AddAsset(std::make_shared<ChPointPointSegment>());
		// Line brought outside in main.cpp
		//lin_ch2lift->Set_dist_funct(ldisplacement);
		system.Add(lin_ch2lift);

#ifdef USE_PNEUMATIC
		
		// Hydraulic Force calculation-input
		std::vector<TimeSeries> ReadHeadPressure;
		ReadPressureFile("../data/HeadLiftPressure.dat", ReadHeadPressure);
		auto lhpressure = std::make_shared<ChFunction_Recorder>();
		for (int i = 0; i < ReadHeadPressure.size(); i++){
			lhpressure->AddPoint(ReadHeadPressure[i].mt, 1.75e5*ReadHeadPressure[i].mv);//2 pistons,data in [bar]
		}
		std::vector<TimeSeries> ReadRodPressure;
		ReadPressureFile("../data/RodLiftPressure.dat", ReadRodPressure);
		auto lrpressure = std::make_shared<ChFunction_Recorder>();
		for (int i = 0; i < ReadRodPressure.size(); i++){
			lrpressure->AddPoint(ReadRodPressure[i].mt, 1.75e5*ReadRodPressure[i].mv);// 2 pistons, data in [bar]
		}


		// Using prismatic connection oriented as pneumatic actuator, the system does not move(hyper-constrained).

		//auto prism_lin_lift2rod = std::make_shared<ChLinkLockPrismatic>();
		//prism_lin_lift2rod->Initialize(rod, chassis, false, ChCoordsys<>(POS_lift2lever, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
		//system.AddLink(prism_lin_lift2rod);


		auto thpressure = std::make_shared<ChFunction_Recorder>();
		for (int i = 0; i < ReadHeadPressure.size(); i++){
			thpressure->AddPoint(ReadHeadPressure[i].mt, .2e5*ReadHeadPressure[i].mv);//2 pistons,data in [bar]
		}
		auto trpressure = std::make_shared<ChFunction_Recorder>();
		for (int i = 0; i < ReadRodPressure.size(); i++){
			trpressure->AddPoint(ReadRodPressure[i].mt, .2e5*ReadRodPressure[i].mv);// 2 pistons, data in [bar]
		}


		auto tforce = std::make_shared<myHYDRforce>();

		// Using only the pneumatic actuator, weight of the arms makes them oscillating like a pendulum, hence real pressures are not enough to lift the system.
		lin_lift2rod = std::make_shared<myHYDRactuator>();
		lin_lift2rod->SetName("linear_lift2rod");// ChLinkMarkers child, force applied on slave m1
		lin_lift2rod->Initialize(rod, chassis, false, ChCoordsys<>(rod->GetFrame_REF_to_abs()*POS_lift2lever_rod, z2x >> rot11.Get_A_quaternion()), ChCoordsys<>(PIS_lift2lever, z2x >> rot11.Get_A_quaternion()));//m2 is the master
		lin_lift2rod->Set_HYDRforce(tforce);
		lin_lift2rod->Set_PressureH(thpressure);
		lin_lift2rod->Set_PressureR(trpressure);
		lin_lift2rod->SetAreaH(CH_C_PI / 4 * pow(.150, 2));// Head Side, Diameter: 150mm
		lin_lift2rod->SetAreaR(CH_C_PI / 4 * (pow(.150, 2) - pow(.080, 2)));// Piston Rod, Diameter: 80mm
		// Attach a visualization asset.
		lin_lift2rod->AddAsset(std::make_shared<ChPointPointSpring>(0.05, 80, 15));

		


		auto lforce = std::make_shared<myHYDRforce>();
		
		


		// Pneum Act:, weight of the arms makes them oscillating like a pendulum, hence real pressures are not enough to lift the system.

		lin_ch2lift = std::make_shared<myHYDRactuator>();
		lin_ch2lift->SetName("linear_chassis2lift");// ChLinkMarkers child, force applied on slave m1
		lin_ch2lift->Initialize(lift, chassis, false, ChCoordsys<>(INS_ch2lift, z2x >> rot22.Get_A_quaternion()), ChCoordsys<>(PIS_ch2lift, z2x >> rot22.Get_A_quaternion()));//m2 is the master
		lin_ch2lift->SetAreaH(CH_C_PI / 4 * pow(.130,2));// Head Side, Diameter: 130mm
		lin_ch2lift->SetAreaR(CH_C_PI / 4 * ( pow(.130, 2) - pow(.080,2)));// Piston Rod, Diameter: 80mm
		lin_ch2lift->Set_HYDRforce(lforce);// CH_C_1_PI was wrong!
		lin_ch2lift->Set_PressureH(lhpressure);
		lin_ch2lift->Set_PressureR(lrpressure);
		// Attach a visualization asset.
		lin_ch2lift->AddAsset(std::make_shared<ChPointPointSpring>(0.05, 80, 15));
		system.AddLink(lin_ch2lift);
		
#endif

	}
	// Destructor
	~MyWheelLoader(){}

	// Getters
	// Get the whole mechanism mass.
	double GetMass(){return	lift->GetMass() + link->GetMass() + rod->GetMass() + bucket->GetMass();	}
	// Get the relative(wrt ground x-z plane) angle of lifting
	double GetLiftArmPitchAngle(){ return this->rev_ch2lift->GetRelAngle(); }
	// Get the relative(wrt ground x-z plane) angle of tilting
	double GetTiltBucketPitchAngle(){ return this->rev_lift2bucket->GetRelAngle(); }
	// Get the reaction force on piston(sure?), lifting
	double GetReactLiftForce(){ return this->lin_ch2lift->Get_react_force().x(); }
	// Get the reaction force on piston(sure?), tilting
	double GetReactTiltForce(){ return this->lin_lift2rod->Get_react_force().x(); }

	// Setters
	void SetPistonLiftImposedMotion(std::shared_ptr<ChFunction> funct){ this->lin_ch2lift->Set_dist_funct(funct); }
	void SetPistonTiltImposedMotion(std::shared_ptr<ChFunction> funct){ this->lin_lift2rod->Set_dist_funct(funct); }

};

