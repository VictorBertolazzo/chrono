using namespace chrono;
using namespace chrono::collision;
// --------------------------------------------------------------------------------------------------------

#include "chrono/collision/ChCCollisionUtils.h"

	// Class implemented to set up a hydraulic/data input force between two markers
	class myHYDRactuator : public ChLinkMarkers{

	public:
		myHYDRactuator() : m_force(0), m_force_fun(NULL), m_pressureH(NULL), m_pressureR(NULL), m_areaH(5e-3), m_areaR(10) {};
		myHYDRactuator(const myHYDRactuator& other);
		virtual ~myHYDRactuator() {}

		/// Class to be used as a functor interface for calculating the general spring-damper force.
		/// A derived class must implement the virtual operator().
		class ForceFunctor {
		public:
			virtual ~ForceFunctor() {}

			/// Calculate and return the general spring-damper force at the specified configuration.
			virtual double operator()(double time,          ///< current time
				double pressureH,   ///< piston head-side pressure
				double areaH,
				myHYDRactuator* link // back-pointer
				) = 0;
		};

		void SetAreaH(double areaH){ m_areaH = areaH; }
		void SetAreaR(double areaR){ m_areaR = areaR; }
		double GetAreaH(){ return m_areaH; }
		double GetAreaR(){ return m_areaR; }

		void Set_HYDRforce(ForceFunctor* force) { m_force_fun = force; }
		void Set_PressureH(ChFunction* pressureH){ m_pressureH = pressureH; }
		void Set_PressureR(ChFunction* pressureR){ m_pressureR = pressureR; }



		void UpdateForces(double time)override{

			// Allow the base class to update itself (possibly adding its own forces)
			ChLinkMarkers::UpdateForces(time);

			// Get the pressure information
			double pressureH = this->m_pressureH->Get_y(time);
			// Invoke the provided functor to evaluate force
			m_force = m_force_fun ? (*m_force_fun)(time, pressureH, m_areaH, this) : 0;

			// Add to existing force.
			C_force += m_force * relM.pos.GetNormalized();

		}
	protected:
		ForceFunctor* m_force_fun;  ///< functor for force calculation
		double m_areaH = 10.;
		double m_areaR = 9.;
		double m_force;  ///< force functor double value
		ChFunction* m_pressureH;
		ChFunction* m_pressureR;

	};


// Class implemented to compute hydraulic force
class myHYDRforce : public myHYDRactuator::ForceFunctor{
public:
	virtual ~myHYDRforce() {}


	virtual double operator()(double time,
		double pressureH,
		double areaH,
		myHYDRactuator* link
		)override
	{
		double force = pressureH * areaH; //pH*aH-pR*aR, formula for piston
		return force;
	}

};
