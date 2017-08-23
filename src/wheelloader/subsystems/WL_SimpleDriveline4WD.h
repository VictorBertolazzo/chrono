// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban, Mike Taylor
// =============================================================================
// Authors: Victor Bertolazzo
// =============================================================================
//
// Simple driveline model for a single axle open differential.
// Modified for a 4WD Driveline.
// =============================================================================

#ifndef WL_SIMPLE_DRIVELINE_4WD_H
#define WL_SIMPLE_DRIVELINE_4WD_H

#include "chrono_vehicle/wheeled_vehicle/ChDriveline.h"
#include "chrono_models/ChApiModels.h"


			/// @addtogroup vehicle_models_generic
			/// @{

			/// Simple driveline model for the generic vehicle (purely kinematic).
			class WL_SimpleDriveline4WD : public chrono::vehicle::ChDriveline {
			public:
				WL_SimpleDriveline4WD(const std::string& name);
				virtual ~WL_SimpleDriveline4WD() {}

				/// Return the number of driven axles.
				virtual int GetNumDrivenAxles() const final override { return 2; }

				/// Initialize the driveline subsystem.
				/// This function connects this driveline subsystem to the axles of the
				/// specified suspension subsystems.
				virtual void Initialize(std::shared_ptr<chrono::ChBody> chassis,      ///< handle to the chassis body
					const chrono::vehicle::ChSuspensionList& suspensions,  ///< list of all vehicle suspension subsystems
					const std::vector<int>& driven_axles  ///< indexes of the driven vehicle axles(ignored)
					) override;

				/// Get the angular speed of the driveshaft.
				/// This represents the output from the driveline subsystem that is passed to
				/// the powertrain system.
				virtual double GetDriveshaftSpeed() const override;

				/// Update the driveline subsystem: apply the specified motor torque.
				/// This represents the input to the driveline subsystem from the powertrain
				/// system.
				virtual void Synchronize(double torque) override;

				/// Get the motor torque to be applied to the specified wheel.
				virtual double GetWheelTorque(const chrono::vehicle::WheelID& wheel_id) const override;

			private:
				static const double m_conicalgear_ratio;

				std::shared_ptr<chrono::ChShaft> m_driven_front_left;
				std::shared_ptr<chrono::ChShaft> m_driven_front_right;

				std::shared_ptr<chrono::ChShaft> m_driven_rear_left;
				std::shared_ptr<chrono::ChShaft> m_driven_rear_right;


			};

			/// @} vehicle_models_generic


#endif
