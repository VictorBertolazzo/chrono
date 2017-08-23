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
// Modified: Victor Bertolazzo
// =============================================================================
//
// Simple driveline model for a single axle open differential.
// Modified for a 4WD Driveline.
// =============================================================================

#include <cmath>

#include "WL_SimpleDriveline4WD.h"

using namespace chrono;
using namespace chrono::vehicle;

			// -----------------------------------------------------------------------------
			// Static variables
			// -----------------------------------------------------------------------------
			const double WL_SimpleDriveline4WD::m_conicalgear_ratio = -0.2433;

			// -----------------------------------------------------------------------------
			// Construct a 2WD open differential simple driveline.
			// -----------------------------------------------------------------------------
			WL_SimpleDriveline4WD::WL_SimpleDriveline4WD(const std::string& name) : ChDriveline(name) {}

			// -----------------------------------------------------------------------------
			// Initialize the driveline subsystem.
			// This function connects this driveline subsystem to the axles of the specified
			// suspension subsystems.
			// -----------------------------------------------------------------------------
			void WL_SimpleDriveline4WD::Initialize(std::shared_ptr<ChBody> chassis,
				const ChSuspensionList& suspensions,
				const std::vector<int>& driven_axles) {
				assert(suspensions.size() == 2);

				m_driven_axles = driven_axles;

				// Grab handles to the suspension wheel shafts.
				m_driven_front_left = suspensions[0]->GetAxle(LEFT);
				m_driven_front_right = suspensions[0]->GetAxle(RIGHT);

				m_driven_rear_left = suspensions[1]->GetAxle(LEFT);
				m_driven_rear_right = suspensions[1]->GetAxle(RIGHT);

			}

			// -----------------------------------------------------------------------------
			// -----------------------------------------------------------------------------
			double WL_SimpleDriveline4WD::GetDriveshaftSpeed() const {
				double wheel_speed = 0.25 * (m_driven_front_left->GetPos_dt() + m_driven_front_right->GetPos_dt() + m_driven_rear_left->GetPos_dt() + m_driven_rear_right->GetPos_dt());

				return (wheel_speed / m_conicalgear_ratio);
			}

			// -----------------------------------------------------------------------------
			// -----------------------------------------------------------------------------
			void WL_SimpleDriveline4WD::Synchronize(double torque) {
				// Split the input torque front/back.
				double torque_drive = -torque / m_conicalgear_ratio;

				m_driven_front_left->SetAppliedTorque(-torque_drive / 4);
				m_driven_front_right->SetAppliedTorque(-torque_drive / 4);

				m_driven_rear_left->SetAppliedTorque(-torque_drive / 4);
				m_driven_rear_right->SetAppliedTorque(-torque_drive / 4);


			}

			// -----------------------------------------------------------------------------
			// -----------------------------------------------------------------------------
			double WL_SimpleDriveline4WD::GetWheelTorque(const WheelID& wheel_id) const {
				if (wheel_id.axle() == 0) {
					switch (wheel_id.side()) {
					case LEFT:
						return -m_driven_front_left->GetAppliedTorque();
					case RIGHT:
						return -m_driven_front_right->GetAppliedTorque();
					}
				}
				if (wheel_id.axle() == 1) {
					switch (wheel_id.side()) {
					case LEFT:
						return -m_driven_rear_left->GetAppliedTorque();
					case RIGHT:
						return -m_driven_rear_right->GetAppliedTorque();
					}
				}

				return 0;
			}
