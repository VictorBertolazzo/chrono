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
// Authors: Radu Serban
// =============================================================================
//
// Generic wheel subsystem
//
// =============================================================================

#include "chrono_models/vehicle/generic/Generic_RigidTire.h"

namespace chrono {
namespace vehicle {
namespace generic {

// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

	const double Generic_RigidTire::m_radius = 0.5 * 63.5 * 0.0254;//0.3099;
	const double Generic_RigidTire::m_width =        23.6 * 0.0254;//0.235;

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
Generic_RigidTire::Generic_RigidTire(const std::string& name) : ChRigidTire(name) {
    SetContactFrictionCoefficient(0.9f);
    SetContactRestitutionCoefficient(0.1f);
    SetContactMaterialProperties(2e7f, 0.3f);
    SetContactMaterialCoefficients(2e5f, 40.0f, 2e5f, 20.0f);
}

}  // end namespace generic
}  // end namespace vehicle
}  // end namespace chrono
