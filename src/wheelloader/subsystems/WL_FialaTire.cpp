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
// Original Template: Radu Serban, Michael Taylor
// =============================================================================
// Modified: Victor Bertolazzo
// =============================================================================
//
// WL Fiala subsystem
//
// =============================================================================

#include <cmath>
#include <algorithm>

#include "chrono_vehicle/ChVehicleModelData.h"
#include "WL_FialaTire.h"

using namespace chrono;
using namespace chrono::vehicle;
// -----------------------------------------------------------------------------
// Static variables
// -----------------------------------------------------------------------------

//const double WL_FialaTire::m_normalStiffness = 707.5;
const double WL_FialaTire::m_normalDamping = 3100;//2.5869


const std::string WL_FialaTire::m_meshName = "WL_tire_POV_geom";
const std::string WL_FialaTire::m_meshFile = "HMMWV/HMMWV_tire.obj";

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WL_FialaTire::WL_FialaTire(const std::string& name) : ChFialaTire(name) {}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WL_FialaTire::SetFialaParams() {
	m_unloaded_radius = 0.5 * 63.5 * 0.0254;// ~ 806mm;
	m_width = 23.6 * 0.0254;// ~ 600mm;
    m_rolling_resistance = 0.015;  // Assume very small since no other data exists
    m_c_slip = 52300.24;
    m_c_alpha = 5000.6;
    m_u_min = 0.700;
    m_u_max = 0.800;
    m_relax_length_x = 0.1244;
    m_relax_length_y = 0.0317;
}

double WL_FialaTire::GetNormalStiffnessForce(double depth) const {
    // corresponding depths = 0 : 0.00254 : 0.08128
    double normalforcetabel[33] = {0,
                                   296.562935405400,
                                   611.630472750000,
                                   889.644324000000,
                                   1278.86371575000,
                                   1890.49418850000,
                                   2390.91912075000,
                                   2946.94682325000,
                                   3558.57729600000,
                                   4225.81053900000,
                                   4893.04378200000,
                                   5615.87979525000,
                                   6338.71580850000,
                                   7005.94905150000,
                                   7673.18229450000,
                                   8451.62107800000,
                                   9230.05986150000,
                                   10008.4986450000,
                                   10786.9374285000,
                                   11593.1775971250,
                                   12399.4177657500,
                                   13150.0551641250,
                                   13900.6925625000,
                                   14818.1382716250,
                                   15735.5839807500,
                                   16458.4199940000,
                                   17181.2560072500,
                                   18015.2975610000,
                                   18849.3391147500,
                                   19655.5792833750,
                                   20461.8194520000,
                                   21295.8610057500,
                                   22129.9025595000};

    depth = depth * (depth > 0);  // Ensure that depth is positive;

    double position = (33. * (depth / 0.08128));

    // Linear extrapolation if the depth is at or greater than the maximum depth in the table (0.08128)
    if (position >= 32) {
        return (22129.9025595000 + (depth - 0.08128) * 3.283628164370066e+05);
    }
    // Linearly interpolate between the table entries
    else {
        double scale = std::ceil(position) - position;
        return (normalforcetabel[int(std::floor(position))] * (1 - scale) +
                normalforcetabel[int(std::floor(position) + 1)] * scale);
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WL_FialaTire::AddVisualizationAssets(VisualizationType vis) {
    if (vis == VisualizationType::MESH) {
        geometry::ChTriangleMeshConnected trimesh;
        trimesh.LoadWavefrontMesh(vehicle::GetDataFile(m_meshFile), false, false);
        m_trimesh_shape = std::make_shared<ChTriangleMeshShape>();
        m_trimesh_shape->SetMesh(trimesh);
        m_trimesh_shape->SetName(m_meshName);
        m_wheel->AddAsset(m_trimesh_shape);
    }
    else {
        ChFialaTire::AddVisualizationAssets(vis);
    }
}

void WL_FialaTire::RemoveVisualizationAssets() {
    ChFialaTire::RemoveVisualizationAssets();

    // Make sure we only remove the assets added by WL_FialaTire::AddVisualizationAssets.
    // This is important for the ChTire object because a wheel may add its own assets
    // to the same body (the spindle/wheel).
    auto it = std::find(m_wheel->GetAssets().begin(), m_wheel->GetAssets().end(), m_trimesh_shape);
    if (it != m_wheel->GetAssets().end())
        m_wheel->GetAssets().erase(it);
}

