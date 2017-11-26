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
//
// =============================================================================

#ifndef WL_FIALA_TIRE_H
#define WL_FIALA_TIRE_H

#include "chrono/assets/ChTriangleMeshShape.h"

#include "chrono_vehicle/wheeled_vehicle/tire/ChFialaTire.h"

#include "chrono_models/ChApiModels.h"

using namespace chrono;
using namespace chrono::vehicle;

/// @addtogroup vehicle_models_WL
/// @{

/// Fiala tire model for the WL vehicle.
class WL_FialaTire : public ChFialaTire {
  public:
    WL_FialaTire(const std::string& name);
    ~WL_FialaTire() {}

    virtual double GetNormalStiffnessForce(double depth) const override;
    virtual double GetNormalDampingForce(double depth, double velocity) const override {
        return m_normalDamping * velocity;
    }

    virtual double GetVisualizationWidth() const override { return m_width; }

    virtual void SetFialaParams() override;

    virtual void AddVisualizationAssets(VisualizationType vis) override;
    virtual void RemoveVisualizationAssets() override final;

  private:
    static const double m_normalDamping;
	static const double m_normalStiffness;

    static const std::string m_meshName;
    static const std::string m_meshFile;
    std::shared_ptr<ChTriangleMeshShape> m_trimesh_shape;
};

/// @} vehicle_models_WL

#endif
