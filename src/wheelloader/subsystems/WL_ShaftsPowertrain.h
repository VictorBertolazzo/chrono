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
// Authors:
// =============================================================================
//
// WL powertrain model based on ChShaft objects.
//
// =============================================================================

#ifndef WL_SHAFTS_POWERTRAIN_H
#define WL_SHAFTS_POWERTRAIN_H

#include "chrono_vehicle/ChVehicle.h"
#include "chrono_vehicle/powertrain/ChShaftsPowertrain.h"

#include "chrono_models/ChApiModels.h"

/// @addtogroup vehicle_models_WL
/// @{

/// Shafts-based powertrain model for the WL vehicle.
class WL_ShaftsPowertrain : public chrono::vehicle::ChShaftsPowertrain {
  public:
    WL_ShaftsPowertrain();

    ~WL_ShaftsPowertrain() {}

    virtual void SetGearRatios(std::vector<double>& gear_ratios) override;

    virtual double GetMotorBlockInertia() const override { return m_motorblock_inertia; }
    virtual double GetCrankshaftInertia() const override { return m_crankshaft_inertia; }
    virtual double GetIngearShaftInertia() const override { return m_ingear_shaft_inertia; }

    virtual void SetEngineTorqueMap(std::shared_ptr<chrono::ChFunction_Recorder>& map) override;
    virtual void SetEngineLossesMap(std::shared_ptr<chrono::ChFunction_Recorder>& map) override;
    virtual void SetTorqueConverterCapacityFactorMap(std::shared_ptr<chrono::ChFunction_Recorder>& map) override;
    virtual void SetTorqeConverterTorqueRatioMap(std::shared_ptr<chrono::ChFunction_Recorder>& map) override;

  private:
    // Shaft inertias.
    static const double m_motorblock_inertia;
    static const double m_crankshaft_inertia;
    static const double m_ingear_shaft_inertia;
};

/// @} vehicle_models_WL


#endif
