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
// Simple powertrain model for the Articulated Vehicle.
// - both power and torque limited
// - no torque converter
// - no transmission box
//
// =============================================================================

#include "WL_SimpleMapPowertrain.h"

using namespace chrono;
using namespace chrono::vehicle;
// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
WL_SimpleMapPowertrain::WL_SimpleMapPowertrain()
    : ChPowertrain(),
      m_motorSpeed(0),
      m_motorTorque(0),
      m_shaftTorque(0),
	  m_gear_ratios({ -1./1.342,- 1. / 3.033, -1. / 6.03, 1e20, 1. / 1.603, 1. / 3.033, 1. / 1.342 }),// 7 Different gear ratios, 3RW,1N,3FW
      m_zeroThrottleMap({-100.,700.},
                        {0.,0.}),// "Idle throttle map is set to zero"
      m_fullThrottleMap({-100., 700., 800., 900., 1000., 1100., 1200., 1300., 1400., 1500., 1600.,
                         1700., 1800., 1900., 2000., 2100.},
                        {300., 550., 650., 840., 841., 845., 843., 841., 835., 828., 793., 
                         720., 697., 630., 585., -400.}) {}

//map->AddPoint(-100 * rpm_to_radsec, 300);  // to start engine
//map->AddPoint(2100 * rpm_to_radsec, -400);  // fading out of engine torque

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WL_SimpleMapPowertrain::Initialize(std::shared_ptr<ChBody> chassis, std::shared_ptr<ChShaft> driveshaft) {
    SetSelectedGear(1);
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WL_SimpleMapPowertrain::SetSelectedGear(int igear) {
    assert(igear >= 0);
    assert(igear < m_gear_ratios.size());

    m_current_gear = igear;
    m_current_gear_ratio = m_gear_ratios[igear +3];// SetSelectedGear receives a number [-3,+3].
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WL_SimpleMapPowertrain::SetDriveMode(ChPowertrain::DriveMode mode) {
    m_drive_mode = mode;
    switch (mode) {
        case FORWARD:
            SetSelectedGear(1);
            break;
        case REVERSE:
            SetSelectedGear(0);
            break;
        case NEUTRAL:
            m_current_gear_ratio = 1e20;
            break;
    }
}

// -----------------------------------------------------------------------------
// -----------------------------------------------------------------------------
void WL_SimpleMapPowertrain::Synchronize(double time, double throttle, double shaft_speed) {
    // The motor speed is the shaft speed multiplied by gear ratio inversed: (limited to 8000rpm)
    m_motorSpeed = shaft_speed / m_current_gear_ratio;
    m_motorSpeed = m_motorSpeed > (8000. * CH_C_PI / 30.) ? (8000. * CH_C_PI / 30.) : m_motorSpeed;
    m_motorSpeed = m_motorSpeed < 0.0 ? 0.0 : m_motorSpeed;

    // Motor torque is linearly interpolated by throttle gas value:
    double zeroThrottleTorque;
    double fullThrottleTorque;
    double curve_dot;   // not used
    double curve_ddot;  // not used
    m_zeroThrottleMap.Evaluate(m_motorSpeed, zeroThrottleTorque, curve_dot, curve_ddot);
    m_fullThrottleMap.Evaluate(m_motorSpeed, fullThrottleTorque, curve_dot, curve_ddot);

    m_motorTorque = zeroThrottleTorque * (1 - throttle) + fullThrottleTorque * (throttle);

    // The torque at motor shaft:
    m_shaftTorque = m_motorTorque / m_current_gear_ratio;
}

