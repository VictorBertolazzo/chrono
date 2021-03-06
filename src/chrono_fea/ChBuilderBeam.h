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
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CHBUILDERBEAM_H
#define CHBUILDERBEAM_H

#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChElementBeamEuler.h"
#include "chrono_fea/ChElementCableANCF.h"

#include "chrono/physics/ChBody.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChLinkMotor.h"

namespace chrono {
namespace fea {

class ChContactSurfaceNodeCloud;

/// Class for an helper object that provides easy functions to create
/// complex beams, for example subdivides a segment in multiple finite
/// elements.
class ChApiFea ChBuilderBeam {
  protected:
    std::vector<std::shared_ptr<ChElementBeamEuler> > beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > beam_nodes;

  public:
    /// Helper function.
    /// Adds beam FEM elements to the mesh to create a segment beam
    /// from point A to point B, using ChElementBeamEuler type elements.
    /// Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   const ChVector<> A,                           ///< starting point
                   const ChVector<> B,                           ///< ending point
                   const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
                   );

    /// Helper function.
    /// Adds beam FEM elements to the mesh to create a segment beam
    /// from one existing node to another existing node, using ChElementBeamEuler type elements.
    /// Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                   std::shared_ptr<ChNodeFEAxyzrot> nodeB,       ///< ending point
                   const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
                   );

    /// Helper function.
    /// Adds beam FEM elements to the mesh to create a segment beam
    /// from one existing node to a point B, using ChElementBeamEuler type elements.
    /// Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,                 ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionAdvanced> sect,  ///< section material for beam elements
                   const int N,                                  ///< number of elements in the segment
                   std::shared_ptr<ChNodeFEAxyzrot> nodeA,       ///< starting point
                   const ChVector<> B,                           ///< ending point
                   const ChVector<> Ydir                         ///< the 'up' Y direction of the beam
                   );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a 'Build...' function is called.
    std::vector<std::shared_ptr<ChElementBeamEuler> >& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a 'Build...' function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> >& GetLastBeamNodes() { return beam_nodes; }
};

/// Class for an helper object that provides easy functions to create
/// complex beams of ChElementCableANCF class, for example subdivides a segment
/// in multiple finite elements.

class ChApiFea ChBuilderBeamANCF {
  protected:
    std::vector<std::shared_ptr<ChElementCableANCF> > beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzD> > beam_nodes;

  public:
    /// Helper function.
    /// Adds beam FEM elements to the mesh to create a segment beam
    /// from point A to point B, using ChElementCableANCF type elements.
    /// Before running, each time resets lists of beam_elems and beam_nodes.
    void BuildBeam(std::shared_ptr<ChMesh> mesh,              ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionCable> sect,  ///< section material for beam elements
                   const int N,                               ///< number of elements in the segment
                   const ChVector<> A,                        ///< starting point
                   const ChVector<> B                         ///< ending point
                   );

    /// Access the list of elements used by the last built beam.
    /// It can be useful for changing properties afterwards.
    /// This list is reset all times a 'Build...' function is called.
    std::vector<std::shared_ptr<ChElementCableANCF> >& GetLastBeamElements() { return beam_elems; }

    /// Access the list of nodes used by the last built beam.
    /// It can be useful for adding constraints or changing properties afterwards.
    /// This list is reset all times a 'Build...' function is called.
    std::vector<std::shared_ptr<ChNodeFEAxyzD> >& GetLastBeamNodes() { return beam_nodes; }
};



/// Class for object that continuously extrude a beam
/// with prescribed velocity

class ChApiFea ChExtruderBeamEuler {
  protected:
    std::vector<std::shared_ptr<ChElementBeamEuler> > beam_elems;
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > beam_nodes;

    std::shared_ptr<ChBody> ground;
    std::shared_ptr<ChLinkMotorLinearSpeed> actuator;
    std::shared_ptr<ChLinkMateGeneric> guide;

    ChSystem* mysystem; 
    std::shared_ptr<ChMesh> mesh;
    
    std::shared_ptr<ChBeamSectionAdvanced> beam_section;
    double h;                                  
    ChCoordsys<> outlet;                                         
    double mytime;
    double speed;
  
    std::shared_ptr<ChMaterialSurfaceSMC> contact_material;

    std::shared_ptr<ChContactSurfaceNodeCloud> contactcloud;
    double contact_radius;

  public:
    /// Initialize and add required constraints to system
    ChExtruderBeamEuler(
                    ChSystem* msystem,         ///< system to store the constraints
                    std::shared_ptr<ChMesh> mmesh,             ///< mesh to store the resulting elements
                   std::shared_ptr<ChBeamSectionAdvanced> sect,///< section material for beam elements
                   double mh,                                  ///< element length
                   const ChCoordsys<> moutlet,                 ///< outlet pos & orientation (x is extrusion direction)
                   double mspeed                               ///< speed 
                   );

    ~ChExtruderBeamEuler();

    /// Sets the material for the beam, and enables collision detection for the beam nodes.
    /// By default, collision not enabled.
    void SetContact(
            std::shared_ptr<ChMaterialSurfaceSMC> mcontact_material, ///< material to use for surface
            double mcontact_radius  ///< radius of colliding spheres at each node (usually = to avg.beam thickness)
            );    

    /// Create beam elements, if needed, and update the constraint that 
    /// imposes the extrusion speed
    void Update();

    /// Access the list of created elements
    std::vector<std::shared_ptr<ChElementBeamEuler> >& GetLastBeamElements() { return beam_elems; }

    /// Access the list of created nodes 
    std::vector<std::shared_ptr<ChNodeFEAxyzrot> >& GetLastBeamNodes() { return beam_nodes; }
};




}  // end namespace fea
}  // end namespace chrono

#endif
