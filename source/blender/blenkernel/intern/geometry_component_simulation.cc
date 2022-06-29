/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BKE_geometry_set.hh"
#include "BKE_lib_id.h"

/* -------------------------------------------------------------------- */
/** \name Geometry Component Implementation
 * \{ */

SimulationComponent::SimulationComponent() : GeometryComponent(GEO_COMPONENT_TYPE_SIMULATION)
{
}

SimulationComponent::~SimulationComponent()
{
}

GeometryComponent *SimulationComponent::copy() const
{
  SimulationComponent *new_component = new SimulationComponent();
  return new_component;
}

blender::VectorSet<SimulationComponent::ShapeConstructionInfo> &SimulationComponent::shapes()
{
  return shapes_;
}

const blender::VectorSet<SimulationComponent::ShapeConstructionInfo> &SimulationComponent::shapes() const
{
  return shapes_;
}

void SimulationComponent::clear()
{
  shapes_.clear();
}

int SimulationComponent::add_shape(const ShapeConstructionInfo &info)
{
  return shapes_.index_of_or_add_as(info);
}

bool SimulationComponent::owns_direct_data() const
{
  for (const ShapeConstructionInfo &shape : shapes_) {
    if (!shape.owns_direct_data()) {
      return false;
    }
  }
  return true;
}

void SimulationComponent::ensure_owns_direct_data()
{
  BLI_assert(this->is_mutable());
  for (const ShapeConstructionInfo &const_shape : shapes_) {
    /* Const cast is fine because we are not changing anything that would change the hash of the
     * reference. */
    ShapeConstructionInfo &shape = const_cast<ShapeConstructionInfo &>(const_shape);
    shape.ensure_owns_direct_data();
  }
}

/** \} */
