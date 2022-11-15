/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_simplex_types.h"

#include "BKE_geometry_set.hh"
#include "BKE_mesh_simplex.hh"

/* -------------------------------------------------------------------- */
/** \name Geometry Component Implementation
 * \{ */

SimplexComponent::SimplexComponent() : GeometryComponent(GEO_COMPONENT_TYPE_SIMPLEX)
{
}

SimplexComponent::~SimplexComponent()
{
  this->clear();
}

GeometryComponent *SimplexComponent::copy() const
{
  SimplexComponent *new_component = new SimplexComponent();
  if (geometry_ != nullptr) {
    new_component->geometry_ = new blender::bke::SimplexGeometry(*geometry_);
    new_component->ownership_ = GeometryOwnershipType::Owned;
  }
  return new_component;
}

void SimplexComponent::clear()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::Owned) {
    delete geometry_;
  }
  geometry_ = nullptr;
}

bool SimplexComponent::is_empty() const
{
  if (geometry_ != nullptr) {
    if (geometry_->simplex_num() > 0) {
      return false;
    }
  }
  return true;
}

bool SimplexComponent::owns_direct_data() const
{
  if (geometry_ != nullptr) {
    /* TODO */
    //return geometry_->owns_direct_data();
    return true;
  }
  return true;
}

void SimplexComponent::ensure_owns_direct_data()
{
  if (geometry_ != nullptr) {
    //geometry_->ensure_owns_direct_data();
  }
}

const blender::bke::SimplexGeometry *SimplexComponent::get_for_read() const
{
  return geometry_;
}

blender::bke::SimplexGeometry *SimplexComponent::get_for_write()
{
  BLI_assert(this->is_mutable());
  if (ownership_ == GeometryOwnershipType::ReadOnly) {
    geometry_ = new blender::bke::SimplexGeometry(*geometry_);
    ownership_ = GeometryOwnershipType::Owned;
  }
  return geometry_;
}

void SimplexComponent::replace(blender::bke::SimplexGeometry *geometry, GeometryOwnershipType ownership)
{
  BLI_assert(this->is_mutable());
  this->clear();
  geometry_ = geometry;
  ownership_ = ownership;
}

/** \} */
