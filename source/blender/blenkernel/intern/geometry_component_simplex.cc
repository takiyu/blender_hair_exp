/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_simplex_types.h"

#include "BKE_geometry_set.hh"
#include "BKE_mesh_simplex.hh"

#include "attribute_access_intern.hh"

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

bool SimplexComponent::has_geometry() const
{
  return geometry_ != nullptr;
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

namespace blender::bke {

template <int Corner>
class SimplexVertexAttributeProvider final : public BuiltinAttributeProvider {
 private:
  static int get_corner_vertex(const int4 &tet)
  {
    return tet[Corner];
  }

 public:
  SimplexVertexAttributeProvider()
      : BuiltinAttributeProvider("vertex" + std::to_string(Corner),
                                 ATTR_DOMAIN_SIMPLEX,
                                 CD_PROP_INT32,
                                 NonCreatable,
                                 Readonly,
                                 NonDeletable)
  {
  }

  GVArray try_get_for_read(const void *owner) const final
  {
    const SimplexGeometry *simplices = static_cast<const SimplexGeometry *>(owner);
    if (simplices == nullptr) {
      return {};
    }
    Span<int4> simplex_span = simplices->simplex_vertices();
    return VArray<int>::ForDerivedSpan<int4, get_corner_vertex>(simplex_span);
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    return {};
  }

  bool try_delete(void * /*owner*/) const final
  {
    return false;
  }

  bool try_create(void * /*owner*/, const AttributeInit & /*initializer*/) const final
  {
    return false;
  }

  bool exists(const void *owner) const final
  {
    const SimplexGeometry *geometry = static_cast<const SimplexGeometry *>(owner);
    return geometry->simplex_num() != 0;
  }
};

static ComponentAttributeProviders create_attribute_providers_for_simplices()
{
  static SimplexVertexAttributeProvider<0> vertex0;
  static SimplexVertexAttributeProvider<1> vertex1;
  static SimplexVertexAttributeProvider<2> vertex2;
  static SimplexVertexAttributeProvider<3> vertex3;

  return ComponentAttributeProviders({&vertex0, &vertex1, &vertex2, &vertex3}, {});
}

static AttributeAccessorFunctions get_simplices_accessor_functions()
{
  static const ComponentAttributeProviders providers = create_attribute_providers_for_simplices();
  AttributeAccessorFunctions fn =
      attribute_accessor_functions::accessor_functions_for_providers<providers>();
  fn.domain_size = [](const void *owner, const eAttrDomain domain) {
    if (owner == nullptr) {
      return 0;
    }
    const SimplexGeometry *simplices = static_cast<const SimplexGeometry *>(owner);
    switch (domain) {
      case ATTR_DOMAIN_POINT:
        return simplices->point_num();
      case ATTR_DOMAIN_SIMPLEX:
        return simplices->simplex_num();
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const eAttrDomain domain) {
    return ELEM(domain, ATTR_DOMAIN_POINT, ATTR_DOMAIN_SIMPLEX);
  };
  fn.adapt_domain = [](const void *owner,
                       const blender::GVArray &varray,
                       const eAttrDomain from_domain,
                       const eAttrDomain to_domain) -> GVArray {
    if (owner == nullptr) {
      return {};
    }
    const SimplexGeometry &geometry = *static_cast<const SimplexGeometry *>(owner);
    return geometry.adapt_domain(varray, from_domain, to_domain);
  };
  return fn;
}

static const AttributeAccessorFunctions &get_simplices_accessor_functions_ref()
{
  static const AttributeAccessorFunctions fn = get_simplices_accessor_functions();
  return fn;
}

blender::bke::AttributeAccessor SimplexGeometry::attributes() const
{
  return blender::bke::AttributeAccessor(this,
                                         blender::bke::get_simplices_accessor_functions_ref());
}

blender::bke::MutableAttributeAccessor SimplexGeometry::attributes_for_write()
{
  return blender::bke::MutableAttributeAccessor(
      this, blender::bke::get_simplices_accessor_functions_ref());
}

}  // namespace blender::bke

std::optional<blender::bke::AttributeAccessor> SimplexComponent::attributes() const
{
  return blender::bke::AttributeAccessor(geometry_,
                                         blender::bke::get_simplices_accessor_functions_ref());
}

std::optional<blender::bke::MutableAttributeAccessor> SimplexComponent::attributes_for_write()
{
  return blender::bke::MutableAttributeAccessor(
      geometry_, blender::bke::get_simplices_accessor_functions_ref());
}

/** \} */
