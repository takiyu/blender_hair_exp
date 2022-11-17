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
  static int get_corner_vertex(const Simplex &simplex)
  {
    return simplex.v[Corner];
  }

  static void set_corner_vertex(Simplex &simplex, int vertex)
  {
    simplex.v[Corner] = vertex;
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
    Span<Simplex> simplex_span = simplices->simplices();
    return VArray<int>::ForDerivedSpan<Simplex, get_corner_vertex>(simplex_span);
  }

  GAttributeWriter try_get_for_write(void *owner) const final
  {
    SimplexGeometry *simplices = static_cast<SimplexGeometry *>(owner);
    if (simplices == nullptr) {
      return {};
    }
    MutableSpan<Simplex> simplex_span = simplices->simplices_for_write();
    return {VMutableArray<int>::ForDerivedSpan<Simplex, get_corner_vertex, set_corner_vertex>(
                simplex_span),
            domain_};
  }

  bool try_delete(void * /*owner*/) const final
  {
    return false;
  }

  bool try_create(void * /*owner*/, const AttributeInit & /*initializer*/) const final
  {
    return false;
  }

  bool exists(const void * /*owner*/) const final
  {
    return true;
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
      case ATTR_DOMAIN_SIMPLEX:
        return simplices->simplex_num();
      default:
        return 0;
    }
  };
  fn.domain_supported = [](const void * /*owner*/, const eAttrDomain domain) {
    return domain == ATTR_DOMAIN_SIMPLEX;
  };
  fn.adapt_domain = [](const void * /*owner*/,
                       const blender::GVArray &varray,
                       const eAttrDomain from_domain,
                       const eAttrDomain to_domain) {
    if (from_domain == to_domain && from_domain == ATTR_DOMAIN_SIMPLEX) {
      return varray;
    }
    return blender::GVArray{};
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

void SimplexGeometry::remove(const IndexMask mask)
{
  using namespace blender;
  if (mask.is_range() && mask.as_range().start() == 0) {
    /* Deleting from the end of the array can be much faster since no data has to be shifted. */
    this->resize(mask.size());
    return;
  }

  //const bke::CustomDataAttributes &src_attributes = attributes_;

  //bke::CustomDataAttributes dst_attributes;
  //dst_attributes.reallocate(mask.size());

  //src_attributes.foreach_attribute(
      //[&](const bke::AttributeIDRef &id, const bke::AttributeMetaData &meta_data) {
      //  if (!id.should_be_kept()) {
      //    return true;
      //  }

      //  GSpan src = *src_attributes.get_for_read(id);
      //  dst_attributes.create(id, meta_data.data_type);
      //  GMutableSpan dst = *dst_attributes.get_for_write(id);
      //  array_utils::gather(src, mask.indices(), dst);

      //  return true;
      //},
      //ATTR_DOMAIN_INSTANCE);

  //attributes_ = std::move(dst_attributes);
  //this->remove_unused_references();
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
