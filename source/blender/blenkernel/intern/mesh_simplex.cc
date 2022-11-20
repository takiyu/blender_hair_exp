/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BLI_math_vector.hh"
#include "BLI_task.hh"

#include "BKE_attribute_math.hh"
#include "BKE_mesh_simplex.hh"

namespace blender::bke {

static const std::string ATTR_POSITION = "position";
static const std::string ATTR_VERTEX = "vertex";

SimplexGeometry::SimplexGeometry() : SimplexGeometry(0, 0)
{
}

SimplexGeometry::SimplexGeometry(int point_num, int simplex_num)
{
  ::SimplexGeometry *c_this = (::SimplexGeometry *)this;

  c_this->point_num = point_num;
  c_this->simplex_num = simplex_num;
  CustomData_reset(&this->point_data);
  CustomData_reset(&this->simplex_data);

  CustomData_add_layer_named(&this->point_data,
                             CD_PROP_FLOAT3,
                             CD_CONSTRUCT,
                             nullptr,
                             c_this->point_num,
                             ATTR_POSITION.c_str());
  c_this->simplex_verts = (int(*)[4])MEM_malloc_arrayN(
      c_this->simplex_num, sizeof(int[4]), __func__);
}

static void copy_simplex_geometry(SimplexGeometry &dst, const SimplexGeometry &src)
{
  ::SimplexGeometry &c_dst = (::SimplexGeometry &)dst;
  const ::SimplexGeometry &c_src = (const ::SimplexGeometry &)src;

  CustomData_free(&c_dst.point_data, c_dst.point_num);
  CustomData_free(&c_dst.simplex_data, c_dst.simplex_num);
  c_dst.point_num = c_src.point_num;
  c_dst.simplex_num = c_src.simplex_num;
  CustomData_copy(&c_src.point_data, &c_dst.point_data, CD_MASK_ALL, CD_DUPLICATE, c_dst.point_num);
  CustomData_copy(&c_src.simplex_data, &c_dst.simplex_data, CD_MASK_ALL, CD_DUPLICATE, c_dst.simplex_num);

  c_dst.simplex_verts = (int(*)[4])MEM_malloc_arrayN(c_dst.simplex_num, sizeof(int[4]), __func__);
  dst.simplex_vertices_for_write().copy_from(src.simplex_vertices());

  dst.tag_topology_changed();
}

/* The source should be empty, but in a valid state so that using it further will work. */
static void move_simplex_geometry(SimplexGeometry &dst, SimplexGeometry &src)
{
  ::SimplexGeometry &c_dst = (::SimplexGeometry &)dst;
  ::SimplexGeometry &c_src = (::SimplexGeometry &)src;

  c_dst.point_num = c_src.point_num;
  std::swap(c_dst.point_data, c_src.point_data);
  CustomData_free(&c_src.point_data, c_src.point_num);
  c_src.point_num = 0;

  c_dst.simplex_num = c_src.simplex_num;
  std::swap(c_dst.simplex_data, c_src.simplex_data);
  CustomData_free(&c_src.simplex_data, c_src.simplex_num);
  c_src.simplex_num = 0;

  std::swap(c_dst.simplex_verts, c_src.simplex_verts);
  MEM_SAFE_FREE(c_src.simplex_verts);
}

SimplexGeometry::SimplexGeometry(const SimplexGeometry &other)
{
  copy_simplex_geometry(*this, other);
}

SimplexGeometry::SimplexGeometry(SimplexGeometry &&other)
{
  move_simplex_geometry(*this, other);
}

SimplexGeometry &SimplexGeometry::operator=(const SimplexGeometry &other)
{
  if (this != &other) {
    copy_simplex_geometry(*this, other);
  }
  return *this;
}

SimplexGeometry &SimplexGeometry::operator=(SimplexGeometry &&other)
{
  if (this != &other) {
    move_simplex_geometry(*this, other);
  }
  return *this;
}

SimplexGeometry::~SimplexGeometry()
{
  ::SimplexGeometry *c_this = (::SimplexGeometry *)this;

  CustomData_free(&c_this->point_data, c_this->point_num);
  CustomData_free(&c_this->simplex_data, c_this->simplex_num);
  MEM_SAFE_FREE((c_this)->simplex_verts);
}

int SimplexGeometry::domain_num(const eAttrDomain domain) const
{
  switch (domain) {
    case ATTR_DOMAIN_POINT:
      return point_num();
    case ATTR_DOMAIN_SIMPLEX:
      return simplex_num();
    default:
      return 0;
  }
}

CustomData &SimplexGeometry::domain_custom_data(const eAttrDomain domain)
{
  switch (domain) {
    case ATTR_DOMAIN_POINT:
      return point_data;
    case ATTR_DOMAIN_SIMPLEX:
      return simplex_data;
    default:
      BLI_assert_unreachable();
      static CustomData dummy;
      return dummy;
  }
}

const CustomData &SimplexGeometry::domain_custom_data(const eAttrDomain domain) const
{
  switch (domain) {
    case ATTR_DOMAIN_POINT:
      return point_data;
    case ATTR_DOMAIN_SIMPLEX:
      return simplex_data;
    default:
      BLI_assert_unreachable();
      static CustomData dummy;
      return dummy;
  }
}

template<typename T>
VArray<T> SimplexGeometry::get_varray_attribute(const eAttrDomain domain,
                                                const StringRefNull name,
                                                const T default_value)
{
  const int num = domain_num(curves, domain);
  const eCustomDataType type = cpp_type_to_custom_data_type(CPPType::get<T>());
  const CustomData &custom_data = domain_custom_data(curves, domain);

  const T *data = (const T *)CustomData_get_layer_named(&custom_data, type, name.c_str());
  if (data != nullptr) {
    return VArray<T>::ForSpan(Span<T>(data, num));
  }
  return VArray<T>::ForSingle(default_value, num);
}

template<typename T>
Span<T> SimplexGeometry::get_span_attribute(const eAttrDomain domain,
                                            const StringRefNull name) const
{
  const int num = domain_num(domain);
  const CustomData &custom_data = domain_custom_data(domain);
  const eCustomDataType type = cpp_type_to_custom_data_type(CPPType::get<T>());

  T *data = (T *)CustomData_get_layer_named(&custom_data, type, name.c_str());
  if (data == nullptr) {
    return {};
  }
  return {data, num};
}

template<typename T>
MutableSpan<T> SimplexGeometry::get_mutable_attribute(const eAttrDomain domain,
                                                      const StringRefNull name,
                                                      const T default_value)
{
  const int num = domain_num(domain);
  const eCustomDataType type = cpp_type_to_custom_data_type(CPPType::get<T>());
  CustomData &custom_data = domain_custom_data(domain);

  T *data = (T *)CustomData_duplicate_referenced_layer_named(
      &custom_data, type, name.c_str(), num);
  if (data != nullptr) {
    return {data, num};
  }
  data = (T *)CustomData_add_layer_named(
      &custom_data, type, CD_SET_DEFAULT, nullptr, num, name.c_str());
  MutableSpan<T> span = {data, num};
  if (num > 0 && span.first() != default_value) {
    span.fill(default_value);
  }
  return span;
}

Span<float3> SimplexGeometry::positions() const
{
  return get_span_attribute<float3>(ATTR_DOMAIN_POINT, ATTR_POSITION);
}
MutableSpan<float3> SimplexGeometry::positions_for_write()
{
  return get_mutable_attribute<float3>(ATTR_DOMAIN_POINT, ATTR_POSITION);
}

Span<int4> SimplexGeometry::simplex_vertices() const
{
  const ::SimplexGeometry *c_this = (const ::SimplexGeometry *)this;
  return Span(reinterpret_cast<const int4 *>(c_this->simplex_verts), c_this->simplex_num);
}
MutableSpan<int4> SimplexGeometry::simplex_vertices_for_write()
{
  ::SimplexGeometry *c_this = (::SimplexGeometry *)this;
  return MutableSpan(reinterpret_cast<int4 *>(c_this->simplex_verts), c_this->simplex_num);
}

void SimplexGeometry::resize(int point_num, int simplex_num)
{
  ::SimplexGeometry *c_this = (::SimplexGeometry *)this;

  if (point_num != c_this->point_num) {
    CustomData_realloc(&c_this->point_data, c_this->point_num, point_num);
    c_this->point_num = point_num;
  }
  if (simplex_num != c_this->simplex_num) {
    CustomData_realloc(&c_this->simplex_data, c_this->simplex_num, simplex_num);
    MEM_reallocN(c_this->simplex_verts, sizeof(int[4]) * simplex_num);
    c_this->simplex_num = simplex_num;
  }
  this->tag_topology_changed();
}

void SimplexGeometry::tag_topology_changed()
{
}

/* -------------------------------------------------------------------- */
/** \name Domain Interpolation
 * \{ */

/**
 * Mix together all corners of a simplex.
 *
 * \note Theoretically this interpolation does not need to compute all values at once.
 * However, doing that makes the implementation simpler, and this can be optimized in the future if
 * only some values are required.
 */
template<typename T>
static void adapt_simplex_domain_point_to_simplex_impl(const SimplexGeometry &geometry,
                                                       const VArray<T> &old_values,
                                                       MutableSpan<T> r_values)
{
  const Span<int4> tets = geometry.simplex_vertices();
  attribute_math::DefaultMixer<T> mixer(r_values);

  threading::parallel_for(geometry.simplex_range(), 128, [&](const IndexRange range) {
    for (const int i_simplex : range) {
      const int4 &tet = tets[i_simplex];
      for (const int k : IndexRange(4)) {
        mixer.mix_in(i_simplex, old_values[tet[k]]);
      }
    }
    mixer.finalize(range);
  });
}

/**
 * A simplex is selected if all of its vertices were selected.
 *
 * \note Theoretically this interpolation does not need to compute all values at once.
 * However, doing that makes the implementation simpler, and this can be optimized in the future if
 * only some values are required.
 */
template<>
void adapt_simplex_domain_point_to_simplex_impl(const SimplexGeometry &geometry,
                                                const VArray<bool> &old_values,
                                                MutableSpan<bool> r_values)
{
  const Span<int4> tets = geometry.simplex_vertices();

  r_values.fill(true);
  for (const int i_simplex : geometry.simplex_range()) {
    const int4 &tet = tets[i_simplex];
    for (const int k : IndexRange(4)) {
      if (!old_values[tet[k]]) {
        r_values[i_simplex] = false;
        break;
      }
    }
  }
}

static GVArray adapt_simplex_domain_point_to_simplex(const SimplexGeometry &geometry,
                                                     const GVArray &varray)
{
  GVArray new_varray;
  attribute_math::convert_to_static_type(varray.type(), [&](auto dummy) {
    using T = decltype(dummy);
    if constexpr (!std::is_void_v<attribute_math::DefaultMixer<T>>) {
      Array<T> values(geometry.simplex_num());
      adapt_simplex_domain_point_to_simplex_impl<T>(geometry, varray.typed<T>(), values);
      new_varray = VArray<T>::ForContainer(std::move(values));
    }
  });
  return new_varray;
}

/**
 * Mix together all simplices of a vertex.
 *
 * \note Theoretically this interpolation does not need to compute all values at once.
 * However, doing that makes the implementation simpler, and this can be optimized in the future if
 * only some values are required.
 */
template<typename T>
static void adapt_simplex_domain_simplex_to_point_impl(const SimplexGeometry &geometry,
                                                       const VArray<T> &old_values,
                                                       MutableSpan<T> r_values)
{
  const Span<int4> tets = geometry.simplex_vertices();
  attribute_math::DefaultMixer<T> mixer(r_values);

  threading::parallel_for(geometry.simplex_range(), 128, [&](const IndexRange range) {
    for (const int i_simplex : range) {
      const int4 &tet = tets[i_simplex];
      for (const int k : IndexRange(4)) {
        mixer.mix_in(tet[k], old_values[i_simplex]);
      }
    }
    mixer.finalize(range);
  });
}

/**
 * A point is selected if any of its simplices were selected.
 *
 * \note Theoretically this interpolation does not need to compute all values at once.
 * However, doing that makes the implementation simpler, and this can be optimized in the future if
 * only some values are required.
 */
template<>
void adapt_simplex_domain_simplex_to_point_impl(const SimplexGeometry &geometry,
                                                const VArray<bool> &old_values,
                                                MutableSpan<bool> r_values)
{
  const Span<int4> tets = geometry.simplex_vertices();

  r_values.fill(false);
  for (const int i_simplex : geometry.simplex_range()) {
    if (old_values[i_simplex]) {
      const int4 &tet = tets[i_simplex];
      for (const int k : IndexRange(4)) {
        r_values[tet[k]] = true;
      }
    }
  }
}

static GVArray adapt_simplex_domain_simplex_to_point(const SimplexGeometry &geometry,
                                                     const GVArray &varray)
{
  GVArray new_varray;
  attribute_math::convert_to_static_type(varray.type(), [&](auto dummy) {
    using T = decltype(dummy);
    Array<T> values(geometry.point_num());
    adapt_simplex_domain_simplex_to_point_impl<T>(geometry, varray.typed<T>(), values);
    new_varray = VArray<T>::ForContainer(std::move(values));
  });
  return new_varray;
}

GVArray SimplexGeometry::adapt_domain(const GVArray &varray,
                                      const eAttrDomain from,
                                      const eAttrDomain to) const
{
  if (!varray) {
    return {};
  }
  if (varray.is_empty()) {
    return {};
  }
  if (from == to) {
    return varray;
  }
  if (varray.is_single()) {
    BUFFER_FOR_CPP_TYPE_VALUE(varray.type(), value);
    varray.get_internal_single(value);
    return GVArray::ForSingle(varray.type(), this->attributes().domain_size(to), value);
  }

  if (from == ATTR_DOMAIN_POINT && to == ATTR_DOMAIN_SIMPLEX) {
    return adapt_simplex_domain_point_to_simplex(*this, varray);
  }
  if (from == ATTR_DOMAIN_SIMPLEX && to == ATTR_DOMAIN_POINT) {
    return adapt_simplex_domain_simplex_to_point(*this, varray);
  }

  BLI_assert_unreachable();
  return {};
}

/** \} */

namespace topology {

/* Evaluate geometry of a tet from vertex positions.
 * The n-th point is part of the n-th side.
 */
void simplex_geometry(Span<float3> positions,
                      const int4 &verts,
                      float3 &v0,
                      float3 &v1,
                      float3 &v2,
                      float3 &v3,
                      float3 &n0,
                      float3 &n1,
                      float3 &n2,
                      float3 &n3)
{
  v0 = positions[verts[0]];
  v1 = positions[verts[1]];
  v2 = positions[verts[2]];
  v3 = positions[verts[3]];
  const float3 e01 = v1 - v0;
  const float3 e02 = v2 - v0;
  const float3 e03 = v3 - v0;

  n0 = math::cross(e01, e02);
  n1 = math::cross(e03, e01);
  n3 = math::cross(e02, e03);
  /* Can avoid cross product for the last normal */
  n2 = -(n0 + n1 + n3);
}

bool simplex_contains_point(const Span<float3> positions, const int4 &verts, const float3 &point)
{
  float3 v[4], n[4];
  simplex_geometry(positions, verts, v[0], v[1], v[2], v[3], n[0], n[1], n[2], n[3]);

  const float dot0 = math::dot(point - v[0], n[0]);
  const float dot1 = math::dot(point - v[1], n[1]);
  const float dot3 = math::dot(point - v[3], n[3]);
  /* Exploit normal redundancy to save one dot product: n2 == -(n0 + n1 + n3) */
  const float dot2 = -(dot0 + dot1 + dot3);
  return (dot0 <= 0.0f) && (dot1 <= 0.0f) && (dot2 <= 0.0f) && (dot3 <= 0.0f);
}

//static float3 triangle_circum_center(const float3 &a, const float3 &b, const float3 &c)
//{
//  const float3 ab = b - a;
//  const float3 ac = c - a;
//  const float3 abc = math::cross(ab, ac);
//
//  const float n = math::length_squared(abc);
//  if (n > 0.0f) {
//    const float3 d = math::cross(abc, ab) * math::length_squared(ac);
//    const float3 t = math::cross(ac, abc) * math::length_squared(ab);
//
//    return a + 0.5f * (d + t) / n;
//  }
//  return a;
//}

bool simplex_circumcenter(
    const float3 &p0, const float3 &p1, const float3 &p2, const float3 &p3, float3 &r_center)
{
  float eps = 1e-6f;

  const float3 b = p1 - p0;
  const float3 c = p2 - p0;
  const float3 d = p3 - p0;
  const float3 n0 = math::cross(b, c);
  const float3 n1 = math::cross(c, d);
  const float3 n2 = math::cross(d, b);
  const float3 n3 = -(n0 + n1 + n2);

  const float det = 2.0 * math::dot(b, n1);
  if (fabs(det) > eps) {
    r_center = p0 + (n1 * math::dot(b, b) + n2 * math::dot(c, c) + n0 * math::dot(d, d)) / det;
    return true;
  }
  else {
    r_center = p0;
    return false;
  }
}

}  // namespace topology

}  // namespace blender::bke
