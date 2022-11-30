/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_index_mask.hh"
#include "BLI_math_vec_types.hh"

#include "DNA_simplex_types.h"

#include "BKE_attribute.hh"
#include "BKE_mesh_simplex.h"

/** \file
 * \ingroup bke
 * \brief Low-level operations for curves.
 */

namespace blender::bke {

/**
 * A C++ class that wraps the DNA struct for better encapsulation and ease of use. It inherits
 * directly from the struct rather than storing a pointer to avoid more complicated ownership
 * handling.
 */
class SimplexGeometry : public ::SimplexGeometry {
 public:
  SimplexGeometry();
  SimplexGeometry(int point_num, int simplex_num);
  SimplexGeometry(const SimplexGeometry &other);
  SimplexGeometry(SimplexGeometry &&other);
  SimplexGeometry &operator=(const SimplexGeometry &other);
  SimplexGeometry &operator=(SimplexGeometry &&other);
  ~SimplexGeometry();

  static SimplexGeometry &wrap(::SimplexGeometry &dna_struct)
  {
    SimplexGeometry *geometry = reinterpret_cast<SimplexGeometry *>(&dna_struct);
    return *geometry;
  }
  static const SimplexGeometry &wrap(const ::SimplexGeometry &dna_struct)
  {
    const SimplexGeometry *geometry = reinterpret_cast<const SimplexGeometry *>(&dna_struct);
    return *geometry;
  }

  /* --------------------------------------------------------------------
   * Accessors.
   */

  /**
   * The number of points.
   */
  int point_num() const;
  /**
   * The number of tetrahedrons.
   */
  int tetrahedron_num() const;
  IndexRange points_range() const;
  IndexRange tetrahedrons_range() const;

  /**
   * Point positions.
   */
  Span<float3> positions() const;
  MutableSpan<float3> positions_for_write();

  /**
   * Simplex vertex indices.
   */
  Span<int4> tetrahedrons() const;
  MutableSpan<int4> tetrahedrons_for_write();

  blender::bke::AttributeAccessor attributes() const;
  blender::bke::MutableAttributeAccessor attributes_for_write();

  /* --------------------------------------------------------------------
   * Operations.
   */

  /** Change the number of elements. New values should be properly initialized afterwards. */
  void resize(int point_num, int tetrahedron_num);

  /** Call after any operation that changes the positions */
  void tag_positions_changed();
  /** Call after any operation that changes the topology */
  void tag_topology_changed();

  /* --------------------------------------------------------------------
   * Attributes.
   */

  GVArray adapt_domain(const GVArray &varray, eAttrDomain from, eAttrDomain to) const;
  template<typename T>
  VArray<T> adapt_domain(const VArray<T> &varray, eAttrDomain from, eAttrDomain to) const
  {
    return this->adapt_domain(GVArray(varray), from, to).typed<T>();
  }

 private:
  int domain_num(const eAttrDomain domain) const;
  CustomData &domain_custom_data(eAttrDomain domain);
  const CustomData &domain_custom_data(const eAttrDomain domain) const;

  template<typename T>
  VArray<T> get_varray_attribute(const eAttrDomain domain,
                                 const StringRefNull name,
                                 const T default_value);

  template<typename T>
  Span<T> get_span_attribute(const eAttrDomain domain, const StringRefNull name) const;

  template<typename T>
  MutableSpan<T> get_mutable_attribute(const eAttrDomain domain,
                                       const StringRefNull name,
                                       const T default_value = T());
};

/* -------------------------------------------------------------------- */
/** \name Inline Simplex Methods
 * \{ */

namespace topology {

/* Returns index triplets for each of the 4 tet sides.
 * For a tet (A, B, C, D) the sides are defined as triangles ABC, BAD, CBD, DAC.
 */
inline void tetrahedron_triangles(
    const int4 &verts, int3 &r_tri0, int3 &r_tri1, int3 &r_tri2, int3 &r_tri3)
{
  r_tri0 = int3(verts[0], verts[1], verts[2]);
  r_tri1 = int3(verts[1], verts[0], verts[3]);
  r_tri2 = int3(verts[2], verts[1], verts[3]);
  r_tri3 = int3(verts[3], verts[0], verts[2]);
}

/* Evaluate geometry of a simplex from vertex positions.
 * The n-th point is part of the n-th side.
 * Normal vectors are not normalized!
 */
void tetrahedron_geometry(Span<float3> positions,
                      const int4 &tet,
                      float3 &v0,
                      float3 &v1,
                      float3 &v2,
                      float3 &v3,
                      float3 &n0,
                      float3 &n1,
                      float3 &n2,
                      float3 &n3);

bool tetrahedron_contains_point(Span<float3> positions, const int4 &tet, const float3 &point);

bool tetrahedron_circumcenter(
    const float3 &p0, const float3 &p1, const float3 &p2, const float3 &p3, float3 &r_center);

}  // namespace topology

/** \} */

/* -------------------------------------------------------------------- */
/** \name #SimplexGeometry Inline Methods
 * \{ */

inline int SimplexGeometry::point_num() const
{
  return ((::SimplexGeometry *)this)->point_num;
}
inline int SimplexGeometry::tetrahedron_num() const
{
  return ((::SimplexGeometry *)this)->tetrahedron_num;
}
inline IndexRange SimplexGeometry::points_range() const
{
  return IndexRange(this->point_num());
}
inline IndexRange SimplexGeometry::tetrahedrons_range() const
{
  return IndexRange(this->tetrahedron_num());
}

/** \} */

}  // namespace blender::bke
