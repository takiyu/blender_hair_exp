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
   * The number of simplices.
   */
  int simplex_num() const;
  IndexRange points_range() const;
  IndexRange simplex_range() const;

  /**
   * Point positions.
   */
  Span<float3> positions() const;
  MutableSpan<float3> positions_for_write();

  /**
   * Simplex vertex indices.
   */
  Span<int4> simplex_vertices() const;
  MutableSpan<int4> simplex_vertices_for_write();

  blender::bke::AttributeAccessor attributes() const;
  blender::bke::MutableAttributeAccessor attributes_for_write();

  /**
   * Remove simplices contained in the index mask.
   */
  void remove(const blender::IndexMask mask);

  /* --------------------------------------------------------------------
   * Operations.
   */

 public:
  /** Change the number of elements. New values should be properly initialized afterwards. */
  void resize(int point_num, int simplex_num);

  /** Call after any operation that changes the topology */
  void tag_topology_changed();
};

/* -------------------------------------------------------------------- */
/** \name Inline Simplex Methods
 * \{ */

namespace topology {

/* Returns index triplets for each of the 4 tet sides.
 * For a tet (A, B, C, D) the sides are defined as triangles ABC, BAD, CBD, DAC.
 */
inline void simplex_triangles(const int4 &verts, int3 &r_tri0, int3 &r_tri1, int3 &r_tri2, int3 &r_tri3)
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
void simplex_geometry(Span<float3> positions,
                      const int4 &verts,
                      float3 &v0,
                      float3 &v1,
                      float3 &v2,
                      float3 &v3,
                      float3 &n0,
                      float3 &n1,
                      float3 &n2,
                      float3 &n3);

bool simplex_contains_point(Span<float3> positions, const int4 &verts, const float3 &point);

bool simplex_circumcenter(
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
inline int SimplexGeometry::simplex_num() const
{
  return ((::SimplexGeometry *)this)->simplex_num;
}
inline IndexRange SimplexGeometry::points_range() const
{
  return IndexRange(this->point_num());
}
inline IndexRange SimplexGeometry::simplex_range() const
{
  return IndexRange(this->simplex_num());
}

Span<int4> SimplexGeometry::simplex_vertices() const
{
  return get_varray_attribute<int8_t>(
      *this, ATTR_DOMAIN_CURVE, ATTR_CURVE_TYPE, CURVE_TYPE_CATMULL_ROM);
}

MutableSpan<int4> SimplexGeometry::simplex_vertices_for_write()
{
  return get_mutable_attribute<int8_t>(*this, ATTR_DOMAIN_CURVE, ATTR_CURVE_TYPE);
}

inline Span<Simplex> SimplexGeometry::simplices() const
{
  return Span<Simplex>(((::SimplexGeometry *)this)->simplices, this->simplex_num());
}
inline MutableSpan<Simplex> SimplexGeometry::simplices_for_write()
{
  return MutableSpan<Simplex>(((::SimplexGeometry *)this)->simplices, this->simplex_num());
}

/** \} */

}  // namespace blender::bke
