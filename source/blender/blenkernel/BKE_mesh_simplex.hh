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
 * Contains derived data, caches, and other information not saved in files, besides a few pointers
 * to arrays that are kept in the non-runtime struct to avoid dereferencing this whenever they are
 * accessed.
 */
class SimplexGeometryRuntime {
 public:
};

/**
 * A C++ class that wraps the DNA struct for better encapsulation and ease of use. It inherits
 * directly from the struct rather than storing a pointer to avoid more complicated ownership
 * handling.
 */
class SimplexGeometry : public ::SimplexGeometry {
 public:
  SimplexGeometry();
  SimplexGeometry(int simplex_num);
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
   * The total number of simplices.
   */
  int simplex_num() const;
  IndexRange simplex_range() const;

  /**
   * List of simplices in this geometry.
   */
  Span<Simplex> simplices() const;
  MutableSpan<Simplex> simplices_for_write();

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
  void resize(int simplex_num);

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
inline void simplex_triangles(const Simplex &tet, int3 &r_tri0, int3 &r_tri1, int3 &r_tri2, int3 &r_tri3)
{
  const unsigned int *verts = tet.v;
  r_tri0 = int3(verts[0], verts[1], verts[2]);
  r_tri1 = int3(verts[1], verts[0], verts[3]);
  r_tri2 = int3(verts[2], verts[1], verts[3]);
  r_tri3 = int3(verts[3], verts[0], verts[2]);
}

/* Evaluate geometry of a tet from vertex positions.
 * The n-th point is part of the n-th side.
 * Normal vectors are not normalized!
 */
void simplex_geometry(Span<float3> positions,
                      const Simplex &tet,
                      float3 &v0,
                      float3 &v1,
                      float3 &v2,
                      float3 &v3,
                      float3 &n0,
                      float3 &n1,
                      float3 &n2,
                      float3 &n3);

bool simplex_contains_point(Span<float3> positions, const Simplex &tet, const float3 &point);

bool simplex_circumcenter(
    const float3 &p0, const float3 &p1, const float3 &p2, const float3 &p3, float3 &r_center);

}  // namespace topology

/** \} */

/* -------------------------------------------------------------------- */
/** \name #SimplexGeometry Inline Methods
 * \{ */

inline int SimplexGeometry::simplex_num() const
{
  return ((::SimplexGeometry *)this)->simplex_num;
}
inline IndexRange SimplexGeometry::simplex_range() const
{
  return IndexRange(this->simplex_num());
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
