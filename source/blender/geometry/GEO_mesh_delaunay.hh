/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_map.hh"
#include "BLI_math_vec_types.hh"

#pragma once

/** \file
 * \ingroup geo
 */

struct Mesh;
namespace blender::bke {
class SimplexGeometry;
}

namespace blender::geometry {

namespace simplex {

enum SimplexFaceMode { All, Shared };
Mesh *simplex_to_mesh(Span<float3> positions, Span<int4> tets, SimplexFaceMode face_mode);

}  // namespace simplex

namespace delaunay {

/* Constructs an axis-aligned tetrahedron enclosing the points.
 * This is not a minimal enclosing simplex, which is an NP-hard problem,
 * but should work well enough in most situations.
 */
void find_enclosing_simplex(Span<float3> positions, float3 simplex[4]);

/* List of adjacent tets for each of the 4 sides. */
Array<int4> tet_build_side_to_tet_map(Span<int4> tets);

/* Brute-force search for a tet containing a point. */
int tet_find(Span<float3> positions, Span<int4> tets, const float3 &point);

/* Find all simplices whose Delaunay condition is violated by adding a point.
 * Returns true for each simplex where the point is inside the circum-sphere.
 */
Array<bool> check_delaunay_condition(Span<float3> positions,
                                     Span<int4> tets,
                                     Span<int4> side_map,
                                     const float3 &new_point,
                                     int containing_simplex);

/* Find tet containing a point by walking in a straight line from a known tet.
 * Warning: this only works as long as all tets are within the same convex hull!
 * \param tets List of tetrahedra.
 * \param side_map Adjacent tet indices for each tet.
 * \return Index of the tet containing the destination point, or -1 if no containing tet could be
 * found.
 */
int tet_find_from_known(Span<float3> positions,
                        Span<int4> tets,
                        Span<int4> side_map,
                        const int src_tet,
                        const float3 &dst_point);

/* Replace marked tets with new tets between boundary faces and a point.
 * /param tets List of all tets, will be modified by removing and adding tets.
 * /param side_map Adjacency map for tet sides, gets updated along with the tets list.
 * /param replace Flag indicating which tets to replace.
 * /param point Index of the new point to add.
 */
void tet_fan_construct(Array<int4> &tets,
                       Array<int4> &side_map,
                       Span<bool> replace,
                       int point,
                       IndexRange &r_old_simplex_range,
                       IndexRange &r_new_simplex_range);

/* Build a Delaunay tetrahedralization of the point set. */
void tetrahedralize_points(Span<float3> positions,
                           Array<float3> &r_points,
                           Array<int4> &r_tets,
                           bool keep_hull = false);

}  // namespace delaunay

}  // namespace blender::geometry
