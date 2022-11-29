/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_map.hh"
#include "BLI_float3x3.hh"
#include "BLI_math_geom.h"
#include "BLI_math_vector.hh"
#include "BLI_set.hh"
#include "BLI_stack.hh"

#include "DNA_meshdata_types.h"
#include "DNA_node_types.h"

#include "BKE_mesh.h"
#include "BKE_mesh_simplex.hh"

#include "GEO_mesh_delaunay.hh"

using namespace blender::bke;

namespace blender::geometry {

/* Make a unique edge identifier by sorting indices */
static int2 get_unique_edge(int2 edge, bool &r_flipped)
{
  if (edge.x < edge.y) {
    r_flipped = false;
    return int2(edge.x, edge.y);
  }
  else {
    r_flipped = true;
    return int2(edge.y, edge.x);
  }
}

/* Make a unique triangle identifier by sorting indices */
static int3 get_unique_tri(int3 tri, bool &r_flipped)
{
  if (tri.x < tri.y) {
    if (tri.y < tri.z) {
      r_flipped = false;
      return int3(tri.x, tri.y, tri.z);
    }
    else {
      if (tri.z < tri.x) {
        r_flipped = false;
        return int3(tri.z, tri.x, tri.y);
      }
      else {
        r_flipped = true;
        return int3(tri.x, tri.z, tri.y);
      }
    }
  }
  else {
    if (tri.y < tri.z) {
      if (tri.z < tri.x) {
        r_flipped = false;
        return int3(tri.y, tri.z, tri.x);
      }
      else {
        r_flipped = true;
        return int3(tri.y, tri.x, tri.z);
      }
    }
    else {
      r_flipped = true;
      return int3(tri.z, tri.y, tri.x);
    }
  }
}

namespace simplex {

Mesh *simplex_to_mesh_separate(const Span<float3> positions, const Span<int4> tets)
{
  /* Unique triangle set */
  Mesh *mesh = BKE_mesh_new_nomain(tets.size() * 4, 0, 0, tets.size() * 12, tets.size() * 4);
  MutableSpan<MVert> verts = mesh->verts_for_write();
  MutableSpan<MPoly> polys = mesh->polys_for_write();
  MutableSpan<MLoop> loops = mesh->loops_for_write();

  int loopstart = 0;
  int polystart = 0;
  for (const int tet_i : tets.index_range()) {
    const int4 &tet = tets[tet_i];

    IndexRange vert_range(tet_i * 4, 4);
    for (const int k : IndexRange(4)) {
      copy_v3_v3(verts[vert_range[k]].co, positions[tet[k]]);
    }

    /* Local indices, same for every tet */
    int3 tri[4];
    topology::simplex_triangles(int4{0, 1, 2, 3}, tri[0], tri[1], tri[2], tri[3]);

    for (const int k : IndexRange(4)) {
      const int poly_i = polystart + k;
      polys[poly_i].loopstart = loopstart;
      polys[poly_i].totloop = 3;

      for (const int i : IndexRange(3)) {
        const int loop_i = loopstart + i;
        loops[loop_i].v = vert_range[tri[k][i]];
      }
      loopstart += 3;
    }
    polystart += 4;
  }
  BKE_mesh_calc_edges(mesh, false, false);

  //BKE_mesh_validate(mesh, true, true);

  return mesh;
}

Mesh *simplex_to_mesh_shared(const Span<float3> positions, const Span<int4> tets, bool both_sides)
{
  /* Unique triangle set */
  Set<int3> tri_set;
  for (const int tet_i : tets.index_range()) {
    int3 tri[4];
    topology::simplex_triangles(tets[tet_i], tri[0], tri[1], tri[2], tri[3]);
    for (const int k : IndexRange(4)) {
      bool flipped;
      const int3 key = get_unique_tri(tri[k], flipped);
      tri_set.add(key);
    }
  }

  const int num_tris = both_sides ? 2 * tri_set.size() : tri_set.size();
  Mesh *mesh = BKE_mesh_new_nomain(positions.size(), 0, 0, num_tris * 3, num_tris);
  MutableSpan<MVert> verts = mesh->verts_for_write();
  MutableSpan<MPoly> polys = mesh->polys_for_write();
  MutableSpan<MLoop> loops = mesh->loops_for_write();

  for (const int vert_i : verts.index_range()) {
    copy_v3_v3(verts[vert_i].co, positions[vert_i]);
  }

  int poly_i = 0;
  int loop_i = 0;
  for (const int3 &tri : tri_set) {
    polys[poly_i].loopstart = loop_i;
    polys[poly_i].totloop = 3;
    for (const int i : IndexRange(3)) {
      loops[loop_i + i].v = tri[i];
    }
    poly_i += 1;
    loop_i += 3;

    if (both_sides) {
      polys[poly_i].loopstart = loop_i;
      polys[poly_i].totloop = 3;
      for (const int i : IndexRange(3)) {
        loops[loop_i + i].v = tri[2 - i];
      }
      poly_i += 1;
      loop_i += 3;
    }
  }
  BKE_mesh_calc_edges(mesh, false, false);

  //BKE_mesh_validate(mesh, true, true);

  return mesh;
}

Mesh *simplex_to_mesh(const Span<float3> positions, const Span<int4> tets, SimplexToMeshMode mode)
{
  switch (mode) {
    case Separate:
      return simplex_to_mesh_separate(positions, tets);
    case SharedVerts:
      return simplex_to_mesh_shared(positions, tets, true);
    case SharedFaces:
      return simplex_to_mesh_shared(positions, tets, false);
  }
  return nullptr;
}

Mesh *simplex_to_dual_mesh(const Span<float3> positions,
                           const Span<int4> tets,
                           SimplexToMeshMode mode)
{
  const bool separate_verts = (mode == Separate);
  const bool double_faces = ELEM(mode, Separate, SharedVerts);
  const Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);

  /* Circumsphere centers that form the new vertices */
  Array<MVert> tet_centers(tets.size());
  /* Up to 4 vertex copies when separating vertices per corner */
  Vector<MVert> tet_verts;
  if (separate_verts) {
    tet_verts.reserve(4 * tets.size());
  }
  /* Map of vertex indices for each of the 4 tet corners, used if separating vertices */
  Array<int4> tet_vert_map(tets.size(), int4(-1));
  /* Boundary vertex marker: true if any of the triangles using a verts is a boundary */
  Array<bool> is_boundary_vert(positions.size(), false);
  /* Maps edges to an adjacent tet, which is the start of the vertex loop */
  Map<int2, int> edge_start_tet;

  for (const int tet_i: tets.index_range()) {
    const int4 &tet = tets[tet_i];
    const int4 neighbor = side_map[tet_i];

    float3 center;
    topology::simplex_circumcenter(
        positions[tet[0]], positions[tet[1]], positions[tet[2]], positions[tet[3]], center);
    copy_v3_v3(tet_centers[tet_i].co, center);

    int3 tri[4];
    topology::simplex_triangles(tet, tri[0], tri[1], tri[2], tri[3]);

    for (const int k : IndexRange(4)) {
      if (neighbor[k] < 0) {
        /* No polygon for this triangle's edges, mark boundary verts and continue */
        is_boundary_vert[tri[k][0]] = true;
        is_boundary_vert[tri[k][1]] = true;
        is_boundary_vert[tri[k][2]] = true;
        continue;
      }

      const int2 edge0 = int2(tri[k][0], tri[k][1]);
      const int2 edge1 = int2(tri[k][1], tri[k][2]);
      const int2 edge2 = int2(tri[k][2], tri[k][0]);
      bool flipped0, flipped1, flipped2;
      const int2 key0 = get_unique_edge(edge0, flipped0);
      const int2 key1 = get_unique_edge(edge1, flipped1);
      const int2 key2 = get_unique_edge(edge2, flipped2);
      if (!flipped0) {
        edge_start_tet.add(key0, tet_i);
      }
      if (!flipped1) {
        edge_start_tet.add(key1, tet_i);
      }
      if (!flipped2) {
        edge_start_tet.add(key2, tet_i);
      }
    }
  }

  /* List of loops and polys for both directions of faces.
   * The b lists are only used when double faces are generated.
   */
  Vector<MPoly> polys_a, polys_b;
  polys_a.reserve(edge_start_tet.size());
  polys_b.reserve(edge_start_tet.size());
  Vector<MLoop> loops_a;
  Vector<MLoop> loops_b;

  for (const Map<int2, int>::Item &edge_item : edge_start_tet.items()) {
    const int vert_a = edge_item.key[0];
    const int vert_b = edge_item.key[1];
    const bool is_boundary_a = is_boundary_vert[vert_a];
    const bool is_boundary_b = is_boundary_vert[vert_b];
    const bool create_poly_a = !is_boundary_a;
    const bool create_poly_b = !is_boundary_b && (double_faces || is_boundary_a);
    if (!create_poly_a && !create_poly_b) {
      /* Full boundary edge, ignore */
      continue;
    }

    MPoly poly_a, poly_b;
    poly_a.loopstart = loops_a.size();
    poly_b.loopstart = loops_b.size();

    const int tet_start = edge_item.value;
    int tet_i = tet_start;
    while (true) {
      const int4 &tet = tets[tet_i];
      int3 tri[4];
      topology::simplex_triangles(tet, tri[0], tri[1], tri[2], tri[3]);

      const int corner_a = (vert_a == tet[0] ?
                                0 :
                                (vert_a == tet[1] ? 1 : (vert_a == tet[2] ? 2 : 3)));
      const int corner_b = (vert_b == tet[0] ?
                                0 :
                                (vert_b == tet[1] ? 1 : (vert_b == tet[2] ? 2 : 3)));
      BLI_assert(corner_a != corner_b);

      if (separate_verts) {
        if (create_poly_a) {
          int &tet_vert_a = tet_vert_map[tet_i][corner_a];
          if (tet_vert_a < 0) {
            tet_vert_a = tet_verts.size();
            tet_verts.append(tet_centers[tet_i]);
          }
          loops_a.append({(unsigned int)tet_vert_a});
        }
        if (create_poly_b) {
          int &tet_vert_b = tet_vert_map[tet_i][corner_b];
          if (tet_vert_b < 0) {
            tet_vert_b = tet_verts.size();
            tet_verts.append(tet_centers[tet_i]);
          }
          /* Loop order for b is inverted below */
          loops_b.append({(unsigned int)tet_vert_b});
        }
      }
      else {
        if (create_poly_a) {
          loops_a.append({(unsigned int)tet_i});
        }
        if (create_poly_b) {
          /* Loop order for b is inverted below */
          loops_b.append({(unsigned int)tet_i});
        }
      }

      /* Side containing the edge in forward direction */
      const int edge_to_side[4][4] = {{-1, 0, 3, 1}, {1, -1, 0, 2}, {0, 2, -1, 3}, {3, 1, 2, -1}};
      const int forward_side = edge_to_side[corner_a][corner_b];
      /* Move to next tet in the loop */
      tet_i = side_map[tet_i][forward_side];
      BLI_assert_msg(tet_i >= 0, "Internal edge must have close tet loop");

      if (tet_i == tet_start) {
        break;
      }
    }

    if (create_poly_a) {
      poly_a.totloop = loops_a.size() - poly_a.loopstart;
      polys_a.append(poly_a);
    }
    if (create_poly_b) {
      poly_b.totloop = (loops_b.size() - poly_b.loopstart) ;
      polys_b.append(poly_b);
    }
  }

  const Span<MVert> vert_span = separate_verts ? tet_verts.as_span() : tet_centers.as_span();
  Mesh *mesh = BKE_mesh_new_nomain(
      vert_span.size(), 0, 0, loops_a.size() + loops_b.size(), polys_a.size() + polys_b.size());
  mesh->verts_for_write().copy_from(vert_span);

  MutableSpan<MPoly> mesh_polys_a = mesh->polys_for_write().slice(0, polys_a.size());
  MutableSpan<MPoly> mesh_polys_b = mesh->polys_for_write().slice(polys_a.size(), polys_b.size());
  mesh_polys_a.copy_from(polys_a);
  for (const int i : mesh_polys_b.index_range()) {
    MPoly &poly = mesh_polys_b[i];
    /* Fix loopstart offset for the inverse faces */
    poly.loopstart = polys_b[i].loopstart + loops_a.size();
    poly.totloop = polys_b[i].totloop;
  }

  MutableSpan<MLoop> mesh_loops_a = mesh->loops_for_write().slice(0, loops_a.size());
  MutableSpan<MLoop> mesh_loops_b = mesh->loops_for_write().slice(loops_a.size(), loops_b.size());
  mesh_loops_a.copy_from(loops_a);
  for (const int poly_i : polys_b.index_range()) {
    const MPoly &poly = polys_b[poly_i];
    IndexRange loop_range(poly.loopstart, poly.totloop);
    for (const int loop_i : loop_range) {
      /* Invert the loop for b for correct winding */
      mesh_loops_b[loop_i] = loops_b[loop_range.start() + loop_range.last() - loop_i];
    }
  }
  BKE_mesh_calc_edges(mesh, false, false);

  BKE_mesh_validate(mesh, true, true);

  return mesh;
}

}  // namespace simplex

namespace delaunay {

void find_enclosing_simplex(const Span<float3> positions, float3 simplex[4])
{
  if (positions.is_empty()) {
    simplex[0] = simplex[1] = simplex[2] = simplex[3] = float3(0.0f);
    return;
  }

  /* Normals of a regular tetrahedron */
  const float3 n0{1.0f, 0.0f, -1.0f / sqrt(2.0f)};
  const float3 n1{-1.0f, 0.0f, -1.0f / sqrt(2.0f)};
  const float3 n2{0.0f, 1.0f, 1.0f / sqrt(2.0f)};
  const float3 n3{0.0f, -1.0f, 1.0f / sqrt(2.0f)};

  /* Find maximum distance from the origin for each plane */
  float d0 = math::dot(positions[0], n0);
  float d1 = math::dot(positions[0], n1);
  float d2 = math::dot(positions[0], n2);
  float d3 = math::dot(positions[0], n3);
  for (const float3 &p : positions.drop_front(1)) {
    d0 = std::max(d0, math::dot(p, n0));
    d1 = std::max(d1, math::dot(p, n1));
    d2 = std::max(d2, math::dot(p, n2));
    d3 = std::max(d3, math::dot(p, n3));
  }

  /* 3-plane intersections form the vertices of the simplex */
  const float3x3 A012(Array<float>{n0.x, n1.x, n2.x, n0.y, n1.y, n2.y, n0.z, n1.z, n2.z}.data());
  const float3x3 A103(Array<float>{n1.x, n0.x, n3.x, n1.y, n0.y, n3.y, n1.z, n0.z, n3.z}.data());
  const float3x3 A213(Array<float>{n2.x, n1.x, n3.x, n2.y, n1.y, n3.y, n2.z, n1.z, n3.z}.data());
  const float3x3 A302(Array<float>{n3.x, n0.x, n2.x, n3.y, n0.y, n2.y, n3.z, n0.z, n2.z}.data());
  simplex[0] = A012.inverted() * float3{d0, d1, d2};
  simplex[1] = A103.inverted() * float3{d1, d0, d3};
  simplex[2] = A213.inverted() * float3{d2, d1, d3};
  simplex[3] = A302.inverted() * float3{d3, d0, d2};
}

Array<int4> tet_build_side_to_tet_map(Span<int4> tets)
{
  /* Maps triangles to the two tet indices on either side */
  Map<int3, int2> tet_map;
  for (const int tet_i : tets.index_range()) {
    int3 tri[4];
    topology::simplex_triangles(tets[tet_i], tri[0], tri[1], tri[2], tri[3]);
    for (const int k : IndexRange(4)) {
      bool flipped;
      const int3 key = get_unique_tri(tri[k], flipped);
      /* Note: lookup_or_add can invalidate references to earlier values */
      int2 &val = tet_map.lookup_or_add(key, int2(-1, -1));
      /* Sorted indices are clock-wise for one adjacent tet and counter-clock-wise for the other.
       * If sorting flips the winding order we use the y index, otherwise use x. */
      int &idx = flipped ? val.y : val.x;
      BLI_assert_msg(idx < 0, "Triangle already has entry");
      idx = tet_i;
    }
  }

  /* Find indices of adjacent tets */
  Array<int4> result(tets.size());
  for (const int tet_i : tets.index_range()) {
    int3 tri0, tri1, tri2, tri3;
    topology::simplex_triangles(tets[tet_i], tri0, tri1, tri2, tri3);
    bool flipped0, flipped1, flipped2, flipped3;
    const int3 key0 = get_unique_tri(tri0, flipped0);
    const int3 key1 = get_unique_tri(tri1, flipped1);
    const int3 key2 = get_unique_tri(tri2, flipped2);
    const int3 key3 = get_unique_tri(tri3, flipped3);
    const int2 val0 = tet_map.lookup(key0);
    const int2 val1 = tet_map.lookup(key1);
    const int2 val2 = tet_map.lookup(key2);
    const int2 val3 = tet_map.lookup(key3);
    /* Sorted indices are clock-wise for one adjacent tet and counter-clock-wise for the other.
     * If sorting flips the winding order the adjacent tet is the x index, otherwise it's y. */
    const int neighbor0 = flipped0 ? val0.x : val0.y;
    const int neighbor1 = flipped1 ? val1.x : val1.y;
    const int neighbor2 = flipped2 ? val2.x : val2.y;
    const int neighbor3 = flipped3 ? val3.x : val3.y;
    result[tet_i] = int4(neighbor0, neighbor1, neighbor2, neighbor3);
  }

  return result;
}

int tet_find_containing(Span<float3> positions, Span<int4> tets, const float3 &point)
{
  for (const int tet_i : tets.index_range()) {
    if (topology::simplex_contains_point(positions, tets[tet_i], point)) {
      return tet_i;
    }
  }
  return -1;
}

int tet_find_circumscribed(Span<float3> positions, Span<int4> tets, const float3 &point)
{
  for (const int tet_i : tets.index_range()) {
    const int4 &tet = tets[tet_i];
    float3 center;
    if (topology::simplex_circumcenter(
            positions[tet[0]], positions[tet[1]], positions[tet[2]], positions[tet[3]], center)) {
      if (math::distance_squared(point, center) <=
          math::distance_squared(positions[tet[0]], center)) {
        return tet_i;
      }
    }
  }
  return -1;
}

static bool tet_isect_ray(const float3 &ray_src,
                          const float3 &ray_dir,
                          const float3 &vert,
                          const float3 &nor,
                          float &r_lambda)
{
  const float dot = math::dot(ray_dir, nor);
  if (dot <= 0.0f) {
    return false;
  }
  r_lambda = math::dot(vert - ray_src, nor) / dot;
  return true;
}

int tet_find_circumscribed_from_known(Span<float3> positions,
                        Span<int4> tets,
                        Span<int4> side_map,
                        const int src_tet,
                        const float3 &dst_point)
{
//#define DEBUG_PRINT

#ifdef DEBUG_PRINT
  {
    std::cout << "points = [" << std::endl;
    for (const float3 &p : positions) {
      std::cout << "  (" << p.x << ", " << p.y << ", " << p.z << ")," << std::endl;
    }
    std::cout << "]" << std::endl;
    std::cout << "tets = [" << std::endl;
    for (const Simplex &s : tets) {
      std::cout << "  [" << s.v[0] << ", " << s.v[1] << ", " << s.v[2] << ", " << s.v[3] << "],"
                << std::endl;
    }
    std::cout << "]" << std::endl;
    std::flush(std::cout);
  }
#endif

#ifdef DEBUG_PRINT
  {
    std::cout << "events = [" << std::endl;
  }
#endif
  const float epsilon = 1e-6f;
  int tet_i = src_tet;
  /* True for checked simplices.
   * Makes sure we only check each simplex once.
   * If simplex is coplanar we may otherwise flip back and forth
   * between a pair of triangles indefinitely.
   */
  Array<bool> checked(tets.size(), false);
  while (tet_i >= 0) {
    const int4 &tet = tets[tet_i];

    float3 v[4], n[4];
    topology::simplex_geometry(positions, tet, v[0], v[1], v[2], v[3], n[0], n[1], n[2], n[3]);

    float3 center;
    if (topology::simplex_circumcenter(v[0], v[1], v[2], v[3], center) &&
        math::distance_squared(dst_point, center) <= math::distance_squared(v[0], center)) {
      /* Found violated simplex */
      break;
    }
    checked[tet_i] = true;

    const float3 centroid = 0.25f * (v[0] + v[1] + v[2] + v[3]);
    const float3 dir = dst_point - centroid;
    float min_lambda = FLT_MAX;
    int next_tet = -1;
    const int4 neighbor = side_map[tet_i];
    for (const int k : IndexRange(4)) {
      if (neighbor[k] >= 0 && !checked[neighbor[k]]) {
        float lambda;
        if (tet_isect_ray(centroid, dir, v[k], n[k], lambda)) {
          if (lambda > -epsilon && lambda < min_lambda) {
            min_lambda = lambda;
            next_tet = neighbor[k];
          }
        }
      }
    }

#ifdef DEBUG_PRINT
    {
      std::cout << "(";
      /* tet points */
      std::cout << "(" << v[0].x << ", " << v[0].y << ", " << v[0].z << "),";
      std::cout << "(" << v[1].x << ", " << v[1].y << ", " << v[1].z << "),";
      std::cout << "(" << v[2].x << ", " << v[2].y << ", " << v[2].z << "),";
      std::cout << "(" << v[3].x << ", " << v[3].y << ", " << v[3].z << "),";
      /* tet normals */
      std::cout << "(" << n[0].x << ", " << n[0].y << ", " << n[0].z << "),";
      std::cout << "(" << n[1].x << ", " << n[1].y << ", " << n[1].z << "),";
      std::cout << "(" << n[2].x << ", " << n[2].y << ", " << n[2].z << "),";
      std::cout << "(" << n[3].x << ", " << n[3].y << ", " << n[3].z << "),";
      /* tet hit_point */
      const float3 hit = src_point + dir * min_lambda;
      std::cout << "(" << hit.x << ", " << hit.y << ", " << hit.z << "),";
      std::cout << min_lambda << ", ";
      std::cout << ")," << std::endl;
      std::flush(std::cout);
    }
#endif

    tet_i = next_tet;
  }
#ifdef DEBUG_PRINT
  {
    std::cout << "]" << std::endl;
  }
#endif

  return tet_i;

#undef DEBUG_PRINT
}

Array<bool> check_delaunay_condition(const Span<float3> positions,
                                     const Span<int4> tets,
                                     const Span<int4> side_map,
                                     const float3 &new_point,
                                     const int containing_simplex)
{
  Array<bool> result(tets.size(), false);
  Array<bool> checked(tets.size(), false);
  Stack<int> stack;

  result[containing_simplex] = true;
  stack.push(containing_simplex);
  checked[containing_simplex] = true;

  while (!stack.is_empty()) {
    const int simplex_i = stack.pop();

    const int4 neighbor = side_map[simplex_i];
    for (const int k : IndexRange(4)) {
      const int simplex_k = neighbor[k];
      if (simplex_k < 0 || checked[simplex_k]) {
        continue;
      }

      const int4 &tet = tets[simplex_k];
      const float3 pos[4] = {
          positions[tet[0]], positions[tet[1]], positions[tet[2]], positions[tet[3]]};
      float3 center;
      if (topology::simplex_circumcenter(pos[0], pos[1], pos[2], pos[3], center)) {
        if (math::length_squared(new_point - center) <= math::length_squared(pos[0] - center)) {
          /* Simplex circum-sphere contains the point,
           * mark it and add to the stack for checking neighbors.
           */
          result[simplex_k] = true;
          stack.push(simplex_k);
        }
      }
      else {
        /* Degenerate triangle, circumradius is infinite, contains all points */
        result[simplex_k] = true;
        stack.push(simplex_k);
      }
      checked[simplex_k] = true;
    }
  }

  return result;
}

void tet_fan_construct(Array<int4> &tets,
                       Array<int4> &side_map,
                       Span<bool> replace,
                       int point,
                       IndexRange &r_old_simplex_range,
                       IndexRange &r_new_simplex_range)
{
  /* Count removed tets and open boundary triangles, and build a map of new indices. */
  int removed_tets = 0;
  int boundary_triangles = 0;
  Array<int> tets_map(tets.size());
  for (const int tet_i : tets.index_range()) {
    if (!replace[tet_i]) {
      tets_map[tet_i] = tet_i - removed_tets;
      continue;
    }

    /* Sides mapping to this tet will generate a new tet at the end. */
    tets_map[tet_i] = -1;
    ++removed_tets;

    int3 tri[4];
    topology::simplex_triangles(tets[tet_i], tri[0], tri[1], tri[2], tri[3]);

    const int4 neighbor = side_map[tet_i];
    for (const int k : IndexRange(4)) {
      /* Count sides with remaining neighbors, ignore internal triangles. */
      if (neighbor[k] < 0 || !replace[neighbor[k]]) {
        ++boundary_triangles;
      }
    }
  }

  /* New arrays */
  const int new_tets_count = tets.size() - removed_tets + boundary_triangles;
  const int boundary_start = tets.size() - removed_tets;
  Array<int4> new_tets(new_tets_count);
  Array<int4> new_side_map(new_tets_count);
  /* Maps edges on the boundary to new tet indices.
   * Each edge forms a triangle with the new point, maps to different tets per side.
   */
  Map<int2, int2> boundary_edge_map;
  /* XXX over-allocation */
  boundary_edge_map.reserve(3 * boundary_triangles);

  /* Populate tets */
  int old_tet_cur = 0;
  int new_tet_cur = boundary_start;
  for (const int tet_i : tets.index_range()) {
    if (replace[tet_i]) {
      /* Generate new tet fan */
      int3 tri[4];
      topology::simplex_triangles(tets[tet_i], tri[0], tri[1], tri[2], tri[3]);
      const int4 neighbor = side_map[tet_i];
      for (const int k : IndexRange(4)) {
        /* Skip open or internal faces */
        if (neighbor[k] >= 0 && replace[neighbor[k]]) {
          continue;
        }

        new_tets[new_tet_cur] = int4{tri[k][0], tri[k][1], tri[k][2], point};

        /* Entries in the boundary edge map to find neighboring tets. */
        bool flipped0, flipped1, flipped2;
        int2 key0 = get_unique_edge(int2(tri[k][0], tri[k][1]), flipped0);
        int2 key1 = get_unique_edge(int2(tri[k][1], tri[k][2]), flipped1);
        int2 key2 = get_unique_edge(int2(tri[k][2], tri[k][0]), flipped2);
        int2 &edge0 = boundary_edge_map.lookup_or_add(key0, int2(-1, -1));
        int2 &edge1 = boundary_edge_map.lookup_or_add(key1, int2(-1, -1));
        int2 &edge2 = boundary_edge_map.lookup_or_add(key2, int2(-1, -1));
        BLI_assert(flipped0 ? edge0.y < 0 : edge0.x < 0);
        BLI_assert(flipped1 ? edge1.y < 0 : edge1.x < 0);
        BLI_assert(flipped2 ? edge2.y < 0 : edge2.x < 0);
        int &tet0 = flipped0 ? edge0.y : edge0.x;
        int &tet1 = flipped1 ? edge1.y : edge1.x;
        int &tet2 = flipped2 ? edge2.y : edge2.x;
        tet0 = new_tet_cur;
        tet1 = new_tet_cur;
        tet2 = new_tet_cur;

        ++new_tet_cur;
      }
    }
    else {
      /* Copy old tet */
      new_tets[old_tet_cur] = tets[tet_i];
      ++old_tet_cur;
    }
  }
  BLI_assert(old_tet_cur == boundary_start);
  BLI_assert(new_tet_cur == new_tets_count);

  /* Update adjacency map */
  old_tet_cur = 0;
  new_tet_cur = boundary_start;
  for (const int tet_i : tets.index_range()) {
    int3 tri[4];
    topology::simplex_triangles(tets[tet_i], tri[0], tri[1], tri[2], tri[3]);

    if (replace[tet_i]) {
      /* Generate new tet fan */
      const int4 neighbor = side_map[tet_i];
      for (const int k : IndexRange(4)) {
        /* Skip internal faces */
        if (neighbor[k] >= 0 && replace[neighbor[k]]) {
          continue;
        }

        /* Adds a new tet for each boundary side */
        int4 &new_neighbor = new_side_map[new_tet_cur];
        /* Use edge map to determine adjacent tet indices */
        bool flipped0, flipped1, flipped2;
        const int2 key0 = get_unique_edge(int2(tri[k][0], tri[k][1]), flipped0);
        const int2 key1 = get_unique_edge(int2(tri[k][1], tri[k][2]), flipped1);
        const int2 key2 = get_unique_edge(int2(tri[k][2], tri[k][0]), flipped2);
        const int2 edge0 = boundary_edge_map.lookup(key0);
        const int2 edge1 = boundary_edge_map.lookup(key1);
        const int2 edge2 = boundary_edge_map.lookup(key2);
        new_neighbor = int4(neighbor[k] >= 0 ? tets_map[neighbor[k]] : -1,
                            flipped0 ? edge0.x : edge0.y,
                            flipped1 ? edge1.x : edge1.y,
                            flipped2 ? edge2.x : edge2.y);
        ++new_tet_cur;
      }
    }
    else {
      /* Copy old tet */
      const int4 neighbor = side_map[tet_i];
      int4 &new_neighbor = new_side_map[old_tet_cur];
      for (const int k : IndexRange(4)) {
        if (neighbor[k] < 0) {
          new_neighbor[k] = -1;
        }
        else if (replace[neighbor[k]]) {
          bool flipped;
          const int2 key = get_unique_edge(int2(tri[k][0], tri[k][1]), flipped);
          /* Use boundary edge map to look up index of the new neighboring tet.
           * Edge direction matches triangle winding of the new tet.
           */
          const int2 edge = boundary_edge_map.lookup(key);
          new_neighbor[k] = flipped ? edge.x : edge.y;
        }
        else {
          new_neighbor[k] = tets_map[neighbor[k]];
        }
      }
      ++old_tet_cur;
    }
  }
  BLI_assert(old_tet_cur == boundary_start);
  BLI_assert(new_tet_cur == new_tets_count);

  tets = std::move(new_tets);
  side_map = std::move(new_side_map);
  r_old_simplex_range = IndexRange(0, boundary_start);
  r_new_simplex_range = IndexRange(boundary_start, boundary_triangles);
}

void tetrahedralize_points(Span<float3> positions,
                           Array<float3> &r_points,
                           Array<int4> &r_tets,
                           bool keep_hull)
{
  const int num_pos = positions.size();
  const IndexRange orig_points(num_pos);

  /* Find enclosing simplex and include its points */
  float3 bounds[4];
  find_enclosing_simplex(positions, bounds);
  /* Scale enclosure to produce better conditioned tetrahedron on the sides */
  {
    const float3 center = 0.25f * (bounds[0] + bounds[1] + bounds[2] + bounds[3]);
    const float scale = 1.05f;
    bounds[0] = (bounds[0] - center) * scale + center;
    bounds[1] = (bounds[1] - center) * scale + center;
    bounds[2] = (bounds[2] - center) * scale + center;
    bounds[3] = (bounds[3] - center) * scale + center;
  }

  /* Internal array with additional hull points */
  Array<float3> hull_points;
  Array<float3> &used_points = keep_hull ? r_points : hull_points;
  used_points.reinitialize(num_pos + 4);
  used_points.as_mutable_span().slice(0, num_pos).copy_from(positions);
  used_points.as_mutable_span().slice(num_pos, 4).copy_from(Span(bounds, 4));
  /* If hull isn't kept then output the original points array */
  if (!keep_hull) {
    r_points = positions;
  }

  r_tets.reinitialize(1);
  r_tets[0] = int4{num_pos + 0, num_pos + 1, num_pos + 2, num_pos + 3};
  Array<int4> side_map{{-1, -1, -1, -1}};

  int containing_simplex = 0;
  for (const int point_i : orig_points) {
    if (point_i > 0) {
      containing_simplex = tet_find_circumscribed_from_known(
          used_points, r_tets, side_map, containing_simplex, used_points[point_i]);
      if (containing_simplex < 0) {
        /* TODO should not happen (point outside enclosing simplex), issue a warning */
        continue;
      }
    }

    Array<bool> replace = check_delaunay_condition(
        used_points, r_tets, side_map, used_points[point_i], containing_simplex);

    IndexRange old_range, new_range;
    tet_fan_construct(r_tets, side_map, replace, point_i, old_range, new_range);
    if (!new_range.is_empty()) {
      containing_simplex = new_range.first();
    }
  }

  if (!keep_hull) {
    /* Remove any simplices of the enclosing hull. */
    int tets_num = 0;
    Array<int4> clean_simplices(r_tets.size());
    for (const int tet_i : r_tets.index_range()) {
      const int4 &tet = r_tets[tet_i];
      if (orig_points.contains(tet[0]) && orig_points.contains(tet[1]) &&
          orig_points.contains(tet[2]) && orig_points.contains(tet[3])) {
        clean_simplices[tets_num] = tet;
        ++tets_num;
      }
    }
    r_tets = clean_simplices.as_span().slice(0, tets_num);
  }
}

}  // namespace delaunay

}  // namespace blender::geometry
