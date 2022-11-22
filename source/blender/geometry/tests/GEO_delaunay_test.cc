/* SPDX-License-Identifier: Apache-2.0 */

#include "testing/testing.h"

#include "BLI_rand.hh"

#include "BKE_mesh_simplex.hh"

#include "DNA_meshdata_types.h"

#include "GEO_mesh_delaunay.hh"

using namespace blender::bke;

namespace blender::geometry::tests {

TEST(delaunay, TetTriangles)
{
  int4 tet{0, 1, 2, 3};
  int3 tri0, tri1, tri2, tri3;
  topology::simplex_triangles(tet, tri0, tri1, tri2, tri3);
  EXPECT_EQ(int3(0, 1, 2), tri0);
  EXPECT_EQ(int3(1, 0, 3), tri1);
  EXPECT_EQ(int3(2, 1, 3), tri2);
  EXPECT_EQ(int3(3, 0, 2), tri3);
}

TEST(delaunay, TetGeometry)
{
  int4 tet{0, 1, 2, 3};
  Array<float3> positions{float3(0, 0, 0), float3(0, 1, 0), float3(1, 0, 0), float3(0, 0, 1)};
  float3 v0, v1, v2, v3;
  float3 n0, n1, n2, n3;
  topology::simplex_geometry(positions, tet, v0, v1, v2, v3, n0, n1, n2, n3);
  EXPECT_EQ(float3(0, 0, 0), v0);
  EXPECT_EQ(float3(0, 1, 0), v1);
  EXPECT_EQ(float3(1, 0, 0), v2);
  EXPECT_EQ(float3(0, 0, 1), v3);
  EXPECT_EQ(float3(0, 0, -1), n0);
  EXPECT_EQ(float3(-1, 0, 0), n1);
  EXPECT_EQ(float3(1, 1, 1), n2);
  EXPECT_EQ(float3(0, -1, 0), n3);
}

TEST(delaunay, TetCircumCenter)
{
  const float eps = 1e-3f;

  { /* Regular tetrahedron */
    float3 center;
    bool ok = topology::simplex_circumcenter(float3(1, 0, sqrt(2.0)),
                                             float3(-1, 0, sqrt(2.0)),
                                             float3(0, 1, -sqrt(2.0)),
                                             float3(0, -1, -sqrt(2.0)),
                                             center);
    EXPECT_TRUE(ok);
    EXPECT_EQ(float3(0, 0, 0), center);
  }
  { /* Irregular tetrahedron */
    float3 center;
    bool ok = topology::simplex_circumcenter(
        float3(-1, 0, -1), float3(2, 1, 0), float3(0, -1, -0.5f), float3(1, -1, 1), center);
    EXPECT_TRUE(ok);
    EXPECT_V3_NEAR(float3(0.083f, 0.722f, 0.528f), center, eps);
  }
  { /* Inverted tetrahedron */
    float3 center;
    bool ok = topology::simplex_circumcenter(
        float3(2, 1, 0), float3(-1, 0, -1), float3(0, -1, -0.5f), float3(1, -1, 1), center);
    EXPECT_TRUE(ok);
    EXPECT_V3_NEAR(float3(0.083f, 0.722f, 0.528f), center, eps);
  }
  { /* Coplanar point */
    float3 center;
    bool ok = topology::simplex_circumcenter(
        float3(-1, 0, -1),
        float3(2, 1, 0),
        float3(0, -1, -0.5f),
        float3(0.7371339201927185, -0.027775347232818604, -0.3451029062271118),
        center);
    EXPECT_FALSE(ok);
  }
  { /* Single colinear point */
    float3 center;
    bool ok = topology::simplex_circumcenter(
        float3(-1, 0, -1),
        float3(2, 1, 0),
        float3(0, -1, -0.5f),
        float3(0.6401998996734619, -0.3598001301288605, -0.3399500250816345),
        center);
    EXPECT_FALSE(ok);
  }
  { /* All points colinear */
    float3 center;
    bool ok = topology::simplex_circumcenter(
        float3(0.20784108340740204, -0.7921589612960815, -0.4480397701263428),
        float3(2, 1, 0),
        float3(0, -1, -0.5f),
        float3(0.6401998996734619, -0.3598001301288605, -0.3399500250816345),
        center);
    EXPECT_FALSE(ok);
  }
  { /* Two points collapsed */
    float3 center;
    bool ok = topology::simplex_circumcenter(
        float3(-1, 0, -1), float3(2, 1, 0), float3(0, -1, -0.5f), float3(2, 1, 0), center);
    EXPECT_FALSE(ok);
  }
  { /* Three points collapsed */
    float3 center;
    bool ok = topology::simplex_circumcenter(
        float3(-1, 0, -1), float3(2, 1, 0), float3(2, 1, 0), float3(2, 1, 0), center);
    EXPECT_FALSE(ok);
  }
  { /* All points collapsed */
    float3 center;
    bool ok = topology::simplex_circumcenter(
        float3(2, 1, 0), float3(2, 1, 0), float3(2, 1, 0), float3(2, 1, 0), center);
    EXPECT_FALSE(ok);
  }
}

TEST(delaunay, TetSideMap)
{
  { /* Empty list */
    Array<int4> tets = {};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    EXPECT_TRUE(side_map.is_empty());
  }
  { /* One tet */
    Array<int4> tets = {{0, 1, 2, 3}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    EXPECT_EQ_ARRAY(Span<int4>{{-1, -1, -1, -1}}.data(), side_map.data(), 1);
  }
  { /* Disconnected tets */
    Array<int4> tets = {{0, 1, 2, 3}, {4, 5, 6, 7}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    EXPECT_EQ_ARRAY(Span<int4>{{-1, -1, -1, -1}, {-1, -1, -1, -1}}.data(), side_map.data(), 2);
  }
  { /* One shared vertex */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 4, 5, 6}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    EXPECT_EQ_ARRAY(Span<int4>{{-1, -1, -1, -1}, {-1, -1, -1, -1}}.data(), side_map.data(), 2);
  }
  { /* One shared edge */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 2, 4, 5}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    EXPECT_EQ_ARRAY(Span<int4>{{-1, -1, -1, -1}, {-1, -1, -1, -1}}.data(), side_map.data(), 2);
  }
  { /* One shared side */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 4, 2, 3}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    EXPECT_EQ_ARRAY(Span<int4>{{-1, -1, 1, -1}, {-1, -1, -1, 0}}.data(), side_map.data(), 2);
  }
  { /* 3 tets, 2 faces shared */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 4, 2, 3}, {1, 5, 2, 4}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{{-1, -1, 1, -1}, {2, -1, -1, 0}, {-1, -1, -1, 1}}.data(), side_map.data(), 3);
  }
  {
    Array<int4> tets{{0, 2, 1, 3}, {3, 2, 1, 4}, {4, 2, 1, 5}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{{-1, -1, 1, -1}, {0, -1, 2, -1}, {1, -1, -1, -1}}.data(), side_map.data(), 3);
  }
}

TEST(delaunay, TetFind)
{
  const Array<float3> positions{float3(0, 0, 0),
                                float3(1, 0, 0),
                                float3(0, 1, 0),
                                float3(0, 0, 1),
                                float3(1, 1, 1),
                                float3(1, 2, 0)};
  const Array<int4> tets{{0, 2, 1, 3}, {3, 2, 1, 4}, {4, 2, 1, 5}};

  const Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
  EXPECT_EQ_ARRAY(
      Span<int4>{{-1, -1, 1, -1}, {0, -1, 2, -1}, {1, -1, -1, -1}}.data(), side_map.data(), 3);

  int src_tet = 1;
  float3 dst_point(0.7f, 1.3f, 0.2f);
  const int dst_tet = delaunay::tet_find_from_known(
      positions, tets, side_map, src_tet, dst_point);
  EXPECT_EQ(2, dst_tet);
}

TEST(delaunay, TetFindCoplanar)
{
  const Array<float3> positions{float3(0, 0, 0),
                                float3(1, 0, 0),
                                float3(0, 1, 0),
                                float3(1, 1, 1),
                                float3(1, 2, 0)};
  const Array<int4> tets{{0, 2, 1, 3}, {3, 2, 1, 4}};

  const Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
  EXPECT_EQ_ARRAY(
      Span<int4>{{-1, -1, 1, -1}, {0, -1, -1, -1}}.data(), side_map.data(), 2);

  int src_tet = 1;
  float3 dst_point(0.3f, 0.2f, 0.0f);
  const int dst_tet = delaunay::tet_find_from_known(
      positions, tets, side_map, src_tet, dst_point);
  EXPECT_EQ(0, dst_tet);
}

TEST(delaunay, TetFindGrid)
{
  GTEST_SKIP("Search requires valid Delaunay tets. TODO: construct a regular tetrahedral grid instead of cubes");

  /* Constructs a regular grid with 6 tets in each cell.
   * This allows finding points straight from their position, which makes it easy
   * to test correct tet finding.
   */

  const int size = 8;

  Array<float3> positions(size * size * size);
  for (const int k : IndexRange(size)) {
    for (const int j : IndexRange(size)) {
      for (const int i : IndexRange(size)) {
        positions[i + (j + k * size) * size] = float3(i, j, k);
      }
    }
  }

  Array<int4> tets((size - 1) * (size - 1) * (size - 1) * 6);
  for (const int k : IndexRange(size - 1)) {
    for (const int j : IndexRange(size - 1)) {
      for (const int i : IndexRange(size - 1)) {
        const int v000 = i + (j + k * size) * size;
        const int v100 = v000 + 1;
        const int v010 = v000 + size;
        const int v110 = v100 + size;
        const int v001 = v000 + size * size;
        const int v101 = v100 + size * size;
        const int v011 = v010 + size * size;
        const int v111 = v110 + size * size;
        IndexRange tet_range((i + (j + k * (size - 1)) * (size - 1)) * 6, 6);
        tets[tet_range[0]] = int4{v000, v010, v100, v001};
        tets[tet_range[1]] = int4{v100, v101, v001, v010};
        tets[tet_range[2]] = int4{v100, v010, v110, v101};
        tets[tet_range[3]] = int4{v111, v110, v011, v101};
        tets[tet_range[4]] = int4{v011, v110, v010, v101};
        tets[tet_range[5]] = int4{v011, v010, v001, v101};
      }
    }
  }

  auto tet_index_at = [&](const float3 &p) -> int {
    const int3 cell = int3(p);
    const IndexRange tet_range(6 * (cell.x + (cell.y + cell.z * (size - 1)) * (size - 1)), 6);
    const float3 local = p - float3(floorf(p.x), floorf(p.y), floorf(p.z));

    const bool split23 = local.y + local.z - 1.0f < 0.0f;
    const bool split01 = local.x + local.y + local.z - 1.0f < 0.0f;
    const bool split12 = local.x + local.y - 1.0f < 0.0f;
    const bool split34 = 2.0f - local.x - local.y - local.z < 0.0f;
    const bool split45 = 1.0f - local.x - local.y < 0.0f;
    int local_tet = split23 ? (split01 ? 0 : (split12 ? 1 : 2)) : (split34 ? 3 : (split45 ? 4 : 5));
    return tet_range[local_tet];
  };

  const Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);

  const int random_points = 100;
  RandomNumberGenerator rng(69428);
  for (const int i : IndexRange(random_points)) {
    float3 src_point = float3(rng.get_float(), rng.get_float(), rng.get_float()) * (size - 1);
    float3 dst_point = float3(rng.get_float(), rng.get_float(), rng.get_float()) * (size - 1);
    int src_tet = tet_index_at(src_point);
    int dst_tet = tet_index_at(dst_point);

    EXPECT_EQ(
        dst_tet,
        delaunay::tet_find_from_known(positions, tets, side_map, src_tet, dst_point));
  }
}

TEST(delaunay, EnclosingSimplex)
{
  const float eps = 1e-3f;

  { /* Empty list */
    Array<float3> points = {};
    float3 simplex[4];
    delaunay::find_enclosing_simplex(points, simplex);
    EXPECT_EQ(float3(0.0f, 0.0f, 0.0f), simplex[0]);
    EXPECT_EQ(float3(0.0f, 0.0f, 0.0f), simplex[1]);
    EXPECT_EQ(float3(0.0f, 0.0f, 0.0f), simplex[2]);
    EXPECT_EQ(float3(0.0f, 0.0f, 0.0f), simplex[3]);
  }
  { /* One point */
    Array<float3> points = {{1.0f, 1.5f, 2.0f}};
    float3 simplex[4];
    delaunay::find_enclosing_simplex(points, simplex);
    EXPECT_V3_NEAR(float3(1.0f, 1.5f, 2.0f), simplex[0], FLT_EPSILON);
    EXPECT_V3_NEAR(float3(1.0f, 1.5f, 2.0f), simplex[1], FLT_EPSILON);
    EXPECT_V3_NEAR(float3(1.0f, 1.5f, 2.0f), simplex[2], FLT_EPSILON);
    EXPECT_V3_NEAR(float3(1.0f, 1.5f, 2.0f), simplex[3], FLT_EPSILON);
  }
  { /* Two points */
    Array<float3> points = {{0.0f, -1.0f, 1.0f}, {1.0f, 1.5f, 2.0f}};
    float3 simplex[4];
    delaunay::find_enclosing_simplex(points, simplex);
    EXPECT_V3_NEAR(float3(0.146f, 2.354f, 0.793f), simplex[0], eps);
    EXPECT_V3_NEAR(float3(0.146f,-1.146f, 0.793f), simplex[1], eps);
    EXPECT_V3_NEAR(float3(-1.604f, 0.604f, 3.267f), simplex[2], eps);
    EXPECT_V3_NEAR(float3(1.896f, 0.604f, 3.268f), simplex[3], eps);
  }
  { /* Three points */
    Array<float3> points = {{0.0f, -1.0f, 1.0f}, {1.0f, 1.5f, 2.0f}, {-2.0f, 0.5f, 0.5f}};
    float3 simplex[4];
    delaunay::find_enclosing_simplex(points, simplex);
    EXPECT_V3_NEAR(float3(-1.030f, 3.530f, -0.871f), simplex[0], eps);
    EXPECT_V3_NEAR(float3(-1.030f, -2.323f, -0.871f), simplex[1], eps);
    EXPECT_V3_NEAR(float3(-3.957f, 0.604f, 3.268f), simplex[2], eps);
    EXPECT_V3_NEAR(float3(1.896f, 0.604f, 3.268f), simplex[3], eps);
  }
  { /* 100 points */
    Array<float3> points(100);
    RandomNumberGenerator rng(1623);
    for (float3 &p : points) {
      p = rng.get_unit_float3() * rng.get_float();
    }
    float3 simplex[4];
    delaunay::find_enclosing_simplex(points, simplex);
    EXPECT_V3_NEAR(float3(0.025f, 1.992f, -1.421f), simplex[0], eps);
    EXPECT_V3_NEAR(float3(0.025f, -2.018f, -1.421f), simplex[1], eps);
    EXPECT_V3_NEAR(float3(-1.980f, -0.013f, 1.414f), simplex[2], eps);
    EXPECT_V3_NEAR(float3(2.03f, -0.013f, 1.414f), simplex[3], eps);
  }
}

TEST(delaunay, TetFanConstruct)
{
  { /* Empty list */
    Array<int4> tets = {};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 0, old_tets, new_tets);
    EXPECT_EQ(0, tets.size());
    EXPECT_EQ(0, side_map.size());
  }
  { /* One tet, no replace */
    Array<int4> tets = {{0, 1, 2, 3}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {false};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 4, old_tets, new_tets);
    EXPECT_EQ_ARRAY(Span<int4>{{0, 1, 2, 3}}.data(), tets.data(), 1);
    EXPECT_EQ_ARRAY(Span<int4>{{-1, -1, -1, -1}}.data(), side_map.data(), 1);
  }
  { /* One tet */
    Array<int4> tets = {{0, 1, 2, 3}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {true};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 4, old_tets, new_tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{{0, 1, 2, 4}, {1, 0, 3, 4}, {2, 1, 3, 4}, {3, 0, 2, 4}}.data(),
                    tets.data(),
                    4);
    EXPECT_EQ_ARRAY(Span<int4>{{-1, 1, 2, 3}, {-1, 0, 3, 2}, {-1, 0, 1, 3}, {-1, 1, 0, 2}}.data(),
                    side_map.data(),
                    4);
  }
  { /* Disconnected tets */
    Array<int4> tets = {{0, 1, 2, 3}, {4, 5, 6, 7}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {false, true};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 8, old_tets, new_tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{{0, 1, 2, 3}, {4, 5, 6, 8}, {5, 4, 7, 8}, {6, 5, 7, 8}, {7, 4, 6, 8}}.data(),
        tets.data(),
        5);
    EXPECT_EQ_ARRAY(
        Span<int4>{{-1, -1, -1, -1}, {-1, 2, 3, 4}, {-1, 1, 4, 3}, {-1, 1, 2, 4}, {-1, 2, 1, 3}}
            .data(),
        side_map.data(),
        5);
  }
  { /* One shared vertex */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 4, 5, 6}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {false, true};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 7, old_tets, new_tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{{0, 1, 2, 3}, {1, 4, 5, 7}, {4, 1, 6, 7}, {5, 4, 6, 7}, {6, 1, 5, 7}}.data(),
        tets.data(),
        5);
    EXPECT_EQ_ARRAY(
        Span<int4>{{-1, -1, -1, -1}, {-1, 2, 3, 4}, {-1, 1, 4, 3}, {-1, 1, 2, 4}, {-1, 2, 1, 3}}
            .data(),
        side_map.data(),
        5);
  }
  { /* One shared edge */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 2, 4, 5}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {false, true};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 6, old_tets, new_tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{{0, 1, 2, 3}, {1, 2, 4, 6}, {2, 1, 5, 6}, {4, 2, 5, 6}, {5, 1, 4, 6}}.data(),
        tets.data(),
        5);
    EXPECT_EQ_ARRAY(
        Span<int4>{{-1, -1, -1, -1}, {-1, 2, 3, 4}, {-1, 1, 4, 3}, {-1, 1, 2, 4}, {-1, 2, 1, 3}}
            .data(),
        side_map.data(),
        5);
  }
  { /* One shared side */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 4, 2, 3}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {false, true};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 5, old_tets, new_tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{{0, 1, 2, 3}, {1, 4, 2, 5}, {4, 1, 3, 5}, {2, 4, 3, 5}, {3, 1, 2, 5}}.data(),
        tets.data(),
        5);
    EXPECT_EQ_ARRAY(
        Span<int4>{{-1, -1, 4, -1}, {-1, 2, 3, 4}, {-1, 1, 4, 3}, {-1, 1, 2, 4}, {0, 2, 1, 3}}
            .data(),
        side_map.data(),
        5);
  }
  { /* One shared side, remove 0 (shift tet 1 to position 0) */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 4, 2, 3}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {true, false};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 5, old_tets, new_tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{{1, 4, 2, 3}, {0, 1, 2, 5}, {1, 0, 3, 5}, {2, 1, 3, 5}, {3, 0, 2, 5}}.data(),
        tets.data(),
        5);
    EXPECT_EQ_ARRAY(
        Span<int4>{{-1, -1, -1, 3}, {-1, 2, 3, 4}, {-1, 1, 4, 3}, {0, 1, 2, 4}, {-1, 2, 1, 3}}
            .data(),
        side_map.data(),
        5);
  }
  { /* 3 tets, 2 faces shared */
    Array<int4> tets = {{0, 1, 2, 3}, {1, 4, 2, 3}, {1, 5, 2, 4}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {true, false, false};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 6, old_tets, new_tets);
    EXPECT_EQ_ARRAY(
        Span<int4>{
            {1, 4, 2, 3}, {1, 5, 2, 4}, {0, 1, 2, 6}, {1, 0, 3, 6}, {2, 1, 3, 6}, {3, 0, 2, 6}}
            .data(),
        tets.data(),
        6);
    EXPECT_EQ_ARRAY(Span<int4>{{1, -1, -1, 4},
                               {-1, -1, -1, 0},
                               {-1, 3, 4, 5},
                               {-1, 2, 5, 4},
                               {0, 2, 3, 5},
                               {-1, 3, 2, 4}}
                        .data(),
                    side_map.data(),
                    6);
  }
  { /* 5 tets, center tet fully surrounded */
    Array<int4> tets{{0, 1, 2, 3}, {0, 2, 1, 4}, {0, 1, 3, 5}, {1, 2, 3, 6}, {2, 0, 3, 7}};
    Array<int4> side_map = delaunay::tet_build_side_to_tet_map(tets);
    Array<bool> replace = {true, false, false, false, false};
    IndexRange old_tets, new_tets;
    delaunay::tet_fan_construct(tets, side_map, replace, 8, old_tets, new_tets);
    EXPECT_EQ_ARRAY(Span<int4>{{0, 2, 1, 4},
                                  {0, 1, 3, 5},
                                  {1, 2, 3, 6},
                                  {2, 0, 3, 7},
                                  {0, 1, 2, 8},
                                  {1, 0, 3, 8},
                                  {2, 1, 3, 8},
                                  {3, 0, 2, 8}}
                        .data(),
                    tets.data(),
                    8);
    EXPECT_EQ_ARRAY(Span<int4>{{4, -1, -1, -1},
                               {5, -1, -1, -1},
                               {6, -1, -1, -1},
                               {7, -1, -1, -1},
                               {0, 5, 6, 7},
                               {1, 4, 7, 6},
                               {2, 4, 5, 7},
                               {3, 5, 4, 6}}
                        .data(),
                    side_map.data(),
                    8);
  }
}

}  // namespace blender::geometry::tests
