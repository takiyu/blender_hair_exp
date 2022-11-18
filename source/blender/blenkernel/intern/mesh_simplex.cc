/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 */

#include "BKE_mesh_simplex.hh"

#include "BLI_math_vector.hh"

namespace blender::bke {

SimplexGeometry::SimplexGeometry() : SimplexGeometry(0)
{
}

SimplexGeometry::SimplexGeometry(int simplex_num)
{
  ((::SimplexGeometry *)this)->simplex_num = simplex_num;
  ((::SimplexGeometry *)this)->simplices = (Simplex *)MEM_calloc_arrayN(
      this->simplex_num(), sizeof(Simplex), __func__);
}

static void copy_simplex_geometry(SimplexGeometry &dst, const SimplexGeometry &src)
{
  ::SimplexGeometry &c_dst = (::SimplexGeometry &)dst;
  const ::SimplexGeometry &c_src = (const ::SimplexGeometry &)src;

  c_dst.simplex_num = c_src.simplex_num;
  c_dst.simplices = (Simplex *)MEM_malloc_arrayN(c_dst.simplex_num, sizeof(Simplex), __func__);
  dst.simplices_for_write().copy_from(src.simplices());

  dst.tag_topology_changed();
}

/* The source should be empty, but in a valid state so that using it further will work. */
static void move_simplex_geometry(SimplexGeometry &dst, SimplexGeometry &src)
{
  ::SimplexGeometry &c_dst = (::SimplexGeometry &)dst;
  ::SimplexGeometry &c_src = (::SimplexGeometry &)src;

  c_dst.simplex_num = c_src.simplex_num;
  c_src.simplex_num = 0;

  std::swap(c_dst.simplices, c_src.simplices);
  MEM_SAFE_FREE(c_src.simplices);
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
  MEM_SAFE_FREE(((::SimplexGeometry *)this)->simplices);
}

void SimplexGeometry::resize(int simplex_num)
{
  if (simplex_num != this->simplex_num()) {
    MEM_reallocN(((::SimplexGeometry *)this)->simplices, sizeof(::Simplex) * this->simplex_num());
    ((::SimplexGeometry *)this)->simplex_num = simplex_num;
  }
  this->tag_topology_changed();
}

void SimplexGeometry::tag_topology_changed()
{
}

namespace topology {

/* Evaluate geometry of a tet from vertex positions.
 * The n-th point is part of the n-th side.
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
                      float3 &n3)
{
  const unsigned int *verts = tet.v;
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

bool simplex_contains_point(const Span<float3> positions, const Simplex &tet, const float3 &point)
{
  float3 v[4], n[4];
  simplex_geometry(positions, tet, v[0], v[1], v[2], v[3], n[0], n[1], n[2], n[3]);

  const float dot0 = math::dot(point - v[0], n[0]);
  const float dot1 = math::dot(point - v[1], n[1]);
  const float dot3 = math::dot(point - v[3], n[3]);
  /* Exploit normal redundancy to save one dot product: n2 == -(n0 + n1 + n3) */
  const bool dot2 = -(dot0 + dot1 + dot3);
  return dot0 <= 0.0f && dot1 <= 0.0f && dot2 <= 0.0f && dot3 <= 0.0f;
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
