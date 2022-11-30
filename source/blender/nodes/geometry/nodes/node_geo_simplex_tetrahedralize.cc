/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_simplex_types.h"

#include "BKE_curves.hh"
#include "BKE_mesh.h"
#include "BKE_mesh_simplex.hh"

#include "GEO_mesh_delaunay.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_simplex_tetrahedralize_cc {

using blender::bke::SimplexGeometry;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Points"))
      .only_realized_data()
      .supported_type({GEO_COMPONENT_TYPE_MESH, GEO_COMPONENT_TYPE_POINT_CLOUD});
  b.add_output<decl::Geometry>(N_("Geometry"));
}

static void tetrahedralize(const GeometrySet &geometry_set,
                           Array<float3> &r_points,
                           Array<int4> &r_tets)
{
  int span_count = 0;
  int count = 0;
  int total_num = 0;

  Span<float3> positions_span;

  if (const MeshComponent *component = geometry_set.get_component_for_read<MeshComponent>()) {
    count++;
    if (const VArray<float3> positions = component->attributes()->lookup<float3>(
            "position", ATTR_DOMAIN_POINT)) {
      if (positions.is_span()) {
        span_count++;
        positions_span = positions.get_internal_span();
      }
      total_num += positions.size();
    }
  }

  if (const PointCloudComponent *component =
          geometry_set.get_component_for_read<PointCloudComponent>()) {
    count++;
    if (const VArray<float3> positions = component->attributes()->lookup<float3>(
            "position", ATTR_DOMAIN_POINT)) {
      if (positions.is_span()) {
        span_count++;
        positions_span = positions.get_internal_span();
      }
      total_num += positions.size();
    }
  }

  if (const Curves *curves_id = geometry_set.get_curves_for_read()) {
    count++;
    span_count++;
    const bke::CurvesGeometry &curves = bke::CurvesGeometry::wrap(curves_id->geometry);
    positions_span = curves.evaluated_positions();
    total_num += positions_span.size();
  }

  if (count == 0) {
    return;
  }

  /* If there is only one positions virtual array and it is already contiguous, avoid copying
   * all of the positions and instead pass the span directly to the conversion function. */
  if (span_count == 1 && count == 1) {
    return geometry::delaunay::tetrahedralize_points(positions_span, r_points, r_tets);
  }

  Array<float3> positions(total_num);
  int offset = 0;

  if (const MeshComponent *component = geometry_set.get_component_for_read<MeshComponent>()) {
    if (const VArray<float3> varray = component->attributes()->lookup<float3>("position",
                                                                              ATTR_DOMAIN_POINT)) {
      varray.materialize(positions.as_mutable_span().slice(offset, varray.size()));
      offset += varray.size();
    }
  }

  if (const PointCloudComponent *component =
          geometry_set.get_component_for_read<PointCloudComponent>()) {
    if (const VArray<float3> varray = component->attributes()->lookup<float3>("position",
                                                                              ATTR_DOMAIN_POINT)) {
      varray.materialize(positions.as_mutable_span().slice(offset, varray.size()));
      offset += varray.size();
    }
  }

  if (const Curves *curves_id = geometry_set.get_curves_for_read()) {
    const bke::CurvesGeometry &curves = bke::CurvesGeometry::wrap(curves_id->geometry);
    Span<float3> array = curves.evaluated_positions();
    positions.as_mutable_span().slice(offset, array.size()).copy_from(array);
    offset += array.size();
  }

  return geometry::delaunay::tetrahedralize_points(positions, r_points, r_tets);
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Points");

  Array<float3> points;
  Array<int4> tets;
  tetrahedralize(geometry_set, points, tets);

  SimplexGeometry *geometry = new SimplexGeometry(points.size(), tets.size());
  geometry->positions_for_write().copy_from(points);
  geometry->tetrahedrons_for_write().copy_from(tets);

  params.set_output("Geometry", GeometrySet::create_with_simplex(geometry));
}

}  // namespace blender::nodes::node_geo_simplex_tetrahedralize_cc

void register_node_type_geo_simplex_tetrahedralize()
{
  namespace file_ns = blender::nodes::node_geo_simplex_tetrahedralize_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SIMPLEX_TETRAHEDRALIZE, "Tetrahedralize", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
