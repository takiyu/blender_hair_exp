/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"
#include "DNA_simplex_types.h"

#include "BKE_curves.hh"
#include "BKE_material.h"
#include "BKE_mesh.h"
#include "BKE_mesh_simplex.hh"

#include "GEO_mesh_delaunay.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_simplex_to_mesh_cc {

using blender::bke::SimplexGeometry;
using blender::geometry::simplex::SimplexFaceMode;

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "face_mode", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = GEO_NODE_SIMPLEX_TO_MESH_FACES_ALL;
}

static void node_declare(NodeDeclarationBuilder &b)
{
  /* XXX Supported types: We need the SimplexComponent for topology
   * and at least one of MeshComponent/PointCloudComponent/CurveComponent for positions.
   * The conditions are too complex to fit into a simple "allowed types" list.
   * Ideally tetrahedra might become their own domain in the MeshComponent,
   * but that is a lot of work for a marginal benefit, so for now it remains
   * its own component and we check for a position attribute from another component.
   */
  b.add_input<decl::Geometry>(N_("Geometry"))
      .only_realized_data()
      .supported_type({GEO_COMPONENT_TYPE_MESH,
                       GEO_COMPONENT_TYPE_CURVE,
                       GEO_COMPONENT_TYPE_POINT_CLOUD,
                       GEO_COMPONENT_TYPE_SIMPLEX});
  b.add_output<decl::Geometry>(N_("Mesh"));
}

static SimplexFaceMode get_face_mode(
    GeometryNodeSimpleToMeshFaceMode face_mode_dna)
{
  switch (face_mode_dna) {
    case GEO_NODE_SIMPLEX_TO_MESH_FACES_ALL:
      return SimplexFaceMode::All;
    case GEO_NODE_SIMPLEX_TO_MESH_FACES_SHARED:
      return SimplexFaceMode::Shared;
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  const SimplexFaceMode face_mode = get_face_mode((GeometryNodeSimpleToMeshFaceMode)params.node().custom1);

  const SimplexGeometry *geometry = geometry_set.get_simplex_for_read();
  if (geometry) {
    Mesh *mesh = geometry::simplex::simplex_to_mesh(
        geometry->positions(), geometry->simplex_vertices(), face_mode);
    BKE_id_material_eval_ensure_default_slot(&mesh->id);
    params.set_output("Mesh", GeometrySet::create_with_mesh(mesh));
  }
  else {
    params.set_default_remaining_outputs();
  }
}

}  // namespace blender::nodes::node_geo_simplex_to_mesh_cc

void register_node_type_geo_simplex_to_mesh()
{
  namespace file_ns = blender::nodes::node_geo_simplex_to_mesh_cc;

  static bNodeType ntype;

  geo_node_type_base(&ntype, GEO_NODE_SIMPLEX_TO_MESH, "Simplex To Mesh", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.initfunc = file_ns::node_init;
  nodeRegisterType(&ntype);
}
