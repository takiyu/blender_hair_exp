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
using blender::geometry::simplex::SimplexToMeshMode;

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "mode", 0, "", ICON_NONE);
  uiItemR(layout, ptr, "output", 0, "", ICON_NONE);
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = GEO_NODE_SIMPLEX_TO_MESH_SEPARATE;
  node->custom2 = GEO_NODE_SIMPLEX_TO_MESH_TETRAHEDRA;
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

static SimplexToMeshMode get_face_mode(GeometryNodeSimplexToMeshMode mode_dna)
{
  switch (mode_dna) {
    case GEO_NODE_SIMPLEX_TO_MESH_SEPARATE:
      return SimplexToMeshMode::Separate;
    case GEO_NODE_SIMPLEX_TO_MESH_SHARED_VERTS:
      return SimplexToMeshMode::SharedVerts;
    case GEO_NODE_SIMPLEX_TO_MESH_SHARED_FACES:
      return SimplexToMeshMode::SharedFaces;
  }
}

static void node_geo_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  const SimplexToMeshMode mode = get_face_mode(
      (GeometryNodeSimplexToMeshMode)params.node().custom1);
  const GeometryNodeSimplexToMeshOutput output_type =
      (GeometryNodeSimplexToMeshOutput)params.node().custom2;

  const SimplexGeometry *geometry = geometry_set.get_simplex_for_read();
  Mesh *mesh;
  if (geometry) {
    switch (output_type) {
      case GEO_NODE_SIMPLEX_TO_MESH_TETRAHEDRA:
        mesh = geometry::simplex::simplex_to_mesh(
            geometry->positions(), geometry->simplex_vertices(), mode);
        break;
      case GEO_NODE_SIMPLEX_TO_MESH_DUAL:
        mesh = geometry::simplex::simplex_to_dual_mesh(
            geometry->positions(), geometry->simplex_vertices(), mode);
        break;
      default:
        BLI_assert_unreachable();
        mesh = nullptr; 
        break;
    }
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
