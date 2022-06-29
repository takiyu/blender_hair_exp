/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"

#include "BKE_attribute_math.hh"
#include "BKE_bvhutils.h"
#include "BKE_mesh_sample.hh"
#include "BKE_rigidbody.h"
#include "BKE_rigidbody.hh"

#include "UI_interface.h"
#include "UI_resources.h"

#include "NOD_geometry_exec.hh"
#include "NOD_simulation.h"
#include "NOD_socket_search_link.hh"

#include "node_simulation_util.hh"

namespace blender::nodes::node_simulation_remove_rigid_bodies_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry"))
      .supported_type(
          {GEO_COMPONENT_TYPE_MESH, GEO_COMPONENT_TYPE_POINT_CLOUD, GEO_COMPONENT_TYPE_INSTANCES});
  b.add_input<decl::Bool>(N_("Selection")).default_value(true).supports_field().hide_value();

  b.add_output<decl::Geometry>(N_("Geometry"));
}

static void node_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  Field<bool> selection = params.get_input<Field<bool>>("Selection");

  simulation_node_clear_rigid_body_flags(
      geometry_set, selection, simulation::RB_FLAG_ENABLE | simulation::RB_FLAG_INITIALIZE);

  params.set_output("Geometry", std::move(geometry_set));
}

}  // namespace blender::nodes::node_simulation_remove_rigid_bodies_cc

void register_node_type_simulation_remove_rigid_bodies()
{
  namespace file_ns = blender::nodes::node_simulation_remove_rigid_bodies_cc;

  static bNodeType ntype;

  simulation_node_type_base(&ntype, SIMULATION_NODE_REMOVE_RIGID_BODIES, "Remove Rigid Bodies", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_exec;
  nodeRegisterType(&ntype);
}
