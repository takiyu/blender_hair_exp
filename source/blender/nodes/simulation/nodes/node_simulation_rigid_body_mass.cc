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

namespace blender::nodes::node_simulation_rigid_body_mass_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
}

static void node_exec(GeoNodeExecParams params)
{
}

}  // namespace blender::nodes::node_simulation_apply_rigid_body_force_cc

void register_node_type_simulation_rigid_body_mass()
{
  namespace file_ns = blender::nodes::node_simulation_rigid_body_mass_cc;

  static bNodeType ntype;

  simulation_node_type_base(
      &ntype, SIMULATION_NODE_RIGID_BODY_MASS, "Mass", NODE_CLASS_INPUT);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_exec;
  nodeRegisterType(&ntype);
}
