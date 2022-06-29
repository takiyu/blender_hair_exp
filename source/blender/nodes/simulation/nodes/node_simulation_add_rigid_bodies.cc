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

namespace blender::nodes::node_simulation_add_rigid_bodies_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry"))
      .supported_type(
          {GEO_COMPONENT_TYPE_MESH, GEO_COMPONENT_TYPE_POINT_CLOUD, GEO_COMPONENT_TYPE_INSTANCES});
  b.add_input<decl::Bool>(N_("Selection")).default_value(true).supports_field().hide_value();
  b.add_input<decl::Int>(N_("Shape Index")).default_value(0).supports_field();
  b.add_input<decl::Int>(N_("Collision Group")).default_value(1).supports_field();
  b.add_input<decl::Int>(N_("Collision Mask")).default_value(3).supports_field();

  b.add_output<decl::Geometry>(N_("Geometry"));
}

static void node_exec(GeoNodeExecParams params)
{
  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  Field<bool> selection = params.get_input<Field<bool>>("Selection");
  Field<int> shape_index = params.get_input<Field<int>>("Shape Index");
  Field<int> collision_group = params.get_input<Field<int>>("Collision Group");
  Field<int> collision_mask = params.get_input<Field<int>>("Collision Mask");

  params.used_named_attribute(simulation::rb_shape_index_attribute_name, eNamedAttrUsage::Write);
  params.used_named_attribute(simulation::rb_flag_attribute_name, eNamedAttrUsage::Write);

  simulation_node_capture_rigidbody_field(
      geometry_set, simulation::rb_shape_index_attribute_name, selection, shape_index);

  static fn::CustomMF_SI_SO<int, int> flag_fn("init rigid body flag", [](int old_flag) {
    int new_flag = old_flag | simulation::RB_FLAG_ENABLE;
    if (old_flag & simulation::RB_FLAG_ENABLE) {
      new_flag |= simulation::RB_FLAG_INITIALIZE;
    }
    return new_flag;
  });
  Field<int> old_flag_field{std::make_unique<AttributeFieldInput>(
      AttributeFieldInput(simulation::rb_flag_attribute_name, CPPType::get<int>()))};
  Field<int> new_flag_field{
      std::make_shared<FieldOperation>(FieldOperation(flag_fn, {old_flag_field}))};
  simulation_node_capture_rigidbody_field(
      geometry_set, simulation::rb_flag_attribute_name, selection, new_flag_field);

  simulation_node_capture_rigidbody_field(
      geometry_set, simulation::rb_collision_group_attribute_name, selection, collision_group);
  simulation_node_capture_rigidbody_field(
      geometry_set, simulation::rb_collision_mask_attribute_name, selection, collision_mask);

  params.set_output("Geometry", std::move(geometry_set));
}

}  // namespace blender::nodes::node_simulation_add_rigid_bodies_cc

void register_node_type_simulation_add_rigid_bodies()
{
  namespace file_ns = blender::nodes::node_simulation_add_rigid_bodies_cc;

  static bNodeType ntype;

  simulation_node_type_base(&ntype, SIMULATION_NODE_ADD_RIGID_BODIES, "Add Rigid Bodies", NODE_CLASS_GEOMETRY);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_exec;
  nodeRegisterType(&ntype);
}
