/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_simulation_util.hh"
#include "node_util.h"

#include "NOD_socket_search_link.hh"

#include "BLI_string_ref.hh"

#include "BKE_attribute_access.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_rigidbody.hh"

#include "FN_field.hh"
#include "FN_multi_function_builder.hh"

bool simulation_node_poll_default(bNodeType *UNUSED(ntype),
                                bNodeTree *ntree,
                                const char **r_disabled_hint)
{
  if (!STR_ELEM(ntree->idname, "SimulationNodeTree")) {
    *r_disabled_hint = TIP_("Not a simulation node tree");
    return false;
  }
  return true;
}

void simulation_node_type_base(bNodeType *ntype, int type, const char *name, short nclass)
{
  node_type_base(ntype, type, name, nclass);
  ntype->poll = simulation_node_poll_default;
  ntype->insert_link = node_insert_link_default;
  ntype->gather_link_search_ops = blender::nodes::search_link_ops_for_basic_node;
}

namespace blender::nodes {

using fn::GField;
using bke::GeometryComponentFieldContext;
using bke::WriteAttributeLookup;

static void capture_rigidbody_component_field(GeometryComponent &component,
                                              const StringRef name,
                                              const eAttrDomain domain,
                                              const fn::Field<bool> &selection_field,
                                              const GField &field)
{
  GeometryComponentFieldContext field_context{component, domain};
  const int domain_num = component.attribute_domain_num(domain);
  const IndexMask mask{IndexMask(domain_num)};

  const CPPType &type = field.cpp_type();
  const eCustomDataType data_type = bke::cpp_type_to_custom_data_type(type);

  /* Could avoid allocating a new buffer if:
   * - We are writing to an attribute that exists already.
   * - The field does not depend on that attribute (we can't easily check for that yet). */
  void *buffer = MEM_mallocN(type.size() * domain_num, __func__);

  fn::FieldEvaluator evaluator{field_context, &mask};
  evaluator.set_selection(selection_field);
  evaluator.add_with_destination(field, GMutableSpan{type, buffer, domain_num});
  evaluator.evaluate();

  component.attribute_try_delete(name);
  if (component.attribute_exists(name)) {
    WriteAttributeLookup write_attribute = component.attribute_try_get_for_write(name);
    if (write_attribute && write_attribute.domain == domain &&
        write_attribute.varray.type() == type) {
      write_attribute.varray.set_all(buffer);
      write_attribute.tag_modified_fn();
    }
    else {
      /* Cannot change type of built-in attribute. */
    }
    type.destruct_n(buffer, domain_num);
    MEM_freeN(buffer);
  }
  else {
    component.attribute_try_create(name, domain, data_type, AttributeInitMove{buffer});
  }
}

void simulation_node_capture_rigidbody_field(GeometrySet &geometry_set,
                                             const StringRef name,
                                             const fn::Field<bool> &selection_field,
                                             const GField &field)
{
  /* Run on the instances component separately to only affect the top level of instances. */
  if (geometry_set.has_instances()) {
    GeometryComponent &component = geometry_set.get_component_for_write(
        GEO_COMPONENT_TYPE_INSTANCES);
    capture_rigidbody_component_field(
        component, name, ATTR_DOMAIN_INSTANCE, selection_field, field);
  }

  geometry_set.modify_geometry_sets([&](GeometrySet &geometry_set) {
    for (const GeometryComponentType type :
          {GEO_COMPONENT_TYPE_MESH, GEO_COMPONENT_TYPE_POINT_CLOUD}) {
      if (geometry_set.has(type)) {
        GeometryComponent &component = geometry_set.get_component_for_write(type);
        capture_rigidbody_component_field(
            component, name, ATTR_DOMAIN_POINT, selection_field, field);
      }
    }
  });
}

void simulation_node_set_rigid_body_flags(GeometrySet &geometry_set,
                                          const fn::Field<bool> &selection_field,
                                          int flags)
{
  static fn::CustomMF_SI_SO<int, int> flag_fn("init rigid body flag",
                                              [flags](int old_flag) { return old_flag | flags; });
  fn::Field<int> old_flag_field{std::make_unique<bke::AttributeFieldInput>(
      bke::AttributeFieldInput(simulation::rb_flag_attribute_name, CPPType::get<int>()))};
  fn::Field<int> new_flag_field{std::make_shared<fn::FieldOperation>(
      fn::FieldOperation(flag_fn, {old_flag_field}))};
  simulation_node_capture_rigidbody_field(
      geometry_set, simulation::rb_flag_attribute_name, selection_field, new_flag_field);
}

void simulation_node_clear_rigid_body_flags(GeometrySet &geometry_set,
                                            const fn::Field<bool> &selection_field,
                                            int flags)
{
  static fn::CustomMF_SI_SO<int, int> flag_fn("init rigid body flag",
                                              [flags](int old_flag) { return old_flag & (~flags); });
  fn::Field<int> old_flag_field{std::make_unique<bke::AttributeFieldInput>(
      bke::AttributeFieldInput(simulation::rb_flag_attribute_name, CPPType::get<int>()))};
  fn::Field<int> new_flag_field{
      std::make_shared<fn::FieldOperation>(fn::FieldOperation(flag_fn, {old_flag_field}))};
  simulation_node_capture_rigidbody_field(
      geometry_set, simulation::rb_flag_attribute_name, selection_field, new_flag_field);
}

}  // namespace blender::nodes
