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

/**
 * Generate shapes for use in rigid body simulation.
 *
 * Inputs:
 *  Name: Unique identifier, helps re-using shapes from earlier iterations.
 *        XXX would be great if we can generate this automatically, based on the node?
 *  Count: Number of shapes when generating primitive types (box, sphere, capsule, cylinder, cone).
 *  Source: Mesh or instances when generating complex types (convex hull, triangle mesh, compound).
 */

namespace blender::nodes::node_simulation_add_collision_shapes_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_input<decl::Geometry>(N_("Geometry"))
      .supported_type(
          {GEO_COMPONENT_TYPE_MESH, GEO_COMPONENT_TYPE_POINT_CLOUD, GEO_COMPONENT_TYPE_INSTANCES});
  b.add_input<decl::String>(N_("Name")).default_value("Shape");
  /* Parameters for primitive shapes */
  b.add_input<decl::Int>(N_("Count")).default_value(1);
  b.add_input<decl::Vector>(N_("Half Extent")).default_value(float3(1.0f, 1.0f, 1.0f)).supports_field();
  b.add_input<decl::Float>(N_("Radius")).default_value(1.0f).supports_field();
  b.add_input<decl::Float>(N_("Height")).default_value(1.0f).supports_field();
  /* Geometry input for mesh-type shapes */
  b.add_input<decl::Geometry>(N_("Source"))
      .supported_type(
          {GEO_COMPONENT_TYPE_MESH, GEO_COMPONENT_TYPE_POINT_CLOUD, GEO_COMPONENT_TYPE_INSTANCES});
  b.add_input<decl::Float>(N_("Margin")).default_value(1.0f).supports_field();

  b.add_output<decl::Geometry>(N_("Geometry"));
}

static void node_layout(uiLayout *layout, bContext *UNUSED(C), PointerRNA *ptr)
{
  uiItemR(layout, ptr, "shape_type", 0, "", ICON_NONE);
}

static void node_init(bNodeTree *UNUSED(tree), bNode *node)
{
  node->custom1 = COLLISION_SHAPE_SPHERE;
}

static bool use_source_geometry(eCollisionShapeType shape_type)
{
  return ELEM(shape_type,
              eCollisionShapeType::COLLISION_SHAPE_CONVEX_HULL,
              eCollisionShapeType::COLLISION_SHAPE_TRIMESH,
              eCollisionShapeType::COLLISION_SHAPE_COMPOUND);
}

static bool use_half_extent(eCollisionShapeType shape_type)
{
  return ELEM(shape_type, eCollisionShapeType::COLLISION_SHAPE_BOX);
}

static bool use_radius(eCollisionShapeType shape_type)
{
  return ELEM(shape_type,
              eCollisionShapeType::COLLISION_SHAPE_SPHERE,
              eCollisionShapeType::COLLISION_SHAPE_CAPSULE,
              eCollisionShapeType::COLLISION_SHAPE_CYLINDER,
              eCollisionShapeType::COLLISION_SHAPE_CONE);
}

static bool use_height(eCollisionShapeType shape_type)
{
  return ELEM(shape_type,
              eCollisionShapeType::COLLISION_SHAPE_CAPSULE,
              eCollisionShapeType::COLLISION_SHAPE_CYLINDER,
              eCollisionShapeType::COLLISION_SHAPE_CONE);
}

static void node_update(bNodeTree *ntree, bNode *node)
{
  eCollisionShapeType shape_type = static_cast<eCollisionShapeType>(node->custom1);

  bNodeSocket *socket_geometry = (bNodeSocket *)node->inputs.first;
  bNodeSocket *socket_name = socket_geometry->next;
  bNodeSocket *socket_count = socket_name->next;
  bNodeSocket *socket_half_extent = socket_count->next;
  bNodeSocket *socket_radius = socket_half_extent->next;
  bNodeSocket *socket_height = socket_radius->next;
  bNodeSocket *socket_source = socket_height->next;
  bNodeSocket *socket_margin = socket_source->next;

  nodeSetSocketAvailability(ntree, socket_count, !use_source_geometry(shape_type));
  nodeSetSocketAvailability(ntree, socket_half_extent, use_half_extent(shape_type));
  nodeSetSocketAvailability(ntree, socket_radius, use_radius(shape_type));
  nodeSetSocketAvailability(ntree, socket_height, use_height(shape_type));
  nodeSetSocketAvailability(ntree, socket_source, use_source_geometry(shape_type));
  nodeSetSocketAvailability(ntree, socket_margin, use_source_geometry(shape_type));
}

static void node_exec(GeoNodeExecParams params)
{
  eCollisionShapeType shape_type = static_cast<eCollisionShapeType>(params.node().custom1);

  GeometrySet geometry_set = params.extract_input<GeometrySet>("Geometry");
  std::string name = params.extract_input<std::string>("Name");
  SimulationComponent &sim_component = geometry_set.get_component_for_write<SimulationComponent>();

  FieldContext context;

  if (use_source_geometry(shape_type)) {
    /* Geometry input for mesh-type shapes */
    GeometrySet src_geometry = params.extract_input<GeometrySet>("Source");
    Field<float> margin_field = params.get_input<Field<float>>("Margin");

    const InstancesComponent *instances =
        src_geometry.get_component_for_read<InstancesComponent>();
    if (instances == 0) {
      const IndexMask mask(1);
      FieldEvaluator evaluator{context, &mask};

      Array<float> margin(1);
      evaluator.add_with_destination(margin_field, margin.as_mutable_span());
      evaluator.evaluate();

      sim_component.add_shape(SimulationComponent::ShapeConstructionInfo{
          name, shape_type, float3(0), 0, 0, margin[0], src_geometry});
    }
    else {
      const IndexMask mask(instances->instances_num());
      FieldEvaluator evaluator{context, &mask};

      Array<float> margin(instances->instances_num());
      evaluator.add_with_destination(margin_field, margin.as_mutable_span());
      evaluator.evaluate();

      int index = 0;
      instances->foreach_referenced_geometry(
          [&](const GeometrySet &referenced_geometry) {
            sim_component.add_shape(SimulationComponent::ShapeConstructionInfo{
                name, shape_type, float3(0), 0, 0, margin[index], referenced_geometry});
            ++index;
          });
    }
  }
  else {
    const int count = params.extract_input<int>("Count");
    const IndexMask mask(count);
    FieldEvaluator evaluator{context, &mask};

    Array<float3> half_extent(count);
    Array<float> radius(count);
    Array<float> height(count);
    if (use_half_extent(shape_type)) {
      Field<float3> half_extent_field = params.get_input<Field<float3>>("Half Extent");
      evaluator.add_with_destination(half_extent_field, half_extent.as_mutable_span());
    }
    if (use_radius(shape_type)) {
      Field<float> radius_field = params.get_input<Field<float>>("Radius");
      evaluator.add_with_destination(radius_field, radius.as_mutable_span());
    }
    if (use_height(shape_type)) {
      Field<float> height_field = params.get_input<Field<float>>("Height");
      evaluator.add_with_destination(height_field, height.as_mutable_span());
    }
    evaluator.evaluate();

    for (int i : IndexRange(count)) {
      sim_component.add_shape(SimulationComponent::ShapeConstructionInfo{
          name, shape_type, half_extent[i], radius[i], height[i], 0.0f, GeometrySet()});
    }
  }

  params.set_output("Geometry", std::move(geometry_set));
}

}  // namespace blender::nodes::node_simulation_add_collision_shapes_cc

void register_node_type_simulation_add_collision_shapes()
{
  namespace file_ns = blender::nodes::node_simulation_add_collision_shapes_cc;

  static bNodeType ntype;

  simulation_node_type_base(&ntype, SIMULATION_NODE_ADD_COLLISION_SHAPES, "Add Collision Shapes", NODE_CLASS_GEOMETRY);
  node_type_init(&ntype, file_ns::node_init);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_exec;
  ntype.draw_buttons = file_ns::node_layout;
  ntype.updatefunc = file_ns::node_update;
  nodeRegisterType(&ntype);
}
