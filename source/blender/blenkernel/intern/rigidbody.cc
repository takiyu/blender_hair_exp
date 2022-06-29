/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2013 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup bke
 * \brief Blender-side interface and methods for dealing with Rigid Body simulations
 */

#include "BLI_generic_virtual_array.hh"
#include "BLI_map.hh"
#include "BLI_math_rotation.h"

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"
#include "DNA_node_types.h"
#include "DNA_object_types.h"
#include "DNA_pointcloud_types.h"
#include "DNA_rigidbody_types.h"
#include "DNA_scene_types.h"

#include "BKE_geometry_set.hh"
#include "BKE_global.h"
#include "BKE_lib_id.h"
#include "BKE_object.h"
#include "BKE_mesh_runtime.h"
#include "BKE_pointcloud.h"
#include "BKE_rigidbody.h"
#include "BKE_rigidbody.hh"

#include "MOD_nodes.h"

#ifdef WITH_BULLET
#  include "RBI_api.h"
#endif

#define RB_DEBUG_PRINT 0

void RigidBodyMap::clear(RigidBodyWorld *rbw)
{
  rbDynamicsWorld *physics_world = (rbDynamicsWorld *)rbw->shared->physics_world;

  for (const RigidBodyMap::BodyPointer &body_ptr : body_map_.values()) {
    RB_dworld_remove_body(physics_world, body_ptr.body);
    RB_body_delete(body_ptr.body);
  }
  body_map_.clear();

  clear_shapes();
}

void RigidBodyMap::add_shape(blender::StringRef name, rbCollisionShape *shape)
{
  shape_map_.add(name, shape);
}

void RigidBodyMap::clear_shapes()
{
  for (rbCollisionShape *shape : shape_map_.values()) {
    RB_shape_delete(shape);
  }
  shape_map_.clear();
}


namespace blender::simulation {

/* create collision shape of mesh - convex hull */
rbCollisionShape *rigidbody_get_shape_convexhull_from_mesh(const Mesh *mesh, float margin, bool &r_can_embed)
{
  if (mesh == nullptr) {
    return nullptr;
  }

  const MVert *mvert = mesh->mvert;
  int totvert = mesh->totvert;
  if (totvert == 0) {
    // CLOG_ERROR(&LOG, "no vertices to define Convex Hull collision shape with");
    return nullptr;
  }

  return RB_shape_new_convex_hull((float *)mvert, sizeof(MVert), totvert, margin, &r_can_embed);
}

/* create collision shape of mesh - triangulated mesh
 * returns NULL if creation fails.
 */
rbCollisionShape *rigidbody_get_shape_trimesh_from_mesh(const Mesh *mesh, bool is_passive)
{
  if (mesh == nullptr) {
    return nullptr;
  }

  const MVert *mvert = mesh->mvert;
  int totvert = mesh->totvert;
  const MLoopTri *looptri = BKE_mesh_runtime_looptri_ensure(mesh);
  int tottri = mesh->runtime.looptris.len;
  const MLoop *mloop = mesh->mloop;
  if ((totvert == 0) || (tottri == 0)) {
    //CLOG_WARN(
    //    &LOG, "no geometry data converted for Mesh Collision Shape (ob = %s)", ob->id.name + 2);
    return nullptr;
  }

  rbCollisionShape *shape = nullptr;
  rbMeshData *mdata;
  int i;

  /* init mesh data for collision shape */
  mdata = RB_trimesh_data_new(tottri, totvert);

  RB_trimesh_add_vertices(mdata, (float *)mvert, totvert, sizeof(MVert));

  /* loop over all faces, adding them as triangles to the collision shape
    * (so for some faces, more than triangle will get added)
    */
  if (mvert && looptri) {
    for (i = 0; i < tottri; i++) {
      /* add first triangle - verts 1,2,3 */
      const MLoopTri *lt = &looptri[i];
      int vtri[3];

      vtri[0] = mloop[lt->tri[0]].v;
      vtri[1] = mloop[lt->tri[1]].v;
      vtri[2] = mloop[lt->tri[2]].v;

      RB_trimesh_add_triangle_indices(mdata, i, UNPACK3(vtri));
    }
  }

  RB_trimesh_finish(mdata);

  /* construct collision shape
    *
    * These have been chosen to get better speed/accuracy tradeoffs with regards
    * to limitations of each:
    *    - BVH-Triangle Mesh: for passive objects only. Despite having greater
    *                         speed/accuracy, they cannot be used for moving objects.
    *    - GImpact Mesh:      for active objects. These are slower and less stable,
    *                         but are more flexible for general usage.
    */
  if (is_passive) {
    shape = RB_shape_new_trimesh(mdata);
  }
  else {
    shape = RB_shape_new_gimpact_mesh(mdata);
  }

  return shape;
}

rbCollisionShape *construct_collision_shape(const SimulationComponent::ShapeConstructionInfo &shape_info)
{
  /* XXX Margin computation in rigidbody.c is crazy */
  const bool use_margin = (shape_info.margin_ > 0.0f);

  rbCollisionShape *new_shape = nullptr;
  /* create new shape */
  switch (shape_info.type_) {
    case COLLISION_SHAPE_BOX:
      new_shape = RB_shape_new_box(shape_info.half_extent_.x, shape_info.half_extent_.y, shape_info.half_extent_.z);
      break;

    case COLLISION_SHAPE_SPHERE:
      new_shape = RB_shape_new_sphere(shape_info.radius_);
      break;

    case COLLISION_SHAPE_CAPSULE: {
      float capsule_height = (shape_info.height_ - shape_info.radius_) * 2.0f;
      new_shape = RB_shape_new_capsule(shape_info.radius_,
                                       (capsule_height > 0.0f) ? capsule_height : 0.0f);
      break;
    }
    case COLLISION_SHAPE_CYLINDER:
      new_shape = RB_shape_new_cylinder(shape_info.radius_, shape_info.height_);
      break;
    case COLLISION_SHAPE_CONE:
      new_shape = RB_shape_new_cone(shape_info.radius_, shape_info.height_ * 2.0f);
      break;

    case COLLISION_SHAPE_CONVEX_HULL: {
      /* try to embed collision margin */
      const bool has_volume =
          (min_fff(shape_info.half_extent_.x, shape_info.half_extent_.y, shape_info.half_extent_.z) > 0.0f);
      const float hull_margin = (!use_margin && has_volume ? 0.04f : 0.0f);

      bool can_embed;
      if (const MeshComponent *component =
              shape_info.mesh_->get_component_for_read<MeshComponent>()) {
        if (const Mesh *mesh = component->get_for_read()) {
          new_shape = rigidbody_get_shape_convexhull_from_mesh(
              mesh, shape_info.margin_, can_embed);
        }
      }
      break;
    }
    case COLLISION_SHAPE_TRIMESH: {
      if (const MeshComponent *component =
              shape_info.mesh_->get_component_for_read<MeshComponent>()) {
        if (const Mesh *mesh = component->get_for_read()) {
          /* XXX Only active bodies from simulation component for now */
          bool is_passive = false;
          new_shape = rigidbody_get_shape_trimesh_from_mesh(mesh, is_passive);
        }
      }
      break;
    }
    case COLLISION_SHAPE_COMPOUND:
      new_shape = RB_shape_new_compound();
      /* TODO */
      //rbCollisionShape *childShape = NULL;
      //float loc[3], rot[4];
      //float mat[4][4];
      ///* Add children to the compound shape */
      //FOREACH_COLLECTION_OBJECT_RECURSIVE_BEGIN (rbw->group, childObject) {
      //  if (childObject->parent == ob) {
      //    childShape = rigidbody_validate_sim_shape_helper(rbw, childObject);
      //    if (childShape) {
      //      BKE_object_matrix_local_get(childObject, mat);
      //      mat4_to_loc_quat(loc, rot, mat);
      //      RB_compound_add_child_shape(new_shape, childShape, loc, rot);
      //    }
      //  }
      //}
      //FOREACH_COLLECTION_OBJECT_RECURSIVE_END;
      break;
  }
  /* use box shape if it failed to create new shape */
  if (new_shape == NULL) {
    new_shape = RB_shape_new_box(shape_info.half_extent_.x, shape_info.half_extent_.y, shape_info.half_extent_.z);
  }
  if (new_shape) {
    RB_shape_set_margin(new_shape, shape_info.margin_);
  }

  return new_shape;
}

struct ComponentTransformReadData {
  VArray<float3> loc_data;
  VArray<float3> rot_data;
  VArray<float4x4> transform_data;

  ComponentTransformReadData(const GeometryComponent &component)
  {
    switch (component.type()) {
      case GeometryComponentType::GEO_COMPONENT_TYPE_MESH:
      case GeometryComponentType::GEO_COMPONENT_TYPE_POINT_CLOUD: {
        bke::ReadAttributeLookup pos_attribute = component.attribute_try_get_for_read(
            pos_attribute_name, CD_PROP_FLOAT3);
        bke::ReadAttributeLookup rot_attribute = component.attribute_try_get_for_read(
            rot_attribute_name, CD_PROP_FLOAT3);
        loc_data = pos_attribute.varray.typed<float3>();
        rot_data = rot_attribute.varray.typed<float3>();
      } break;
      case GeometryComponentType::GEO_COMPONENT_TYPE_INSTANCES: {
        const InstancesComponent &instances_component = static_cast<const InstancesComponent &>(
            component);
        transform_data = VArray<float4x4>::ForSpan(instances_component.instance_transforms());
      } break;
      default:
        break;
    }
  }
};

struct ComponentTransformWriteData {
  VMutableArray<float3> loc_data;
  VMutableArray<float3> rot_data;
  VMutableArray<float4x4> transform_data;

  ComponentTransformWriteData(GeometryComponent &component)
  {
    switch (component.type()) {
      case GeometryComponentType::GEO_COMPONENT_TYPE_MESH:
      case GeometryComponentType::GEO_COMPONENT_TYPE_POINT_CLOUD: {
        bke::WriteAttributeLookup pos_attribute = component.attribute_try_get_for_write(
            pos_attribute_name);
        bke::WriteAttributeLookup rot_attribute = component.attribute_try_get_for_write(
            rot_attribute_name);
        loc_data = pos_attribute.varray.typed<float3>();
        rot_data = rot_attribute.varray.typed<float3>();
      } break;
      case GeometryComponentType::GEO_COMPONENT_TYPE_INSTANCES: {
        InstancesComponent &instances_component = static_cast<InstancesComponent &>(component);
        transform_data = VMutableArray<float4x4>::ForSpan(instances_component.instance_transforms());
      } break;
      default:
        break;
    }
  }
};

static void update_simulation_nodes_component(
    struct RigidBodyWorld *rbw,
    Object *object,
    NodesModifierData *nmd,
    GeometryComponent &component,
    Span<SimulationComponent::ShapeConstructionInfo> new_shape_ctors,
    MutableSpan<rbCollisionShape *> new_shapes,
    int &num_used_bodies,
    bool &r_rebuild_islands)
{
  static rbCollisionShape *generic_shape = RB_shape_new_sphere(0.05f);
  static int generic_collision_groups = 0xFFFFFFFF;

  rbDynamicsWorld *physics_world = (rbDynamicsWorld *)rbw->shared->physics_world;
  Object *orig_ob = (Object *)object->id.orig_id;
  BLI_assert(orig_ob->runtime.rigid_body_map);
  RigidBodyMap &rb_map = *orig_ob->runtime.rigid_body_map;

  bke::ReadAttributeLookup id_attribute =
      component.attribute_try_get_for_read(id_attribute_name, CD_PROP_INT32);
  bke::ReadAttributeLookup shape_index_attribute = component.attribute_try_get_for_read(
      rb_shape_index_attribute_name, CD_PROP_INT32);
  bke::WriteAttributeLookup flag_attribute = component.attribute_try_get_for_write(
      rb_flag_attribute_name);
  bke::ReadAttributeLookup linear_velocity_attribute = component.attribute_try_get_for_read(
      rb_linear_velocity_attribute_name, CD_PROP_FLOAT3);
  bke::ReadAttributeLookup angular_velocity_attribute = component.attribute_try_get_for_read(
      rb_angular_velocity_attribute_name, CD_PROP_FLOAT3);
  bke::ReadAttributeLookup collision_group_attribute = component.attribute_try_get_for_read(
      rb_collision_group_attribute_name, CD_PROP_INT32);
  bke::ReadAttributeLookup collision_mask_attribute = component.attribute_try_get_for_read(
      rb_collision_mask_attribute_name, CD_PROP_INT32);
  ComponentTransformReadData transform_data(component);

  #if RB_DEBUG_PRINT
  std::cout << "Update bodies:" << std::endl;
  #endif
  if (id_attribute && flag_attribute) {
    VArray<int> id_data = id_attribute.varray.typed<int>();
    BLI_assert(id_data.size() == component.attribute_domain_num(id_attribute.domain));
    VMutableArray<int> flag_data = flag_attribute.varray.typed<int>();
    BLI_assert(flag_data.size() == component.attribute_domain_num(flag_attribute.domain));
    VArray<int> shape_index_data = shape_index_attribute.varray.typed<int>();
    VArray<float3> linear_velocity_data = linear_velocity_attribute.varray.typed<float3>();
    VArray<float3> angular_velocity_data = angular_velocity_attribute.varray.typed<float3>();
    VArray<int> collision_group_data = collision_group_attribute.varray.typed<int>();
    VArray<int> collision_mask_data = collision_mask_attribute.varray.typed<int>();

    auto lazy_init_shape = [shape_index_data, new_shape_ctors, &rb_map, &new_shapes](
                               int body_index) -> rbCollisionShape * {
      if (shape_index_data) {
        int shape_index = shape_index_data[body_index];
        if (shape_index >= 0 && shape_index < new_shapes.size()) {
          rbCollisionShape *shape = new_shapes[shape_index];
          if (shape == nullptr) {
            const SimulationComponent::ShapeConstructionInfo &shape_info =
                new_shape_ctors[shape_index];
            shape = rb_map.shape_map_.lookup_or_add_cb(shape_info.name_, [shape_info]() {
              return construct_collision_shape(shape_info);
            });
            BLI_assert(shape != nullptr);
            new_shapes[shape_index] = shape;
          }
          return shape;
        }
      }
      return generic_shape;
    };

    for (int i : id_data.index_range()) {
      int flag = flag_data[i];
      if (!(flag & RB_FLAG_ENABLE)) {
        continue;
      }
      ++num_used_bodies;

      int uid = id_data[i];
      RigidBodyMap::BodyPointer &body_ptr = rb_map.body_map_.lookup_or_add(
          uid, RigidBodyMap::BodyPointer{RigidBodyMap::Used, nullptr});

      /* Remove existing body when initializing, to ensure a clean slate for motion state and
       * other properties. */
      //int old_world_index = -1;
      if (flag & RB_FLAG_INITIALIZE) {
        if (rbRigidBody *body = body_ptr.body) {
          /* XXX Optimize, devirtualize, etc. etc. */
          const float3 pos = transform_data.loc_data ?
                                 transform_data.loc_data[i] :
                                 (transform_data.transform_data ?
                                      transform_data.transform_data[i].translation() :
                                      float3(0, 0, 0));
          const float3 rot_eul = transform_data.rot_data ?
                                     transform_data.rot_data[i] :
                                     (transform_data.transform_data ?
                                          transform_data.transform_data[i].to_euler() :
                                          float3(0, 0, 0));
          float rot_qt[4];
          eul_to_quat(rot_qt, rot_eul);

          int col_groups, filter_group, filter_mask;
          RB_body_get_collision_group_ex(body, &col_groups, &filter_group, &filter_mask);
          if (collision_group_data) {
            filter_group = collision_group_data[i];
          }
          if (collision_mask_data) {
            filter_mask = collision_mask_data[i];
          }

#if RB_DEBUG_PRINT
           old_world_index = RB_debug_get_world_index(physics_world,
           body_ptr.body);
          std::cout << "  " << i << ". remove " << body_ptr.body << " world index "
                    << RB_debug_get_world_index(physics_world, body_ptr.body) << std::endl;
#endif
          RB_dworld_remove_body(physics_world, body);
          RB_body_set_collision_shape(body, lazy_init_shape(i));
          //RB_body_delete(body_ptr.body);
          //body_ptr.body = nullptr;
          RB_body_clear_forces(physics_world, body);
          RB_body_set_linear_velocity(body, float3(0, 0, 0));
          RB_body_set_angular_velocity(body, float3(0, 0, 0));
          RB_body_reset_loc_rot(physics_world, body, pos, rot_qt);
          RB_dworld_add_body_ex(physics_world, body, col_groups, filter_group, filter_mask);
          r_rebuild_islands = true;
        }
      }

      /* Add new body */
      if (body_ptr.body == nullptr) {
        /* XXX Optimize, devirtualize, etc. etc. */
        const float3 pos = transform_data.loc_data ?
                               transform_data.loc_data[i] :
                               (transform_data.transform_data ?
                                    transform_data.transform_data[i].translation() :
                                    float3(0, 0, 0));
        const float3 rot_eul = transform_data.rot_data ?
                                   transform_data.rot_data[i] :
                                   (transform_data.transform_data ?
                                        transform_data.transform_data[i].to_euler() :
                                        float3(0, 0, 0));
        float rot_qt[4];
        eul_to_quat(rot_qt, rot_eul);

        int filter_group, filter_mask;
        if (collision_group_data) {
          filter_group = collision_group_data[i];
        }
        if (collision_mask_data) {
          filter_mask = collision_mask_data[i];
        }

        rbRigidBody *body = RB_body_new(lazy_init_shape(i), pos, rot_qt);
        /* This also computes local moment of inertia, which is needed for rotations! */
        RB_body_set_friction(body, 0.5f);
        RB_body_set_restitution(body, 0.05f);
        RB_body_set_mass(body, 1.0f);
        RB_dworld_add_body_ex(physics_world, body, generic_collision_groups, filter_group, filter_mask);
        body_ptr.body = body;
        r_rebuild_islands = true;
#if RB_DEBUG_PRINT
         std::cout << "  " << i << ". added " << body << " world index "
                  << RB_debug_get_world_index(physics_world, body) << std::endl;
        int new_world_index = RB_debug_get_world_index(physics_world, body);
        BLI_assert(new_world_index >= old_world_index);
#endif
      }

      if (linear_velocity_data && (flag & RB_FLAG_SET_LINEAR_VELOCITY)) {
        RB_body_set_linear_velocity(body_ptr.body, linear_velocity_data[i]);
      }
      if (angular_velocity_data && (flag & RB_FLAG_SET_ANGULAR_VELOCITY)) {
        RB_body_set_angular_velocity(body_ptr.body, angular_velocity_data[i]);
      }

#if RB_DEBUG_PRINT
      if (body_ptr.body) {
        float qt[4];
        RB_body_get_orientation(body_ptr.body, qt);
        float3 rot;
        quat_to_eul(rot, qt);
        float3 avel;
        RB_body_get_angular_velocity(body_ptr.body, avel);
        std::cout << "Body start rotation=" << rot;
        std::cout << "ang.vel.=" << avel;
        std::cout << std::endl;
      }
#endif

      /* Flag existing bodies as used */
      body_ptr.flag |= RigidBodyMap::Used;
      /* Initialization finished */
      flag &= ~(RB_FLAG_INITIALIZE | RB_FLAG_SET_LINEAR_VELOCITY | RB_FLAG_SET_ANGULAR_VELOCITY);
      flag_data.set(i, flag);
    }

    if (flag_attribute.tag_modified_fn) {
      flag_attribute.tag_modified_fn();
    }
  }

  /* Clear transient attributes after applying them */
  component.attribute_try_delete(rb_linear_velocity_attribute_name);
  component.attribute_try_delete(rb_angular_velocity_attribute_name);
}

}  // namespace blender::simulation

void BKE_rigidbody_update_simulation_nodes(RigidBodyWorld *rbw,
                                           Object *object,
                                           NodesModifierData *nmd)
{
  using namespace blender;

  rbDynamicsWorld *physics_world = (rbDynamicsWorld *)rbw->shared->physics_world;

  BKE_object_runtime_ensure_rigid_body_map(
      rbw, object, MOD_nodes_needs_rigid_body_sim(object, nmd));
  Object *orig_ob = (Object *)object->id.orig_id;
  RigidBodyMap &rb_map = *orig_ob->runtime.rigid_body_map;

  GeometrySet *geometry_set = object->runtime.geometry_set_eval;

  /* Update flags for used bodies */
  const int num_existing_bodies = rb_map.body_map_.size();
  for (RigidBodyMap::BodyPointer &body_ptr : rb_map.body_map_.values()) {
    body_ptr.flag &= ~RigidBodyMap::BodyFlag::Used;
  }

  int num_used_bodies = 0;
  bool rebuild_islands = false;
  if (geometry_set) {
    /* Lazy init for new shapes: reserve space in the shape list, but only construct if actually used. */
    Span<SimulationComponent::ShapeConstructionInfo> new_shape_ctors;
    if (const SimulationComponent *sim_component =
            geometry_set->get_component_for_read<SimulationComponent>()) {
      new_shape_ctors = sim_component->shapes();
      rb_map.shape_map_.reserve(rb_map.shape_map_.size() + new_shape_ctors.size());
    }
    Vector<rbCollisionShape *> new_shapes(new_shape_ctors.size(), nullptr);

    if (geometry_set->has_mesh()) {
      simulation::update_simulation_nodes_component(
          rbw,
          object,
          nmd,
          geometry_set->get_component_for_write<MeshComponent>(),
          new_shape_ctors,
          new_shapes,
          num_used_bodies,
          rebuild_islands);
    }
    if (geometry_set->has_pointcloud()) {
      simulation::update_simulation_nodes_component(
          rbw,
          object,
          nmd,
          geometry_set->get_component_for_write<PointCloudComponent>(),
          new_shape_ctors,
          new_shapes,
          num_used_bodies,
          rebuild_islands);
    }
    if (geometry_set->has_instances()) {
      simulation::update_simulation_nodes_component(
          rbw,
          object,
          nmd,
          geometry_set->get_component_for_write<InstancesComponent>(),
          new_shape_ctors,
          new_shapes,
          num_used_bodies,
          rebuild_islands);
    }
  }

  /* Remove unused bodies */
  Vector<RigidBodyMap::UID> bodies_to_remove;
  bodies_to_remove.reserve(num_existing_bodies - num_used_bodies);
  for (const auto &item : rb_map.body_map_.items()) {
    if (!(item.value.flag & RigidBodyMap::BodyFlag::Used)) {
      RB_dworld_remove_body(physics_world, item.value.body);
      RB_body_delete(item.value.body);
      bodies_to_remove.append(item.key);
      rebuild_islands = true;
    }
  }
  for (RigidBodyMap::UID uid : bodies_to_remove) {
    rb_map.body_map_.remove(uid);
  }

  if (rebuild_islands) {
    RB_dworld_rebuild_islands(physics_world);
  }
}

namespace blender::simulation {

static void update_simulation_nodes_component_post_step(RigidBodyWorld *rbw,
                                                        Object *object,
                                                        NodesModifierData *nmd,
                                                        GeometryComponent *component)
{
  if (component == nullptr) {
    return;
  }

  Object *orig_ob = (Object *)object->id.orig_id;
  if (orig_ob->runtime.rigid_body_map == nullptr) {
    return;
  }
  const RigidBodyMap &rb_map = *orig_ob->runtime.rigid_body_map;

  bke::ReadAttributeLookup id_attribute = component->attribute_try_get_for_read(id_attribute_name,
                                                                                CD_PROP_INT32);
  ComponentTransformWriteData transform_data(*component);

  if (id_attribute) {
    const int num_points = component->attribute_domain_num(id_attribute.domain);

    VArray<int> id_data = id_attribute.varray.typed<int>();
    BLI_assert(id_data.size() == num_points);

    for (int i : id_data.index_range()) {
      int uid = id_data[i];

      const RigidBodyMap::BodyPointer *body_ptr = rb_map.body_map_.lookup_ptr(uid);
      if (body_ptr) {
        float3 loc;
        RB_body_get_position(body_ptr->body, loc);
        float4 rot_qt;
        RB_body_get_orientation(body_ptr->body, rot_qt);
        float3 rot_eul;
        quat_to_eul(rot_eul, rot_qt);
#if RB_DEBUG_PRINT
        {
          std::cout << "Body end rotation: " << rot_eul << std::endl;
        }
#endif

        if (transform_data.transform_data)
        {
          float3 scale = transform_data.transform_data[i].scale();
          transform_data.transform_data.set(i, float4x4::from_loc_eul_scale(loc, rot_eul, scale));
        }
        else {
          if (transform_data.loc_data) {
            transform_data.loc_data.set(i, loc);
          }
          if (transform_data.rot_data) {
            transform_data.rot_data.set(i, rot_eul);
          }
        }
      }
    }
  }
}

}  // namespace blender::simulation

void BKE_rigidbody_update_simulation_nodes_post_step(RigidBodyWorld *rbw,
                                                     Object *object,
                                                     NodesModifierData *nmd)
{
  using namespace blender;

  if (GeometrySet *geometry_set = object->runtime.geometry_set_eval) {
    simulation::update_simulation_nodes_component_post_step(
        rbw, object, nmd, &geometry_set->get_component_for_write<MeshComponent>());
    simulation::update_simulation_nodes_component_post_step(
        rbw, object, nmd, &geometry_set->get_component_for_write<PointCloudComponent>());
    simulation::update_simulation_nodes_component_post_step(
        rbw, object, nmd, &geometry_set->get_component_for_write<InstancesComponent>());
  }
}
