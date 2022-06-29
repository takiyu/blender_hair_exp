/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2013 Blender Foundation. All rights reserved. */

/** \file
 * \ingroup bke
 * \brief API for Blender-side Rigid Body stuff
 */

#pragma once

namespace blender::simulation {

static const char *id_attribute_name = "id";
static const char *pos_attribute_name = "position";
static const char *rot_attribute_name = "rotation";
static const char *rb_shape_index_attribute_name = "__rb_shape_index__";
static const char *rb_flag_attribute_name = "__rb_flag__";
static const char *rb_linear_velocity_attribute_name = "__rb_linear_velocity__";
static const char *rb_angular_velocity_attribute_name = "__rb_angular_velocity__";
static const char *rb_collision_group_attribute_name = "__rb_collision_group__";
static const char *rb_collision_mask_attribute_name = "__rb_collision_mask__";

enum eRigidBodyFlag {
  /* The point will simulated with a rigid body. */
  RB_FLAG_ENABLE = (1 << 0),
  /* Rigid body state will be initialized from current point position. */
  RB_FLAG_INITIALIZE = (1 << 1),

  /* Flags to set various properties. */
  RB_FLAG_SET_LINEAR_VELOCITY = (1 << 16),
  RB_FLAG_SET_ANGULAR_VELOCITY = (1 << 17),
};

}  // namespace blender::simulation

struct RigidBodyMap {
  using UID = int;

  enum BodyFlag {
    /* Body is used by data in Blender, used for removing bodies that are no longer needed */
    Used = (1 << 0),
  };

  struct BodyPointer {
    int flag;
    struct rbRigidBody *body;
  };

  blender::Map<UID, BodyPointer> body_map_;
  blender::Map<std::string, struct rbCollisionShape *> shape_map_;

  void clear(struct RigidBodyWorld *rbw);

  void add_shape(blender::StringRef name, struct rbCollisionShape *shape);
  void clear_shapes();
};
