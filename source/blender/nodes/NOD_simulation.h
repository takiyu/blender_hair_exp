/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BKE_node.h"

#ifdef __cplusplus
extern "C" {
#endif

extern struct bNodeTreeType *ntreeType_Particles;

void register_node_tree_type_simulation(void);

void register_node_type_simulation_add_collision_shapes(void);
void register_node_type_simulation_add_rigid_bodies(void);
void register_node_type_simulation_add_rigid_body_impulse(void);
void register_node_type_simulation_apply_rigid_body_force(void);
void register_node_type_simulation_apply_rigid_body_torque(void);
void register_node_type_simulation_remove_rigid_bodies(void);
void register_node_type_simulation_rigid_body_mass(void);
void register_node_type_simulation_rigid_body_velocity(void);
void register_node_type_simulation_set_rigid_body_collision_response(void);
void register_node_type_simulation_set_rigid_body_dynamics(void);
void register_node_type_simulation_set_rigid_body_effector_weights(void);
void register_node_type_simulation_set_rigid_body_shape(void);
void register_node_type_simulation_set_rigid_body_velocity(void);
void register_node_type_simulation_set_rigid_body_angular_velocity(void);

#ifdef __cplusplus
}
#endif
