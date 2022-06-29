/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include <string.h>

#include "BLI_math_vec_types.hh"
#include "BLI_utildefines.h"

#include "MEM_guardedalloc.h"

#include "DNA_node_types.h"

#include "BKE_node.h"

#include "BLT_translation.h"

#include "NOD_simulation.h"
#include "NOD_socket_declarations.hh"
#include "NOD_socket_declarations_geometry.hh"

#include "node_util.h"

void simulation_node_type_base(struct bNodeType *ntype, int type, const char *name, short nclass);
bool simulation_node_poll_default(struct bNodeType *ntype,
                                struct bNodeTree *ntree,
                                const char **r_disabled_hint);

namespace blender::nodes {

void simulation_node_capture_rigidbody_field(GeometrySet &geometry_set,
                                             const StringRef name,
                                             const fn::Field<bool> &selection_field,
                                             const GField &field);

void simulation_node_set_rigid_body_flags(GeometrySet &geometry_set,
                                          const fn::Field<bool> &selection_field,
                                          int flags);
void simulation_node_clear_rigid_body_flags(GeometrySet &geometry_set,
                                            const fn::Field<bool> &selection_field,
                                            int flags);

}  // namespace blender::nodes
