/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BLI_utildefines.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
namespace blender::bke {
class SimplexGeometryRuntime;
}  // namespace blender::bke
using SimplexGeometryRuntimeHandle = blender::bke::SimplexGeometryRuntime;
#else
typedef struct SimplexGeometryRuntimeHandle SimplexGeometryRuntimeHandle;
#endif

/**
 * Tetrahedral simplex consisting of 4 vertices.
 */
typedef struct Simplex {
  /* Point indices making up the tetrahedron.
   * The vertices form 4 triangles: ABC, BAD, CBD, DAC.
   */
  unsigned int v[4];
} Simplex;

/**
 * A reusable data structure for geometry consisting of simplices (tetrahedra).
 *
 * The data structure is meant to be embedded in other data-blocks to allow reusing
 * tetrahedralization algorithms.
 */
typedef struct SimplexGeometry {
  /**
   * List of simplices in this geometry.
   */
  Simplex *simplices;

  /**
   * The total number of simplices.
   */
  int simplex_num;

  int _pad1;

  /**
   * Runtime data for curves, stored as a pointer to allow defining this as a C++ class.
   */
  SimplexGeometryRuntimeHandle *runtime;
} SimplexGeometry;

#ifdef __cplusplus
}
#endif
