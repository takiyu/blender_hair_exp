/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup DNA
 */

#pragma once

#include "BLI_utildefines.h"

#include "DNA_customdata_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * A reusable data structure for geometry consisting of simplices (tetrahedra).
 *
 * The data structure is meant to be embedded in other data-blocks to allow reusing
 * tetrahedralization algorithms.
 */
typedef struct SimplexGeometry {
  /**
   * Vertex indices of simplices.
   *
   * \note This is *not* stored in #CustomData because the int4 type is currently unsupported.
   */
  int (*simplex_verts)[4];

  /**
   * All attributes stored on control points (#ATTR_DOMAIN_POINT).
   * This might not contain a layer for positions if there are no points.
   */
  CustomData point_data;
  /**
   * All attributes stored on simplices (#ATTR_DOMAIN_SIMPLEX).
   */
  CustomData simplex_data;

  /**
   * The number of points.
   */
  int point_num;
  /**
   * The number of simplices.
   */
  int simplex_num;
} SimplexGeometry;

#ifdef __cplusplus
}
#endif
