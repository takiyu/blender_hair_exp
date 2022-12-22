/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BKE_node_runtime.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_rotate_matrix3x3_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix3x3>(N_("Matrix"));
  b.add_input<decl::Vector>(N_("Rotation"));
  b.add_output<decl::Matrix3x3>(N_("Matrix"));
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static fn::CustomMF_SI_SI_SO<float3x3, float3, float3x3> rotate_matrix_fn{
      "rotate_matrix", [](const float3x3 &mat, const float3 &rot) {
        float3x3 rot_mat;
        eulO_to_mat3(rot_mat.values, rot, EULER_ORDER_DEFAULT);
        return rot_mat * mat;
      }};
  builder.set_matching_fn(&rotate_matrix_fn);
}

}  // namespace blender::nodes::node_fn_rotate_matrix3x3_cc

void register_node_type_fn_rotate_matrix_3x3(void)
{
  namespace file_ns = blender::nodes::node_fn_rotate_matrix3x3_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_ROTATE_MATRIX_3X3, "Rotate 3x3 Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;

  nodeRegisterType(&ntype);
}
