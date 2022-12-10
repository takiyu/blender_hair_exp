/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "NOD_socket_search_link.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::math {

/* XXX These are placeholder functions based on BLI math until SIMD versions are available */

inline float4x4 [[nodiscard]] add(const float4x4 &a, const float4x4 &b)
{
  float4x4 r;
  add_m4_m4m4(r.ptr(), a.ptr(), b.ptr());
  return r;
}

inline float4x4 [[nodiscard]] subtract(const float4x4 &a, const float4x4 &b)
{
  float4x4 r;
  sub_m4_m4m4(r.ptr(), a.ptr(), b.ptr());
  return r;
}

}

namespace blender::nodes::node_fn_matrix4x4_math_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix4x4>(N_("Matrix"));
  b.add_input<decl::Matrix4x4>(N_("Matrix"), "Matrix_001");
  b.add_input<decl::Float>(N_("Scale")).default_value(1.0f);
  b.add_output<decl::Matrix4x4>(N_("Matrix"));
  b.add_output<decl::Float>(N_("Value"));
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  uiItemR(layout, ptr, "operation", UI_ITEM_R_SPLIT_EMPTY_NAME, "", ICON_NONE);
}

static void node_update(bNodeTree *tree, bNode *node)
{
  const NodeMatrixMathOperation op = (NodeMatrixMathOperation)node->custom1;

  bNodeSocket *inMatrixA = &node->input_socket(0);
  bNodeSocket *inMatrixB = &node->input_socket(1);
  bNodeSocket *inScale = &node->input_socket(2);
  bNodeSocket *outMatrix = &node->output_socket(3);
  bNodeSocket *outValue = &node->output_socket(4);

  nodeSetSocketAvailability(tree, inMatrixA, true);
  nodeSetSocketAvailability(
      tree,
      inMatrixB,
      ELEM(op, NODE_MATRIX_MATH_ADD, NODE_MATRIX_MATH_SUBTRACT, NODE_MATRIX_MATH_MULTIPLY));
  nodeSetSocketAvailability(tree, inScale, ELEM(op, NODE_MATRIX_MATH_SCALAR_MULTIPLY));

  nodeSetSocketAvailability(tree,
                            outMatrix,
                            ELEM(op,
                                 NODE_MATRIX_MATH_ADD,
                                 NODE_MATRIX_MATH_SUBTRACT,
                                 NODE_MATRIX_MATH_SCALAR_MULTIPLY,
                                 NODE_MATRIX_MATH_MULTIPLY,
                                 NODE_MATRIX_MATH_TRANSPOSE,
                                 NODE_MATRIX_MATH_INVERSE));
  nodeSetSocketAvailability(
      tree, outValue, ELEM(op, NODE_MATRIX_MATH_DETERMINANT, NODE_MATRIX_MATH_TRACE));

  /* Labels */
  node_sock_label_clear(inMatrixA);
  node_sock_label_clear(inMatrixB);
  node_sock_label_clear(inScale);
  switch (op) {
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = NODE_MATRIX_MATH_ADD;
}

static const fn::MultiFunction *get_multi_function(NodeMatrixMathOperation op)
{
  static auto exec_preset_fast = fn::CustomMF_presets::AllSpanOrSingle();
  static auto exec_preset_slow = fn::CustomMF_presets::Materialized();

  const fn::MultiFunction *multi_fn = nullptr;

  switch (op) {
    case NODE_MATRIX_MATH_ADD: {
      static fn::CustomMF_SI_SI_SO<float4x4, float4x4, float4x4> fn{
          "add",
          [](const float4x4 &a, const float4x4 &b) -> float4x4 { return math::add(a, b); },
          exec_preset_fast};
      return &fn;
    }
  }

  return nullptr;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeMatrixMathOperation op = (NodeMatrixMathOperation)builder.node().custom1;
  const fn::MultiFunction *fn = get_multi_function(op);
  builder.set_matching_fn(fn);
}

}  // namespace blender::nodes::node_fn_matrix4x4_math_cc

void register_node_type_fn_matrix_4x4_math(void)
{
  namespace file_ns = blender::nodes::node_fn_matrix4x4_math_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_MATRIX_4X4_MATH, "4x4 Matrix Math", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.updatefunc = file_ns::node_update;
  ntype.initfunc = file_ns::node_init;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;

  nodeRegisterType(&ntype);
}
