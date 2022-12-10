/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BKE_node_runtime.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_input_matrix4x4_cc {

NODE_STORAGE_FUNCS(NodeInputMatrix);

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Matrix4x4>(N_("Matrix"));
};

static void node_layout(uiLayout *layout, bContext * /*C*/, PointerRNA *ptr)
{
  const bNode *node = (const bNode *)ptr->data;
  const NodeCombSepMatrixMode mode = (NodeCombSepMatrixMode)node->custom1;

  uiItemR(layout, ptr, "mode", 0, "", ICON_NONE);

  switch (mode) {
    case NODE_COMBSEP_MATRIX_COLUMNS:
      uiItemR(layout, ptr, "vec0", 0, "Column 0", ICON_NONE);
      uiItemR(layout, ptr, "vec1", 0, "Column 1", ICON_NONE);
      uiItemR(layout, ptr, "vec2", 0, "Column 2", ICON_NONE);
      uiItemR(layout, ptr, "vec3", 0, "Column 3", ICON_NONE);
      break;
    case NODE_COMBSEP_MATRIX_ROWS:
      uiItemR(layout, ptr, "vec0", 0, "Row 0", ICON_NONE);
      uiItemR(layout, ptr, "vec1", 0, "Row 1", ICON_NONE);
      uiItemR(layout, ptr, "vec2", 0, "Row 2", ICON_NONE);
      uiItemR(layout, ptr, "vec3", 0, "Row 3", ICON_NONE);
      break;
    case NODE_COMBSEP_MATRIX_ELEMENTS:
      uiItemR(layout, ptr, "elements", 0, "", ICON_NONE);
      break;
  }
}

static void node_init(bNodeTree * /*tree*/, bNode *node)
{
  node->custom1 = NODE_COMBSEP_MATRIX_COLUMNS;

  NodeInputMatrix *data = MEM_new<NodeInputMatrix>(__func__);
  copy_v3_v3(data->vec0, float3(1.0f, 0.0f, 0.0f));
  copy_v3_v3(data->vec1, float3(0.0f, 1.0f, 0.0f));
  copy_v3_v3(data->vec2, float3(0.0f, 0.0f, 1.0f));
  copy_v3_v3(data->vec3, float3(0.0f, 0.0f, 0.0f));
  unit_m4(data->elements);
  node->storage = data;
}

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  const NodeInputMatrix &storage = node_storage(builder.node());
  const NodeCombSepMatrixMode mode = (NodeCombSepMatrixMode)builder.node().custom1;
  float4x4 matrix;
  switch (mode) {
    case NODE_COMBSEP_MATRIX_COLUMNS:
      copy_v3_v3(matrix[0], storage.vec0);
      matrix[0][3] = 0.0f;
      copy_v3_v3(matrix[1], storage.vec1);
      matrix[1][3] = 0.0f;
      copy_v3_v3(matrix[2], storage.vec2);
      matrix[2][3] = 0.0f;
      copy_v3_v3(matrix[3], storage.vec3);
      matrix[3][3] = 1.0f;
      break;
    case NODE_COMBSEP_MATRIX_ROWS:
      matrix[0][0] = storage.vec0[0];
      matrix[0][1] = storage.vec1[0];
      matrix[0][2] = storage.vec2[0];
      matrix[0][3] = storage.vec3[0];
      matrix[1][0] = storage.vec0[1];
      matrix[1][1] = storage.vec1[1];
      matrix[1][2] = storage.vec2[1];
      matrix[1][3] = storage.vec3[1];
      matrix[2][0] = storage.vec0[2];
      matrix[2][1] = storage.vec1[2];
      matrix[2][2] = storage.vec2[2];
      matrix[2][3] = storage.vec3[2];
      matrix[3][0] = 0.0f;
      matrix[3][1] = 0.0f;
      matrix[3][2] = 0.0f;
      matrix[3][3] = 1.0f;
      break;
    case NODE_COMBSEP_MATRIX_ELEMENTS:
      matrix = float4x4(storage.elements);
      break;
  }
  builder.construct_and_set_matching_fn<fn::CustomMF_Constant<float4x4>>(matrix);
}

}  // namespace blender::nodes::node_fn_input_matrix4x4_cc

void register_node_type_fn_input_matrix_4x4(void)
{
  namespace file_ns = blender::nodes::node_fn_input_matrix4x4_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_INPUT_MATRIX_4X4, "4x4 Matrix", NODE_CLASS_INPUT);
  ntype.declare = file_ns::node_declare;
  ntype.initfunc = file_ns::node_init;
  ntype.build_multi_function = file_ns::node_build_multi_function;
  ntype.draw_buttons = file_ns::node_layout;

  nodeRegisterType(&ntype);
}
