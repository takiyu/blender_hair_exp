/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "node_function_util.hh"

#include "BKE_node_runtime.hh"

#include "UI_interface.h"
#include "UI_resources.h"

namespace blender::nodes::node_fn_decompose_matrix4x4_cc {

static void node_declare(NodeDeclarationBuilder &b)
{
  b.is_function_node();
  b.add_input<decl::Matrix4x4>(N_("Matrix"));
  b.add_output<decl::Vector>(N_("Translation"));
  b.add_output<decl::Vector>(N_("Rotation"));
  b.add_output<decl::Vector>(N_("Scale"));
};

class DecomposeMatrix4x4Function : public fn::MultiFunction {
 public:
  DecomposeMatrix4x4Function()
  {
    static fn::MFSignature signature = create_signature();
    this->set_signature(&signature);
  }

  static fn::MFSignature create_signature()
  {
    fn::MFSignatureBuilder signature{"Decompose Matrix 3x3"};
    signature.single_input<float4x4>("Matrix");
    signature.single_output<float3>("Translation");
    signature.single_output<float3>("Rotation");
    signature.single_output<float3>("Scale");
    return signature.build();
  }

  void call(IndexMask mask, fn::MFParams params, fn::MFContext /*context*/) const override
  {
    const VArray<float4x4> &matrices = params.readonly_single_input<float4x4>(0, "Matrix");
    MutableSpan<float3> translations = params.uninitialized_single_output<float3>(0, "Translation");
    MutableSpan<float3> rotations = params.uninitialized_single_output<float3>(0, "Rotation");
    MutableSpan<float3> scales = params.uninitialized_single_output<float3>(1, "Scale");

    for (int64_t i : mask) {
      const float4x4 &mat = matrices[i];
      copy_v3_v3(translations[i], mat[3]);
      mat4_to_eul(rotations[i], mat.ptr());
      mat4_to_size(scales[i], mat.ptr());
    }
  }
};

static void node_build_multi_function(NodeMultiFunctionBuilder &builder)
{
  static DecomposeMatrix4x4Function decompose_matrix_fn;
  builder.set_matching_fn(&decompose_matrix_fn);
}

}  // namespace blender::nodes::node_fn_decompose_matrix4x4_cc

void register_node_type_fn_decompose_matrix_4x4(void)
{
  namespace file_ns = blender::nodes::node_fn_decompose_matrix4x4_cc;

  static bNodeType ntype;

  fn_node_type_base(&ntype, FN_NODE_DECOMPOSE_MATRIX_4X4, "Decompose 4x4 Matrix", NODE_CLASS_CONVERTER);
  ntype.declare = file_ns::node_declare;
  ntype.build_multi_function = file_ns::node_build_multi_function;

  nodeRegisterType(&ntype);
}
