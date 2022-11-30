/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "DNA_mesh_types.h"
#include "DNA_meshdata_types.h"

#include "BKE_mesh.h"
#include "BKE_mesh_simplex.hh"

#include "node_geometry_util.hh"

namespace blender::nodes::node_geo_input_mesh_simplex_vertices_cc {

using blender::bke::SimplexGeometry;

static void node_declare(NodeDeclarationBuilder &b)
{
  b.add_output<decl::Int>(N_("Vertex Index 1"))
      .field_source()
      .description(N_("The index of the first vertex in the simplex"));
  b.add_output<decl::Int>(N_("Vertex Index 2"))
      .field_source()
      .description(N_("The index of the second vertex in the simplex"));
  b.add_output<decl::Int>(N_("Vertex Index 3"))
      .field_source()
      .description(N_("The index of the third vertex in the simplex"));
  b.add_output<decl::Int>(N_("Vertex Index 4"))
      .field_source()
      .description(N_("The index of the fourth vertex in the simplex"));
  b.add_output<decl::Vector>(N_("Position 1"))
      .field_source()
      .description(N_("The position of the first vertex in the simplex"));
  b.add_output<decl::Vector>(N_("Position 2"))
      .field_source()
      .description(N_("The position of the second vertex in the simplex"));
  b.add_output<decl::Vector>(N_("Position 3"))
      .field_source()
      .description(N_("The position of the third vertex in the simplex"));
  b.add_output<decl::Vector>(N_("Position 4"))
      .field_source()
      .description(N_("The position of the fourth vertex in the simplex"));
}

enum class VertNumber { V1, V2, V3, V4 };

template<int V>
static VArray<int> construct_simplex_verts_gvarray(const SimplexGeometry &geometry)
{
  const Span<int4> verts = geometry.tetrahedrons();
  return VArray<int>::ForFunc(verts.size(), [verts](const int i) -> int { return verts[i][V]; });
}

static VArray<int> construct_simplex_verts_gvarray(const SimplexGeometry &geometry,
                                                   const VertNumber vertex,
                                                   const eAttrDomain domain)
{
  if (domain == ATTR_DOMAIN_SIMPLEX) {
    switch (vertex) {
      case VertNumber::V1:
        return construct_simplex_verts_gvarray<0>(geometry);
      case VertNumber::V2:
        return construct_simplex_verts_gvarray<1>(geometry);
      case VertNumber::V3:
        return construct_simplex_verts_gvarray<2>(geometry);
      case VertNumber::V4:
        return construct_simplex_verts_gvarray<3>(geometry);
    }
  }
  return {};
}

class SimplexVertsInput final : public bke::SimplexFieldInput {
 private:
  VertNumber vertex_;

 public:
  SimplexVertsInput(VertNumber vertex)
      : bke::SimplexFieldInput(CPPType::get<int>(), "Simplex Vertices Field"), vertex_(vertex)
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const SimplexGeometry &geometry,
                                 const eAttrDomain domain,
                                 const IndexMask /*mask*/) const final
  {
    return construct_simplex_verts_gvarray(geometry, vertex_, domain);
  }

  uint64_t hash() const override
  {
    switch (vertex_) {
      case VertNumber::V1:
        return 37104786908910;
      case VertNumber::V2:
        return 81169989515310;
      case VertNumber::V3:
        return 88431189047671;
      case VertNumber::V4:
        return 68220415607437;
    }
    return 0;
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    if (const SimplexVertsInput *other_field = dynamic_cast<const SimplexVertsInput *>(&other)) {
      return vertex_ == other_field->vertex_;
    }
    return false;
  }

  std::optional<eAttrDomain> preferred_domain(const SimplexGeometry & /*geometry*/) const override
  {
    return ATTR_DOMAIN_SIMPLEX;
  }
};

template <int V>
static VArray<float3> construct_simplex_positions_gvarray(const SimplexGeometry &geometry)
{
  const Span<float3> positions = geometry.positions();
  const Span<int4> verts = geometry.tetrahedrons();

  return VArray<float3>::ForFunc(verts.size(), [positions, verts](const int i) {
    const int index = verts[i][V];
    return positions.index_range().contains(index) ? positions[index] : float3(0.0f);
  });
}

static VArray<float3> construct_simplex_positions_gvarray(const SimplexGeometry &geometry,
                                                          const VertNumber vertex,
                                                          const eAttrDomain domain)
{
  switch (vertex) {
    case VertNumber::V1:
      return geometry.attributes().adapt_domain<float3>(
          construct_simplex_positions_gvarray<0>(geometry), ATTR_DOMAIN_SIMPLEX, domain);
    case VertNumber::V2:
      return geometry.attributes().adapt_domain<float3>(
          construct_simplex_positions_gvarray<1>(geometry), ATTR_DOMAIN_SIMPLEX, domain);
    case VertNumber::V3:
      return geometry.attributes().adapt_domain<float3>(
          construct_simplex_positions_gvarray<2>(geometry), ATTR_DOMAIN_SIMPLEX, domain);
    case VertNumber::V4:
      return geometry.attributes().adapt_domain<float3>(
          construct_simplex_positions_gvarray<3>(geometry), ATTR_DOMAIN_SIMPLEX, domain);
  }
}

class SimplexPositionFieldInput final : public bke::SimplexFieldInput {
 private:
  VertNumber vertex_;

 public:
  SimplexPositionFieldInput(VertNumber vertex)
      : bke::SimplexFieldInput(CPPType::get<float3>(), "Simplex Position Field"), vertex_(vertex)
  {
    category_ = Category::Generated;
  }

  GVArray get_varray_for_context(const SimplexGeometry &geometry,
                                 const eAttrDomain domain,
                                 IndexMask /*mask*/) const final
  {
    return construct_simplex_positions_gvarray(geometry, vertex_, domain);
  }

  uint64_t hash() const override
  {
    switch (vertex_) {
      case VertNumber::V1:
        return 72980954293981;
      case VertNumber::V2:
        return 36610862259396;
      case VertNumber::V3:
        return 90570267722071;
      case VertNumber::V4:
        return 36968683208228;
    }
    return 0;
  }

  bool is_equal_to(const fn::FieldNode &other) const override
  {
    if (const SimplexPositionFieldInput *other_field =
            dynamic_cast<const SimplexPositionFieldInput *>(&other)) {
      return vertex_ == other_field->vertex_;
    }
    return false;
  }

  std::optional<eAttrDomain> preferred_domain(const SimplexGeometry & /*geometry*/) const override
  {
    return ATTR_DOMAIN_SIMPLEX;
  }
};

static void node_geo_exec(GeoNodeExecParams params)
{
  Field<int> vertex_field_1{std::make_shared<SimplexVertsInput>(VertNumber::V1)};
  Field<int> vertex_field_2{std::make_shared<SimplexVertsInput>(VertNumber::V2)};
  Field<int> vertex_field_3{std::make_shared<SimplexVertsInput>(VertNumber::V3)};
  Field<int> vertex_field_4{std::make_shared<SimplexVertsInput>(VertNumber::V4)};
  Field<float3> position_field_1{std::make_shared<SimplexPositionFieldInput>(VertNumber::V1)};
  Field<float3> position_field_2{std::make_shared<SimplexPositionFieldInput>(VertNumber::V2)};
  Field<float3> position_field_3{std::make_shared<SimplexPositionFieldInput>(VertNumber::V3)};
  Field<float3> position_field_4{std::make_shared<SimplexPositionFieldInput>(VertNumber::V4)};

  params.set_output("Vertex Index 1", std::move(vertex_field_1));
  params.set_output("Vertex Index 2", std::move(vertex_field_2));
  params.set_output("Vertex Index 3", std::move(vertex_field_3));
  params.set_output("Vertex Index 4", std::move(vertex_field_4));
  params.set_output("Position 1", std::move(position_field_1));
  params.set_output("Position 2", std::move(position_field_2));
  params.set_output("Position 3", std::move(position_field_3));
  params.set_output("Position 4", std::move(position_field_4));
}

}  // namespace blender::nodes::node_geo_input_mesh_edge_vertices_cc

void register_node_type_geo_input_mesh_simplex_vertices()
{
  namespace file_ns = blender::nodes::node_geo_input_mesh_simplex_vertices_cc;

  static bNodeType ntype;
  geo_node_type_base(
      &ntype, GEO_NODE_INPUT_MESH_SIMPLEX_VERTICES, "Simplex Vertices", NODE_CLASS_INPUT);
  ntype.declare = file_ns::node_declare;
  ntype.geometry_node_execute = file_ns::node_geo_exec;
  nodeRegisterType(&ntype);
}
