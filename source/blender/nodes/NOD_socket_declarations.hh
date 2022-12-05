/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "NOD_node_declaration.hh"

#include "RNA_types.h"

#include "BLI_color.hh"
#include "BLI_float3x3.hh"
#include "BLI_float4x4.hh"
#include "BLI_math_vec_types.hh"

namespace blender::nodes::decl {

class FloatBuilder;

class Float : public SocketDeclaration {
 private:
  float default_value_ = 0.0f;
  float soft_min_value_ = -FLT_MAX;
  float soft_max_value_ = FLT_MAX;
  PropertySubType subtype_ = PROP_NONE;

  friend FloatBuilder;

 public:
  using Builder = FloatBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class FloatBuilder : public SocketDeclarationBuilder<Float> {
 public:
  FloatBuilder &min(float value);
  FloatBuilder &max(float value);
  FloatBuilder &default_value(float value);
  FloatBuilder &subtype(PropertySubType subtype);
};

class IntBuilder;

class Int : public SocketDeclaration {
 private:
  int default_value_ = 0;
  int soft_min_value_ = INT32_MIN;
  int soft_max_value_ = INT32_MAX;
  PropertySubType subtype_ = PROP_NONE;

  friend IntBuilder;

 public:
  using Builder = IntBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class IntBuilder : public SocketDeclarationBuilder<Int> {
 public:
  IntBuilder &min(int value);
  IntBuilder &max(int value);
  IntBuilder &default_value(int value);
  IntBuilder &subtype(PropertySubType subtype);
};

class VectorBuilder;

class Vector : public SocketDeclaration {
 private:
  float3 default_value_ = {0, 0, 0};
  float soft_min_value_ = -FLT_MAX;
  float soft_max_value_ = FLT_MAX;
  PropertySubType subtype_ = PROP_NONE;

  friend VectorBuilder;

 public:
  using Builder = VectorBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class VectorBuilder : public SocketDeclarationBuilder<Vector> {
 public:
  VectorBuilder &default_value(const float3 value);
  VectorBuilder &subtype(PropertySubType subtype);
  VectorBuilder &min(float min);
  VectorBuilder &max(float max);
  VectorBuilder &compact();
};

class Matrix3x3Builder;

class Matrix3x3 : public SocketDeclaration {
 private:
  float3x3 default_value_ = float3x3::identity();
  float soft_min_value_ = -FLT_MAX;
  float soft_max_value_ = FLT_MAX;

  friend Matrix3x3Builder;

 public:
  using Builder = Matrix3x3Builder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class Matrix3x3Builder : public SocketDeclarationBuilder<Matrix3x3> {
 public:
  Matrix3x3Builder &default_value(const float3x3 value);
  Matrix3x3Builder &min(float min);
  Matrix3x3Builder &max(float max);
};

class Matrix4x4Builder;

class Matrix4x4 : public SocketDeclaration {
 private:
  float4x4 default_value_ = float4x4::identity();
  float soft_min_value_ = -FLT_MAX;
  float soft_max_value_ = FLT_MAX;

  friend Matrix4x4Builder;

 public:
  using Builder = Matrix4x4Builder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class Matrix4x4Builder : public SocketDeclarationBuilder<Matrix4x4> {
 public:
  Matrix4x4Builder &default_value(const float4x4 value);
  Matrix4x4Builder &min(float min);
  Matrix4x4Builder &max(float max);
};

class BoolBuilder;

class Bool : public SocketDeclaration {
 private:
  bool default_value_ = false;
  friend BoolBuilder;

 public:
  using Builder = BoolBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class BoolBuilder : public SocketDeclarationBuilder<Bool> {
 public:
  BoolBuilder &default_value(bool value);
};

class ColorBuilder;

class Color : public SocketDeclaration {
 private:
  ColorGeometry4f default_value_;

  friend ColorBuilder;

 public:
  using Builder = ColorBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class ColorBuilder : public SocketDeclarationBuilder<Color> {
 public:
  ColorBuilder &default_value(const ColorGeometry4f value);
};

class StringBuilder;

class String : public SocketDeclaration {
 private:
  std::string default_value_;

  friend StringBuilder;

 public:
  using Builder = StringBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class StringBuilder : public SocketDeclarationBuilder<String> {
 public:
  StringBuilder &default_value(const std::string value);
};

class IDSocketDeclaration : public SocketDeclaration {
 private:
  const char *idname_;

 public:
  IDSocketDeclaration(const char *idname);

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bNodeSocket &update_or_build(bNodeTree &ntree, bNode &node, bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class Object : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Object>;

  Object();
};

class Material : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Material>;

  Material();
};

class Collection : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Collection>;

  Collection();
};

class Texture : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Texture>;

  Texture();
};

class Image : public IDSocketDeclaration {
 public:
  using Builder = SocketDeclarationBuilder<Image>;

  Image();
};

class ShaderBuilder;

class Shader : public SocketDeclaration {
 private:
  friend ShaderBuilder;

 public:
  using Builder = ShaderBuilder;

  bNodeSocket &build(bNodeTree &ntree, bNode &node) const override;
  bool matches(const bNodeSocket &socket) const override;
  bool can_connect(const bNodeSocket &socket) const override;
};

class ShaderBuilder : public SocketDeclarationBuilder<Shader> {
};

/* -------------------------------------------------------------------- */
/** \name #FloatBuilder Inline Methods
 * \{ */

inline FloatBuilder &FloatBuilder::min(const float value)
{
  decl_->soft_min_value_ = value;
  return *this;
}

inline FloatBuilder &FloatBuilder::max(const float value)
{
  decl_->soft_max_value_ = value;
  return *this;
}

inline FloatBuilder &FloatBuilder::default_value(const float value)
{
  decl_->default_value_ = value;
  return *this;
}

inline FloatBuilder &FloatBuilder::subtype(PropertySubType subtype)
{
  decl_->subtype_ = subtype;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #IntBuilder Inline Methods
 * \{ */

inline IntBuilder &IntBuilder::min(const int value)
{
  decl_->soft_min_value_ = value;
  return *this;
}

inline IntBuilder &IntBuilder::max(const int value)
{
  decl_->soft_max_value_ = value;
  return *this;
}

inline IntBuilder &IntBuilder::default_value(const int value)
{
  decl_->default_value_ = value;
  return *this;
}

inline IntBuilder &IntBuilder::subtype(PropertySubType subtype)
{
  decl_->subtype_ = subtype;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #VectorBuilder Inline Methods
 * \{ */

inline VectorBuilder &VectorBuilder::default_value(const float3 value)
{
  decl_->default_value_ = value;
  return *this;
}

inline VectorBuilder &VectorBuilder::subtype(PropertySubType subtype)
{
  decl_->subtype_ = subtype;
  return *this;
}

inline VectorBuilder &VectorBuilder::min(const float min)
{
  decl_->soft_min_value_ = min;
  return *this;
}

inline VectorBuilder &VectorBuilder::max(const float max)
{
  decl_->soft_max_value_ = max;
  return *this;
}

inline VectorBuilder &VectorBuilder::compact()
{
  decl_->compact_ = true;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #BoolBuilder Inline Methods
 * \{ */

inline BoolBuilder &BoolBuilder::default_value(const bool value)
{
  decl_->default_value_ = value;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #ColorBuilder Inline Methods
 * \{ */

inline ColorBuilder &ColorBuilder::default_value(const ColorGeometry4f value)
{
  decl_->default_value_ = value;
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #StringBuilder Inline Methods
 * \{ */

inline StringBuilder &StringBuilder::default_value(std::string value)
{
  decl_->default_value_ = std::move(value);
  return *this;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name #IDSocketDeclaration and Children Inline Methods
 * \{ */

inline IDSocketDeclaration::IDSocketDeclaration(const char *idname) : idname_(idname)
{
}

inline Object::Object() : IDSocketDeclaration("NodeSocketObject")
{
}

inline Material::Material() : IDSocketDeclaration("NodeSocketMaterial")
{
}

inline Collection::Collection() : IDSocketDeclaration("NodeSocketCollection")
{
}

inline Texture::Texture() : IDSocketDeclaration("NodeSocketTexture")
{
}

inline Image::Image() : IDSocketDeclaration("NodeSocketImage")
{
}

/** \} */

}  // namespace blender::nodes::decl
