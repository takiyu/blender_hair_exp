#pragma once

#include "BLI_index_range.hh"
#include "BLI_span.hh"

namespace blender {

template<typename T> class OffsetArrayRef {
 private:
  static_assert(std::is_integral_v<T>);

  Span<T> offsets_;

 public:
  OffsetArrayRef(const Span<T> offsets) : offsets_(offsets)
  {
    BLI_assert(std::is_sorted(offsets_.begin(), offsets_.end()));
  }

  IndexRange operator[](const int64_t index) const
  {
    BLI_assert(index >= 0);
    BLI_assert(index < offsets_.size() - 1);
    const int64_t begin = offsets_[index];
    const int64_t end = offsets_[index + 1];
    const int64_t size = end - begin;
    return IndexRange(begin, size);
  }

  IndexRange operator[](const IndexRange indices) const
  {
    const int64_t begin = offsets_[indices.start()];
    const int64_t end = offsets_[indices.one_after_last()];
    const int64_t size = end - begin;
    return IndexRange(begin, size);
  }
};

}  // namespace blender
