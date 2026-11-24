// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// \file
/// \brief Move assignment of Bonxai::Mask and Bonxai::Grid.

#include <gtest/gtest.h>

#include <utility>
#include <vector>

#include "bonxai/bonxai.hpp"
#include "bonxai/mask.hpp"

namespace
{

// log2dim 2 -> 64 bits (1 word, inline); log2dim 4 -> 4096 bits (64 words, heap)
constexpr size_t kSmall = 2;
constexpr size_t kInline8 = 3;
constexpr size_t kLarge = 4;

Bonxai::Mask make_mask(size_t log2dim, const std::vector<uint32_t> & bits)
{
  Bonxai::Mask m(log2dim);
  for (auto b : bits) {
    m.setOn(b);
  }
  return m;
}

void expect_bits(const Bonxai::Mask & m, size_t log2dim, const std::vector<uint32_t> & bits)
{
  ASSERT_EQ(m.size(), 1u << (3 * log2dim));
  EXPECT_EQ(m.countOn(), bits.size());
  for (auto b : bits) {
    EXPECT_TRUE(m.isOn(b)) << "bit " << b;
  }
}

}  // namespace

TEST(BonxaiMaskMove, SmallIntoSmall)
{
  Bonxai::Mask dst = make_mask(kSmall, {1});
  Bonxai::Mask src = make_mask(kSmall, {0, 63});
  dst = std::move(src);
  expect_bits(dst, kSmall, {0, 63});
}

TEST(BonxaiMaskMove, LargeIntoLarge)
{
  Bonxai::Mask dst = make_mask(kLarge, {5, 4000});
  Bonxai::Mask src = make_mask(kLarge, {0, 777, 4095});
  dst = std::move(src);
  expect_bits(dst, kLarge, {0, 777, 4095});
  // moved-from heap mask is left empty and safe to destroy
  EXPECT_EQ(src.size(), 0u);
  EXPECT_EQ(src.countOn(), 0u);
}

TEST(BonxaiMaskMove, LargeIntoSmall)
{
  Bonxai::Mask dst = make_mask(kSmall, {3});
  Bonxai::Mask src = make_mask(kLarge, {2048, 4095});
  dst = std::move(src);
  expect_bits(dst, kLarge, {2048, 4095});
}

TEST(BonxaiMaskMove, SmallIntoLarge)
{
  // destination releases its heap words and switches to inline storage
  Bonxai::Mask dst = make_mask(kLarge, {100, 4095});
  Bonxai::Mask src = make_mask(kSmall, {7});
  dst = std::move(src);
  expect_bits(dst, kSmall, {7});
}

TEST(BonxaiMaskMove, InlineLimitEightWords)
{
  Bonxai::Mask dst = make_mask(kLarge, {1});
  Bonxai::Mask src = make_mask(kInline8, {0, 511});
  dst = std::move(src);
  expect_bits(dst, kInline8, {0, 511});
  // inline source keeps its contents
  expect_bits(src, kInline8, {0, 511});
}

TEST(BonxaiMaskMove, SelfMoveKeepsContents)
{
  Bonxai::Mask small = make_mask(kSmall, {9});
  Bonxai::Mask large = make_mask(kLarge, {9, 3000});
  Bonxai::Mask & small_ref = small;
  Bonxai::Mask & large_ref = large;
  small = std::move(small_ref);
  large = std::move(large_ref);
  expect_bits(small, kSmall, {9});
  expect_bits(large, kLarge, {9, 3000});
}

TEST(BonxaiMaskMove, MoveConstructedSourceCanBeReassigned)
{
  Bonxai::Mask src = make_mask(kLarge, {10});
  Bonxai::Mask taken(std::move(src));
  expect_bits(taken, kLarge, {10});
  src = make_mask(kLarge, {20, 30});
  expect_bits(src, kLarge, {20, 30});
}

TEST(BonxaiMaskMove, ChainedMoves)
{
  Bonxai::Mask a = make_mask(kLarge, {1, 2});
  Bonxai::Mask b(kSmall);
  Bonxai::Mask c(kInline8);
  b = std::move(a);
  c = std::move(b);
  a = std::move(c);
  expect_bits(a, kLarge, {1, 2});
}

TEST(BonxaiGridMove, MoveAssignKeepsMaskAndData)
{
  Bonxai::Grid<float> dst(kSmall);
  Bonxai::Grid<float> src(kLarge);
  src.mask().setOn(42);
  src.cell(42) = 3.5f;
  dst = std::move(src);
  EXPECT_EQ(dst.size(), 4096u);
  EXPECT_TRUE(dst.mask().isOn(42));
  EXPECT_FLOAT_EQ(dst.cell(42), 3.5f);
}
