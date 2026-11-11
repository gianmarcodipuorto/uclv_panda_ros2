#include <gtest/gtest.h>

#include "panda_utils/bridge/jitter_buffer.hpp"

using panda_bridge::JitterBuffer;

namespace {

using Buffer = JitterBuffer<1, 64>;

constexpr int64_t kMs = 1'000'000;

double quadratic(double t) { return 1.0 + 2.0 * t + 3.0 * t * t; }

Buffer::Sample sample(int64_t stamp_ns, double value) {
  Buffer::Sample s;
  s.stamp_ns = stamp_ns;
  s.values[0] = value;
  return s;
}

}  // namespace

TEST(JitterBuffer, RejectsDuplicatesAndReordering) {
  Buffer buffer;
  EXPECT_TRUE(buffer.push(sample(10 * kMs, 1.0)));
  EXPECT_FALSE(buffer.push(sample(10 * kMs, 2.0)));
  EXPECT_FALSE(buffer.push(sample(9 * kMs, 2.0)));
  EXPECT_TRUE(buffer.push(sample(11 * kMs, 2.0)));
  EXPECT_EQ(buffer.size(), 2u);
}

TEST(JitterBuffer, OverwritesOldestWhenFull) {
  Buffer buffer;
  for (int i = 0; i < 100; ++i) {
    buffer.push(sample(i * kMs, i));
  }
  EXPECT_EQ(buffer.size(), 64u);
  EXPECT_EQ(buffer.oldest().stamp_ns, 36 * kMs);
  EXPECT_EQ(buffer.newest().stamp_ns, 99 * kMs);
}

TEST(JitterBuffer, ReportsStatusOutsideTheSamples) {
  Buffer buffer;
  EXPECT_EQ(buffer.sample_fit(0, 3 * kMs).status, Buffer::Status::kEmpty);

  buffer.push(sample(10 * kMs, 1.0));
  buffer.push(sample(11 * kMs, 2.0));
  EXPECT_EQ(buffer.sample_fit(9 * kMs, 3 * kMs).status, Buffer::Status::kWaiting);

  const auto starved = buffer.sample_fit(14 * kMs, 3 * kMs);
  EXPECT_EQ(starved.status, Buffer::Status::kStarved);
  EXPECT_EQ(starved.age_ns, 3 * kMs);
  // Two samples only: linear extrapolation
  EXPECT_NEAR(starved.value[0], 5.0, 1e-9);
  EXPECT_NEAR(starved.rate[0], 1000.0, 1e-6);
}

TEST(JitterBuffer, FitReproducesQuadraticWithDerivatives) {
  Buffer buffer;
  for (int i = 0; i < 20; ++i) {
    const double t = i * 1e-3;
    buffer.push(sample(i * kMs, quadratic(t)));
  }
  for (double t = 5.25e-3; t < 15e-3; t += 0.5e-3) {
    const auto ref = buffer.sample_fit(static_cast<int64_t>(t * 1e9), 3 * kMs);
    ASSERT_EQ(ref.status, Buffer::Status::kInterpolated);
    EXPECT_NEAR(ref.value[0], quadratic(t), 1e-9);
    EXPECT_NEAR(ref.rate[0], 2.0 + 6.0 * t, 1e-6);
    EXPECT_NEAR(ref.rate_dot[0], 6.0, 1e-6);
  }
}

TEST(JitterBuffer, HandlesNonUniformSpacing) {
  Buffer buffer;
  const int64_t stamps[] = {0, 1 * kMs, 3 * kMs, 4 * kMs, 7 * kMs, 8 * kMs};
  for (int64_t stamp : stamps) {
    buffer.push(sample(stamp, 0.5 * stamp * 1e-9));  // linear with slope 0.5
  }
  const auto ref = buffer.sample_fit(5 * kMs, 3 * kMs);
  ASSERT_EQ(ref.status, Buffer::Status::kInterpolated);
  EXPECT_NEAR(ref.value[0], 0.5 * 5e-3, 1e-12);
  EXPECT_NEAR(ref.rate[0], 0.5, 1e-9);
}

TEST(JitterBuffer, StarvedExtrapolatesTheFit) {
  Buffer buffer;
  for (int i = 0; i <= 10; ++i) {
    buffer.push(sample(i * kMs, quadratic(i * 1e-3)));
  }
  const auto ref = buffer.sample_fit(14 * kMs, 3 * kMs);
  ASSERT_EQ(ref.status, Buffer::Status::kStarved);
  EXPECT_NEAR(ref.value[0], quadratic(14e-3), 1e-9);
  EXPECT_NEAR(ref.rate[0], 2.0 + 6.0 * 14e-3, 1e-6);
  EXPECT_NEAR(ref.rate_dot[0], 6.0, 1e-6);
}

TEST(JitterBuffer, LinearInterpolation) {
  Buffer buffer;
  buffer.push(sample(0, 0.0));
  buffer.push(sample(10 * kMs, 1.0));
  const auto ref = buffer.sample_linear(2 * kMs);
  ASSERT_EQ(ref.status, Buffer::Status::kInterpolated);
  EXPECT_NEAR(ref.value[0], 0.2, 1e-12);
  EXPECT_NEAR(ref.rate[0], 100.0, 1e-9);
}
