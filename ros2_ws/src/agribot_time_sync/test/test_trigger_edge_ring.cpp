#include <gtest/gtest.h>

#include <cstdio>
#include <string>

#include <agribot_time_sync/trigger_edge_ring.hpp>

namespace
{
class TemporaryPath
{
public:
  TemporaryPath()
  {
    char pattern[] = "/tmp/agribot-trigger-edges-XXXXXX";
    const int descriptor = mkstemp(pattern);
    if (descriptor >= 0) {
      close(descriptor);
    }
    path_ = pattern;
  }
  ~TemporaryPath() {std::remove(path_.c_str());}
  const std::string & path() const {return path_;}

private:
  std::string path_;
};
}  // namespace

TEST(TriggerEdgeRing, PublishesKernelTimestampsAndWrapsSafely)
{
  TemporaryPath temporary;
  agribot_time_sync::TriggerEdgeRingWriter writer(temporary.path());
  agribot_time_sync::TriggerEdgeRingReader reader;
  reader.open_path(temporary.path());

  for (std::uint32_t index = 1; index <= 300U; ++index) {
    EXPECT_EQ(writer.write(1000000000LL + index, index), index);
  }

  EXPECT_EQ(reader.latest_sequence(), 300U);
  agribot_time_sync::TriggerEdge edge{};
  EXPECT_FALSE(reader.read(44U, edge));
  ASSERT_TRUE(reader.read(45U, edge));
  EXPECT_EQ(edge.timestamp_ns, 1000000045LL);
  EXPECT_EQ(edge.kernel_sequence, 45U);
  ASSERT_TRUE(reader.read(300U, edge));
  EXPECT_EQ(edge.timestamp_ns, 1000000300LL);
  EXPECT_EQ(edge.kernel_sequence, 300U);
}

TEST(TriggerEdgeMatcher, UsesFrameNumberAcrossDroppedFrames)
{
  TemporaryPath temporary;
  agribot_time_sync::TriggerEdgeRingWriter writer(temporary.path());
  agribot_time_sync::TriggerEdgeRingReader reader;
  reader.open_path(temporary.path());
  agribot_time_sync::TriggerEdgeMatcher matcher(90000000LL, 2000000LL);

  writer.write(1000000000LL, 1U);
  auto first = matcher.match(100U, 1020000000LL, reader);
  ASSERT_TRUE(first.matched);
  EXPECT_EQ(first.edge.sequence, 1U);

  writer.write(1100000000LL, 2U);
  writer.write(1200000000LL, 3U);
  auto after_drop = matcher.match(102U, 1220000000LL, reader);
  ASSERT_TRUE(after_drop.matched);
  EXPECT_EQ(after_drop.edge.sequence, 3U);
  EXPECT_EQ(after_drop.receipt_delay_ns, 20000000LL);

  auto duplicate = matcher.match(102U, 1400000000LL, reader);
  EXPECT_FALSE(duplicate.matched);
}
