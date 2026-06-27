#include <gtest/gtest.h>

#include "radar_lidar/cluster.hpp"
#include "radar_lidar/cluster_message.hpp"

TEST(ClusterMessageTest, PreservesStructuredClusterFields) {
    radar::ClusterResult cluster;
    cluster.centroid    = Eigen::Vector3d(1.0, 2.0, 3.0);
    cluster.min_bound   = Eigen::Vector3d(0.5, 1.5, 2.5);
    cluster.max_bound   = Eigen::Vector3d(1.5, 2.5, 3.5);
    cluster.point_count = 42;

    auto msg = radar::make_cluster_array_message({ cluster }, 123456789LL, "map");

    ASSERT_EQ(msg.header.stamp.sec, 0);
    ASSERT_EQ(msg.header.stamp.nanosec, 123456789u);
    ASSERT_EQ(msg.header.frame_id, "map");
    ASSERT_EQ(msg.clusters.size(), 1u);

    const auto& out = msg.clusters.front();
    EXPECT_DOUBLE_EQ(out.centroid.x, 1.0);
    EXPECT_DOUBLE_EQ(out.centroid.y, 2.0);
    EXPECT_DOUBLE_EQ(out.centroid.z, 3.0);
    EXPECT_DOUBLE_EQ(out.min_bound.x, 0.5);
    EXPECT_DOUBLE_EQ(out.min_bound.y, 1.5);
    EXPECT_DOUBLE_EQ(out.min_bound.z, 2.5);
    EXPECT_DOUBLE_EQ(out.max_bound.x, 1.5);
    EXPECT_DOUBLE_EQ(out.max_bound.y, 2.5);
    EXPECT_DOUBLE_EQ(out.max_bound.z, 3.5);
    EXPECT_EQ(out.point_count, 42);
}
