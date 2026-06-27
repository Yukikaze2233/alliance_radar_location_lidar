#pragma once

#include <string>
#include <vector>

#include "radar_interfaces/msg/cluster.hpp"
#include "radar_interfaces/msg/cluster_array.hpp"
#include "radar_lidar/cluster.hpp"
#include "radar_lidar/types.hpp"

namespace radar {

inline auto make_cluster_message(const ClusterResult& cluster) -> radar_interfaces::msg::Cluster {
    radar_interfaces::msg::Cluster msg;

    msg.centroid.x = cluster.centroid.x();
    msg.centroid.y = cluster.centroid.y();
    msg.centroid.z = cluster.centroid.z();

    msg.min_bound.x = cluster.min_bound.x();
    msg.min_bound.y = cluster.min_bound.y();
    msg.min_bound.z = cluster.min_bound.z();

    msg.max_bound.x = cluster.max_bound.x();
    msg.max_bound.y = cluster.max_bound.y();
    msg.max_bound.z = cluster.max_bound.z();

    msg.point_count = cluster.point_count;
    return msg;
}

inline auto make_cluster_array_message(const std::vector<ClusterResult>& clusters,
    types::Timestamp stamp, const std::string& frame_id) -> radar_interfaces::msg::ClusterArray {
    radar_interfaces::msg::ClusterArray msg;
    msg.header.stamp.sec     = static_cast<int32_t>(stamp / 1000000000LL);
    msg.header.stamp.nanosec = static_cast<uint32_t>(stamp % 1000000000LL);
    msg.header.frame_id      = frame_id;
    msg.clusters.reserve(clusters.size());

    for (const auto& cluster : clusters) {
        msg.clusters.push_back(make_cluster_message(cluster));
    }

    return msg;
}

}
