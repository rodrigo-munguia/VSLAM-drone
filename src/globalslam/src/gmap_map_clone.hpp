/**
 * Deep-ish GLOBAL_MAP clone for M7.4 correction workspaces.
 * Clones OpenCV Mats that would otherwise alias across maps.
 */
#ifndef GMAP_MAP_CLONE_HPP
#define GMAP_MAP_CLONE_HPP

#include "gmap_landmark_identity.hpp"
#include "globalslam_types.hpp"

namespace gmap_map_clone
{

inline GLOBAL_MAP deep_clone(const GLOBAL_MAP & src)
{
  GLOBAL_MAP dst;
  dst.idx_ref_pose_slam = src.idx_ref_pose_slam;
  dst.next_landmark_id = src.next_landmark_id;
  dst.KeyFDATA = src.KeyFDATA;
  for (auto & kf : dst.KeyFDATA) {
    if (!kf.frame.empty()) {
      kf.frame = kf.frame.clone();
    }
    if (!kf.Descriptors_all.empty()) {
      kf.Descriptors_all = kf.Descriptors_all.clone();
    }
  }
  dst.AnchorsDATA.reserve(src.AnchorsDATA.size());
  for (const auto & a : src.AnchorsDATA) {
    G_ANCHOR b = a;
    if (!a.Descriptor.empty()) {
      b.Descriptor = a.Descriptor.clone();
    }
    dst.AnchorsDATA.push_back(std::move(b));
  }
  gmap_landmark_id::rebuild_id_to_index(dst);
  if (!src.Vgraph.empty()) {
    dst.Vgraph = src.Vgraph;
  }
  return dst;
}

/** Replace dst content with src (deep). Used for authoritative commit. */
inline void assign_deep(GLOBAL_MAP & dst, const GLOBAL_MAP & src)
{
  dst = deep_clone(src);
}

}  // namespace gmap_map_clone

#endif
