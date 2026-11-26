/**
 * M7.5 — Controlled authoritative map read / owned computational extracts.
 *
 * Callers must hold the component authoritative-map mutex around extract
 * entry points that touch live GMAP storage. Returned data is owned; no
 * pointers/references/iterators into the authoritative map escape.
 *
 * Matching, PnP, ROS publish, and proposal waits must run outside that lock.
 */
#ifndef GMAP_AUTHORITATIVE_READ_HPP
#define GMAP_AUTHORITATIVE_READ_HPP

#include "gmap_map_clone.hpp"
#include "globalslam_types.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace gmap_auth_read
{

/** Owned detection computational input (temporary; not a persistent replica). */
struct DetectionExtract
{
  GLOBAL_MAP map;
  uint64_t map_lifetime{0};
  uint64_t correction_epoch{0};
  uint64_t auth_seq{0};
};

/**
 * Deep-owned detection extract from a coherent authoritative snapshot.
 * OpenCV Mats are cloned (see gmap_map_clone::deep_clone).
 */
inline DetectionExtract make_detection_extract(
  const GLOBAL_MAP & auth,
  uint64_t map_lifetime,
  uint64_t correction_epoch,
  uint64_t auth_seq)
{
  DetectionExtract out;
  out.map = gmap_map_clone::deep_clone(auth);
  out.map_lifetime = map_lifetime;
  out.correction_epoch = correction_epoch;
  out.auth_seq = auth_seq;
  return out;
}

/** Compact KF pose row for candidate ranking / significance (owned values). */
struct CandidateKfPose
{
  int64_t keyframe_id{-1};
  int storage_index{-1};
  arma::vec::fixed<3> t_c2n;
};

/**
 * Candidate-selection pose list preserving KeyFDATA order.
 * Does not copy descriptors or landmark storage.
 */
inline std::vector<CandidateKfPose> extract_candidate_poses(const GLOBAL_MAP & auth)
{
  std::vector<CandidateKfPose> out;
  out.reserve(auth.KeyFDATA.size());
  for (std::size_t i = 0; i < auth.KeyFDATA.size(); ++i) {
    CandidateKfPose row;
    row.keyframe_id = static_cast<int64_t>(auth.KeyFDATA[i].keyframe_id);
    row.storage_index = static_cast<int>(i);
    row.t_c2n = auth.KeyFDATA[i].t_c2n;
    out.push_back(row);
  }
  return out;
}

/** Publication geometry only (no descriptors / images). */
struct PublicationGeometry
{
  std::vector<arma::mat::fixed<3, 3>> Rn2c;
  std::vector<arma::vec::fixed<3>> t_c2n;
  std::vector<arma::vec::fixed<3>> landmark_xyz;
};

inline PublicationGeometry extract_publication_geometry(const GLOBAL_MAP & auth)
{
  PublicationGeometry out;
  out.Rn2c.reserve(auth.KeyFDATA.size());
  out.t_c2n.reserve(auth.KeyFDATA.size());
  for (const auto & kf : auth.KeyFDATA) {
    out.Rn2c.push_back(kf.Rn2c);
    out.t_c2n.push_back(kf.t_c2n);
  }
  out.landmark_xyz.reserve(auth.AnchorsDATA.size());
  for (const auto & a : auth.AnchorsDATA) {
    out.landmark_xyz.push_back(a.AnchorState);
  }
  return out;
}

}  // namespace gmap_auth_read

#endif
