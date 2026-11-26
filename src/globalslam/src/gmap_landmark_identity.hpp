/**
 * GSLAM-MAP-ARCH-M1/M2: production stable landmark identity + dense lookup.
 *
 * Operational KF observations remain storage-index-based until M3–M5.
 * These helpers maintain LandmarkId / id_to_index / next_landmark_id only.
 */
#ifndef GMAP_LANDMARK_IDENTITY_HPP
#define GMAP_LANDMARK_IDENTITY_HPP

#include <cstdint>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "globalslam_types.hpp"

namespace gmap_landmark_id
{

/** Invalid / unassigned: value == 0. Valid IDs allocate from 1 upward. */
inline constexpr LandmarkId kInvalidId{0};

inline bool is_valid(LandmarkId id) { return id.value != 0; }

inline std::string to_string(LandmarkId id)
{
  return std::to_string(id.value);
}

/**
 * Allocate next production ID (does not register). Never returns invalid
 * unless the counter is exhausted (should not occur in practice).
 */
inline LandmarkId allocate_landmark_id(GLOBAL_MAP & map)
{
  if (map.next_landmark_id == 0) {
    map.next_landmark_id = 1;
  }
  if (map.next_landmark_id == std::numeric_limits<std::uint64_t>::max()) {
    return kInvalidId;
  }
  const LandmarkId id{map.next_landmark_id};
  ++map.next_landmark_id;
  return id;
}

inline void rebuild_id_to_index(GLOBAL_MAP & map)
{
  map.id_to_index.clear();
  map.id_to_index.reserve(map.AnchorsDATA.size());
  for (std::size_t i = 0; i < map.AnchorsDATA.size(); ++i) {
    const LandmarkId id = map.AnchorsDATA[i].id;
    if (!is_valid(id)) {
      continue;
    }
    map.id_to_index[id] = i;
  }
}

/** True if ID is present in the lookup table. Does not insert. */
inline bool contains_landmark(const GLOBAL_MAP & map, LandmarkId id)
{
  if (!is_valid(id)) {
    return false;
  }
  return map.id_to_index.find(id) != map.id_to_index.end();
}

/**
 * Resolve ID → storage index. Fail-closed; never uses operator[].
 * @return false if missing/invalid (out unchanged).
 */
inline bool resolve_landmark_index(
  const GLOBAL_MAP & map, LandmarkId id, std::size_t & out_index)
{
  if (!is_valid(id)) {
    return false;
  }
  const auto it = map.id_to_index.find(id);
  if (it == map.id_to_index.end()) {
    return false;
  }
  out_index = it->second;
  return true;
}

inline void register_landmark_at_index(
  GLOBAL_MAP & map, LandmarkId id, std::size_t index)
{
  if (!is_valid(id)) {
    return;
  }
  map.id_to_index[id] = index;
}

/**
 * Assign production ID, push_back, register lookup.
 * Call only when birth gates have already accepted the landmark.
 */
inline LandmarkId insert_landmark(GLOBAL_MAP & map, G_ANCHOR anchor)
{
  const LandmarkId id = allocate_landmark_id(map);
  anchor.id = id;
  map.AnchorsDATA.push_back(std::move(anchor));
  if (is_valid(id)) {
    register_landmark_at_index(map, id, map.AnchorsDATA.size() - 1);
  }
  return id;
}

/**
 * Clear landmark storage + lookup; preserve allocator high-watermark so a
 * discarded/obsolete snapshot cannot reuse IDs after clear/reset.
 */
inline void clear_landmark_storage(GLOBAL_MAP & map)
{
  map.AnchorsDATA.clear();
  map.id_to_index.clear();
}

/**
 * Map handoff: copy src into dst, then raise next_landmark_id to
 * max(old_dst, src) so commit of an older snapshot cannot regress the
 * allocator. Rebuilds lookup from AnchorsDATA.
 */
inline void assign_global_map_preserving_id_watermark(
  GLOBAL_MAP & dst, const GLOBAL_MAP & src)
{
  const std::uint64_t watermark =
    (dst.next_landmark_id > src.next_landmark_id) ? dst.next_landmark_id
                                                    : src.next_landmark_id;
  dst = src;
  dst.next_landmark_id = watermark;
  rebuild_id_to_index(dst);
}

/**
 * Remove KF observations referencing deleted_id (no remapping of survivors).
 * Order-preserving erase of AnchorsDATA[i]. Does not touch id_to_index;
 * caller must rebuild after a deletion batch (M3–M5).
 */
inline void erase_landmark_at_storage_index(
  GLOBAL_MAP & map,
  std::size_t i,
  std::vector<int> * kf_vl_out)
{
  if (i >= map.AnchorsDATA.size()) {
    return;
  }
  const LandmarkId deleted_id = map.AnchorsDATA[i].id;
  std::vector<int> kf_vl;
  for (int j = static_cast<int>(map.KeyFDATA.size()) - 1; j >= 0; --j) {
    auto & ids = map.KeyFDATA[j].Matched_landmark_ids;
    auto & uvs = map.KeyFDATA[j].UV_Matched_points;
    for (std::size_t k = 0; k < ids.size();) {
      if (ids[k] == deleted_id) {
        ids.erase(ids.begin() + static_cast<std::ptrdiff_t>(k));
        if (k < uvs.size()) {
          uvs.erase(uvs.begin() + static_cast<std::ptrdiff_t>(k));
        }
        kf_vl.push_back(j);
        continue;
      }
      ++k;
    }
  }
  map.AnchorsDATA.erase(
    map.AnchorsDATA.begin() + static_cast<std::ptrdiff_t>(i));
  if (kf_vl_out) {
    *kf_vl_out = std::move(kf_vl);
  }
}

/** Alias used by M1/M2 tests; prefer erase_landmark_at_storage_index. */
inline void erase_storage_index_preserving_order(
  GLOBAL_MAP & map,
  std::size_t i,
  std::vector<int> * kf_vl_out)
{
  erase_landmark_at_storage_index(map, i, kf_vl_out);
}

/**
 * Production culling body used by GMAP::Delete_points (testable seam).
 * Same predicate: n_tries > (min_keep-3) && n_matches < min_keep.
 */
inline void cull_underobserved_landmarks(
  GLOBAL_MAP & map,
  int delete_min_kf_matched_for_keep,
  std::vector<std::vector<int>> & kf_vg_info,
  int * n_deleted_out = nullptr)
{
  int n_deleted = 0;
  for (int64_t i = static_cast<int64_t>(map.AnchorsDATA.size()) - 1; i >= 0; --i) {
    const int n_matches = map.AnchorsDATA[static_cast<std::size_t>(i)].n_kf_matched;
    const int n_tries = map.AnchorsDATA[static_cast<std::size_t>(i)].n_tries_matchs;
    if (n_tries > (delete_min_kf_matched_for_keep - 3) &&
      n_matches < delete_min_kf_matched_for_keep)
    {
      std::vector<int> kf_vl;
      erase_landmark_at_storage_index(map, static_cast<std::size_t>(i), &kf_vl);
      kf_vg_info.push_back(std::move(kf_vl));
      ++n_deleted;
    }
  }
  if (n_deleted > 0) {
    rebuild_id_to_index(map);
  }
  if (n_deleted_out) {
    *n_deleted_out = n_deleted;
  }
}

struct IdentityInvariantReport
{
  bool ok{true};
  std::string reason;
};

/**
 * Structural invariants of one map instance (not historical non-reuse).
 * Intended for tests / opt-in diagnostics — not every production Update.
 */
inline IdentityInvariantReport validate_landmark_identity_invariants(
  const GLOBAL_MAP & map)
{
  IdentityInvariantReport r;
  std::size_t n_valid = 0;
  for (const auto & a : map.AnchorsDATA) {
    if (is_valid(a.id)) {
      ++n_valid;
    }
  }
  if (map.id_to_index.size() != n_valid) {
    r.ok = false;
    r.reason = "lookup_size_mismatch";
    return r;
  }

  std::unordered_map<LandmarkId, int, LandmarkIdHash> seen;
  for (std::size_t i = 0; i < map.AnchorsDATA.size(); ++i) {
    const LandmarkId id = map.AnchorsDATA[i].id;
    if (!is_valid(id)) {
      r.ok = false;
      r.reason = "invalid_id_at_slot";
      return r;
    }
    if (seen.count(id)) {
      r.ok = false;
      r.reason = "duplicate_id";
      return r;
    }
    seen[id] = 1;
    const auto it = map.id_to_index.find(id);
    if (it == map.id_to_index.end() || it->second != i) {
      r.ok = false;
      r.reason = "lookup_slot_mismatch";
      return r;
    }
    if (id.value >= map.next_landmark_id) {
      r.ok = false;
      r.reason = "allocator_behind_active_id";
      return r;
    }
  }
  for (const auto & kv : map.id_to_index) {
    if (kv.second >= map.AnchorsDATA.size()) {
      r.ok = false;
      r.reason = "lookup_oob";
      return r;
    }
    if (map.AnchorsDATA[kv.second].id != kv.first) {
      r.ok = false;
      r.reason = "lookup_reverse_mismatch";
      return r;
    }
  }
  // Persistent observation consistency (M3+).
  for (std::size_t ki = 0; ki < map.KeyFDATA.size(); ++ki) {
    const auto & kf = map.KeyFDATA[ki];
    if (kf.Matched_landmark_ids.size() != kf.UV_Matched_points.size()) {
      r.ok = false;
      r.reason = "obs_uv_length_mismatch";
      return r;
    }
    for (const LandmarkId lid : kf.Matched_landmark_ids) {
      if (!is_valid(lid) || !contains_landmark(map, lid)) {
        r.ok = false;
        r.reason = "obs_unresolved_id";
        return r;
      }
    }
  }
  return r;
}

}  // namespace gmap_landmark_id

#endif
