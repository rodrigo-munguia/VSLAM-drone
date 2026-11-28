/**
 * M7.7.2: bounded FIFO of newly admitted authoritative KF closing-query
 * eligibility records. Detection-only metadata — no map payloads.
 */
#ifndef GMAP_ADMITTED_CLOOP_QUEUE_HPP
#define GMAP_ADMITTED_CLOOP_QUEUE_HPP

#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <unordered_set>

#include <armadillo>

namespace gmap_admitted_cloop
{

/** Capacity sized for ~MAP-KF cadence with slow CLOOP; drop oldest on overflow. */
inline constexpr std::size_t kEligibilityCapacity = 32;

struct Eligibility
{
  uint64_t keyframe_id{0};
  uint64_t map_lifetime{0};
  /** Bumped on confirmed Local sync apply; obsolete queries rejected. */
  uint64_t closing_query_generation{0};
  arma::vec::fixed<3> p_L_original{};
  bool p_L_original_valid{false};
};

struct QueueStats
{
  uint64_t admitted_eligible{0};
  uint64_t consumed{0};
  uint64_t overflow_dropped{0};
  uint64_t obsolete_discarded{0};
  uint64_t duplicate_skipped{0};
};

class EligibilityQueue
{
public:
  void clear()
  {
    std::lock_guard<std::mutex> lock(mu_);
    q_.clear();
    enqueued_ids_.clear();
  }

  void invalidate_all_as_obsolete()
  {
    std::lock_guard<std::mutex> lock(mu_);
    stats_.obsolete_discarded += q_.size();
    q_.clear();
    enqueued_ids_.clear();
  }

  /**
   * Push after successful Open admit. Exactly-once per keyframe_id while
   * present in the queue. On overflow, drop oldest unprocessed entry.
   */
  bool push(Eligibility e)
  {
    std::lock_guard<std::mutex> lock(mu_);
    if (e.keyframe_id == 0) {
      return false;
    }
    if (enqueued_ids_.count(e.keyframe_id) != 0) {
      ++stats_.duplicate_skipped;
      return false;
    }
    while (q_.size() >= kEligibilityCapacity) {
      const Eligibility old = q_.front();
      q_.pop_front();
      enqueued_ids_.erase(old.keyframe_id);
      ++stats_.overflow_dropped;
    }
    enqueued_ids_.insert(e.keyframe_id);
    q_.push_back(std::move(e));
    ++stats_.admitted_eligible;
    return true;
  }

  std::optional<Eligibility> try_pop(uint64_t current_map_lifetime,
    uint64_t current_query_generation)
  {
    std::lock_guard<std::mutex> lock(mu_);
    while (!q_.empty()) {
      Eligibility e = q_.front();
      q_.pop_front();
      enqueued_ids_.erase(e.keyframe_id);
      if (e.map_lifetime != current_map_lifetime ||
        e.closing_query_generation != current_query_generation)
      {
        ++stats_.obsolete_discarded;
        continue;
      }
      ++stats_.consumed;
      return e;
    }
    return std::nullopt;
  }

  std::size_t size() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    return q_.size();
  }

  QueueStats stats() const
  {
    std::lock_guard<std::mutex> lock(mu_);
    return stats_;
  }

private:
  mutable std::mutex mu_;
  std::deque<Eligibility> q_;
  std::unordered_set<uint64_t> enqueued_ids_;
  QueueStats stats_{};
};

}  // namespace gmap_admitted_cloop

#endif
