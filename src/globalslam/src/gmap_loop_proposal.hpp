/**
 * GSLAM-MAP-ARCH-M7.4: production loop proposal + outcome contract.
 * Owned payload; no raw pointers / storage indices / OpenCV Mats.
 */
#ifndef GMAP_LOOP_PROPOSAL_HPP
#define GMAP_LOOP_PROPOSAL_HPP

#include <cstdint>
#include <string>
#include <vector>

#include <armadillo>
#include <opencv2/core.hpp>

#include "../../common/landmark_id.hpp"
#include "../../common/keyframe_types.hpp"

namespace gmap_loop
{

enum class Kind : uint8_t
{
  Visual = 0,
  Tag = 1,
};

enum class ProposalState : uint8_t
{
  Submitted = 0,
  DeferredPendingEndpoint = 1,
  Processing = 2,
  Rejected = 3,
  CorrectionFailed = 4,
  GlobalCommitted = 5,
  LocalSyncPending = 6,
  LocalSyncApplied = 7,
  LocalSyncNotApplied = 8,
  LocalSyncUnknown = 9,
};

inline const char * state_name(ProposalState s)
{
  switch (s) {
    case ProposalState::Submitted: return "submitted";
    case ProposalState::DeferredPendingEndpoint: return "deferred_pending_endpoint";
    case ProposalState::Processing: return "processing";
    case ProposalState::Rejected: return "rejected";
    case ProposalState::CorrectionFailed: return "correction_failed";
    case ProposalState::GlobalCommitted: return "global_committed";
    case ProposalState::LocalSyncPending: return "local_sync_pending";
    case ProposalState::LocalSyncApplied: return "local_sync_applied";
    case ProposalState::LocalSyncNotApplied: return "local_sync_not_applied";
    case ProposalState::LocalSyncUnknown: return "local_sync_unknown";
  }
  return "unknown";
}

struct LoopProposal
{
  uint64_t proposal_id{0};
  uint64_t map_lifetime{0};
  uint64_t correction_epoch{0};

  uint64_t closing_keyframe_id{0};
  uint64_t pose_graph_ref_keyframe_id{0};
  /** -1 when absent (e.g. tag loop). */
  int64_t matched_historical_keyframe_id{-1};

  arma::vec::fixed<3> pnp_C_nav{};
  arma::vec::fixed<3> p_L_original{};
  /** Coherent: t_ref_at_verification − pnp_C_nav (same verification epoch). */
  arma::vec::fixed<3> t_loop{};
  arma::vec::fixed<3> t_ref_at_verification{};

  std::vector<LandmarkId> inlier_landmark_ids;
  std::vector<cv::Point2d> inlier_uvs;  // distorted image UVs (pairing preserved)
  std::vector<arma::vec::fixed<3>> inlier_xyz_at_verification;

  Kind kind{Kind::Visual};
  std::string source_label{"visual_loop"};

  /** Closing KF snapshot for deferred admission / same-KF identity checks. */
  KEYFRAME closing_kf{};
};

struct ProposalOutcome
{
  uint64_t proposal_id{0};
  ProposalState state{ProposalState::Rejected};
  std::string reason;
  arma::vec::fixed<3> delta_L{};
  arma::vec::fixed<3> p_G_final_closing{};
  bool match_ba_refine_ran{false};
};

/** Geometry comparison tolerances (M7.2 design). */
struct RevalidationTolerances
{
  double common_translation_tol_m{1e-3};
  double position_tol_m{1e-3};
};

enum class RevalidationDecision : uint8_t
{
  ReuseCoherent = 0,
  CommonTranslationReuse = 1,
  RequireRepnp = 2,
  Reject = 3,
};

inline const char * revalidation_name(RevalidationDecision d)
{
  switch (d) {
    case RevalidationDecision::ReuseCoherent: return "reuse_coherent";
    case RevalidationDecision::CommonTranslationReuse: return "common_translation";
    case RevalidationDecision::RequireRepnp: return "require_repnp";
    case RevalidationDecision::Reject: return "reject";
  }
  return "unknown";
}

}  // namespace gmap_loop

#endif
