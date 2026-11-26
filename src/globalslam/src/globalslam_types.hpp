#ifndef LOCALSLAM_TYPES_H
#define LOCALSLAM_TYPES_H

#include <armadillo>
#include <cstdint>
#include <cstddef>
#include <unordered_map>
#include "opencv2/opencv.hpp"
#include "../../common/keyframe_types.hpp"



using namespace std;


// KEYFRAME is defined in common/keyframe_types.hpp (shared with localslam).

struct KeyFramesData
 {
     int KeyF_idx;  // index of Keyframes associated with the Mapfeature
     bool matched;
     cv::Point2f PredictedPoint; // Predicted pixel position of the feature
     cv::Point2f MatchedPoint;  // Matched (measured) position of the feature
 };

/**
 * Production stable landmark identity (GSLAM-MAP-ARCH-M1/M2).
 * Explicit construction only — not convertible to/from storage indices.
 * Convention: value 0 = invalid/unassigned; valid IDs allocate from 1.
 */
struct LandmarkId
{
  std::uint64_t value{0};
};

struct LandmarkIdHash
{
  std::size_t operator()(LandmarkId id) const noexcept
  {
    return static_cast<std::size_t>(id.value);
  }
};

inline bool operator==(LandmarkId a, LandmarkId b)
{
  return a.value == b.value;
}
inline bool operator!=(LandmarkId a, LandmarkId b) { return !(a == b); }

struct G_ANCHOR
{       
    string feat_type; // Feature tupe: euclidean (XYZ): ID, etc
    arma::vec::fixed<3> AnchorState; //
    cv::Mat Descriptor;  //  Keypoint descriptor
    std::vector<KeyFramesData> iKeyFrame; // vector for storing data for the i-KeyFrame
    cv::KeyPoint Keypoint; // current feature Keypoint (include actual matched pixel position of the feature)
    int init_KF; //  index of the KeyFrame  where the Feature-anchor was initialized
    int n_kf_matched; // number of kf where the map point has been matched
    int n_tries_matchs; // number of times that the point has been considered to be matched
    // Production stable identity (independent of storage index). Default invalid.
    LandmarkId id{};
    // Audit-only persistent identity (GSLAM_LANDMARK_AUDIT_DIR). Unused in production.
    int64_t audit_id{-1};
};


// Global map structure
struct GLOBAL_MAP
{
    std::vector<KEYFRAME> KeyFDATA;  // Structure for storing KeyFrames
    std::vector<G_ANCHOR> AnchorsDATA; // Structure for storing map points (anchors)
    /// LandmarkId → current AnchorsDATA slot (dense vector index).
    std::unordered_map<LandmarkId, std::size_t, LandmarkIdHash> id_to_index;
    /// Next ID to allocate (monotonic). Valid IDs are in [1, next_landmark_id).
    std::uint64_t next_landmark_id{1};
    arma::mat Vgraph; // visibility graph 
    //int idx_Fixed_kf_ref_BA; // index of the i-(oldest) keyFrame that will be considered "fixed" for BA
    int idx_ref_pose_slam;
    
    
};

struct STOREG
{      
    int n_init_anchors;    
    int n_delete_anchors;
    int n_kf;       
    std::pair<std::vector<double>,std::vector<double>> n_anchors_per_step;
    std::pair<std::vector<double>,std::vector<double>> time_per_step;
    std::pair<std::vector<double>,std::vector<double>> n_kf_per_step;      
    double total_comp_time;  // total computation time
}; 

struct CLOOP_Stat
{   
    double step;
    double reprojection_error;
    double error_correction;
    double distance_traveled; // distance traveled from the initial position / last closing loop point
    int n_loop_closing_intents; // number of intents (loop detections) before close loop is accepted
};

struct STOREC
{
    std::vector<CLOOP_Stat> cloop_stats;
    std::pair<std::vector<double>,std::vector<double>> time_search_per_step;
    double total_comp_time;  // total computation time

};    




#endif
