#include "gmap_post_loop_refine.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <opencv2/features2d.hpp>

#include "gmap_landmark_identity.hpp"

namespace gmap_post_loop_refine
{
namespace
{

constexpr double kMatchRatio = 0.85;

void match_features4(
  const cv::Mat & query, const cv::Mat & target,
  std::vector<cv::DMatch> & goodMatches)
{
  std::vector<std::vector<cv::DMatch>> matches;
  cv::FlannBasedMatcher matcher =
    cv::FlannBasedMatcher(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
  matcher.knnMatch(query, target, matches, 2);
  for (unsigned int i = 0; i < matches.size(); ++i) {
    if (!matches[i].empty() && matches[i].size() > 1) {
      if (matches[i][0].distance < matches[i][1].distance * kMatchRatio) {
        goodMatches.push_back(matches[i][0]);
      }
    }
  }
}

}  // namespace

std::vector<int> get_visually_linked_kf(
  const GLOBAL_MAP & map, int idx_kf, int min_strength)
{
  std::vector<int> idx_vl_kf;
  if (idx_kf < 0 || map.Vgraph.empty() ||
    static_cast<std::size_t>(idx_kf) >= map.KeyFDATA.size())
  {
    return idx_vl_kf;
  }
  const int n_cols = static_cast<int>(map.Vgraph.n_cols);
  for (int i = 0; i < n_cols; ++i) {
    const int n_vlp =
      static_cast<int>(map.Vgraph.at(idx_kf, i));
    if (n_vlp > min_strength) {
      idx_vl_kf.push_back(i);
    }
  }
  return idx_vl_kf;
}

void match_list_of_points_into_kf(
  GLOBAL_MAP & map,
  const CAM & cam,
  const MatchConfig & config,
  const std::vector<int64_t> & idx_points,
  int idx_KF,
  std::vector<int64_t> & idx_matched_points)
{
  idx_matched_points.clear();
  if (idx_KF < 0 ||
    static_cast<std::size_t>(idx_KF) >= map.KeyFDATA.size())
  {
    return;
  }

  std::vector<cv::KeyPoint> keyPoints_A;
  cv::Mat Descriptors_A;
  std::vector<cv::Point2d> Predicted_projections;
  std::vector<int64_t> Idx_candidate_A;

  const arma::mat::fixed<3, 3> Rn2c = map.KeyFDATA[static_cast<std::size_t>(idx_KF)].Rn2c;
  const arma::vec::fixed<3> t_c2n = map.KeyFDATA[static_cast<std::size_t>(idx_KF)].t_c2n;

  for (std::size_t i = 0; i < idx_points.size(); ++i) {
    const int64_t idx_pt = idx_points[i];
    if (idx_pt < 0 ||
      static_cast<std::size_t>(idx_pt) >= map.AnchorsDATA.size())
    {
      continue;
    }
    const arma::vec::fixed<3> pxyz =
      map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].AnchorState;
    const arma::vec::fixed<3> pc = Rn2c * (pxyz - t_c2n);
    arma::mat::fixed<2, 3> duv_dPc;
    const cv::Point2d uvd = Projection_model(pc, 0, false, cam, duv_dPc);
    const int sm = config.vm_max_inov_error;
    if ((uvd.x > -sm) && (uvd.x < config.img_cols + sm) &&
      (uvd.y > -sm) && (uvd.y < config.img_rows + sm))
    {
      Idx_candidate_A.push_back(idx_pt);
      Descriptors_A.push_back(
        map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].Descriptor);
      keyPoints_A.push_back(
        map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].Keypoint);
      Predicted_projections.push_back(uvd);
    }
  }

  if (Predicted_projections.empty()) {
    return;
  }
  if (map.KeyFDATA[static_cast<std::size_t>(idx_KF)].Descriptors_all.empty()) {
    return;
  }

  std::vector<cv::DMatch> matches;
  match_features4(
    Descriptors_A,
    map.KeyFDATA[static_cast<std::size_t>(idx_KF)].Descriptors_all, matches);

  const double max_inov_error = static_cast<double>(config.vm_max_inov_error);
  for (unsigned int k = 0; k < matches.size(); ++k) {
    const int Descriptor_idx_A = matches[k].queryIdx;
    const int Descriptor_idx_KF = matches[k].trainIdx;
    if (Descriptor_idx_A < 0 ||
      static_cast<std::size_t>(Descriptor_idx_A) >= Predicted_projections.size() ||
      Descriptor_idx_KF < 0 ||
      static_cast<std::size_t>(Descriptor_idx_KF) >=
        map.KeyFDATA[static_cast<std::size_t>(idx_KF)].keyPoints_all.size())
    {
      continue;
    }
    const cv::Point2d pt_m =
      map.KeyFDATA[static_cast<std::size_t>(idx_KF)]
        .keyPoints_all[static_cast<std::size_t>(Descriptor_idx_KF)]
        .pt;
    const cv::Point2d pt_p =
      Predicted_projections[static_cast<std::size_t>(Descriptor_idx_A)];
    const double dis =
      std::sqrt(std::pow(pt_p.x - pt_m.x, 2) + std::pow(pt_p.y - pt_m.y, 2));
    if (dis < max_inov_error) {
      const int64_t Idx_matched_A =
        Idx_candidate_A[static_cast<std::size_t>(Descriptor_idx_A)];
      idx_matched_points.push_back(Idx_matched_A);
      map.KeyFDATA[static_cast<std::size_t>(idx_KF)].Matched_landmark_ids.push_back(
        map.AnchorsDATA[static_cast<std::size_t>(Idx_matched_A)].id);
      map.KeyFDATA[static_cast<std::size_t>(idx_KF)].UV_Matched_points.push_back(
        pt_m);
      map.AnchorsDATA[static_cast<std::size_t>(Idx_matched_A)].n_kf_matched++;
    }
  }
}

void update_visibility_graph_with_matched_points(
  GLOBAL_MAP & map,
  const std::vector<int64_t> & idx_points,
  int idx_KF)
{
  if (map.Vgraph.empty() || idx_KF < 0 ||
    static_cast<std::size_t>(idx_KF) >= map.KeyFDATA.size())
  {
    return;
  }
  for (std::size_t i = 0; i < idx_points.size(); ++i) {
    const int64_t idx_pt = idx_points[i];
    if (idx_pt < 0 ||
      static_cast<std::size_t>(idx_pt) >= map.AnchorsDATA.size())
    {
      continue;
    }
    const LandmarkId lid =
      map.AnchorsDATA[static_cast<std::size_t>(idx_pt)].id;
    for (int j = 0; j < static_cast<int>(map.KeyFDATA.size()); ++j) {
      if (j == idx_KF) {
        continue;
      }
      for (std::size_t k = 0;
           k < map.KeyFDATA[static_cast<std::size_t>(j)]
                 .Matched_landmark_ids.size();
           ++k)
      {
        if (map.KeyFDATA[static_cast<std::size_t>(j)].Matched_landmark_ids[k] ==
          lid)
        {
          map.Vgraph.at(idx_KF, j)++;
          map.Vgraph.at(j, idx_KF)++;
          break;
        }
      }
    }
  }
}

void run_post_loop_matching(
  GLOBAL_MAP & map,
  int idx_closing,
  int idx_matched_historical,
  const CAM & cam,
  const MatchConfig & config,
  std::vector<int> & idx_vl_new_kf,
  std::vector<int> & idx_vl_old_kf)
{
  idx_vl_new_kf = get_visually_linked_kf(
    map, idx_closing, config.min_vgraph_strength);
  idx_vl_new_kf.push_back(idx_closing);
  std::sort(idx_vl_new_kf.begin(), idx_vl_new_kf.end());
  idx_vl_new_kf.erase(
    std::unique(idx_vl_new_kf.begin(), idx_vl_new_kf.end()), idx_vl_new_kf.end());

  idx_vl_old_kf = get_visually_linked_kf(
    map, idx_matched_historical, config.min_vgraph_strength);
  idx_vl_old_kf.push_back(idx_matched_historical);
  std::sort(idx_vl_old_kf.begin(), idx_vl_old_kf.end());
  idx_vl_old_kf.erase(
    std::unique(idx_vl_old_kf.begin(), idx_vl_old_kf.end()), idx_vl_old_kf.end());

  std::vector<int64_t> idx_old_points;
  for (int idx_kf : idx_vl_old_kf) {
    if (idx_kf < 0 ||
      static_cast<std::size_t>(idx_kf) >= map.KeyFDATA.size())
    {
      continue;
    }
    for (std::size_t j = 0;
         j < map.KeyFDATA[static_cast<std::size_t>(idx_kf)]
               .Matched_landmark_ids.size();
         ++j)
    {
      const LandmarkId lid =
        map.KeyFDATA[static_cast<std::size_t>(idx_kf)].Matched_landmark_ids[j];
      std::size_t idx_pt = 0;
      if (!gmap_landmark_id::resolve_landmark_index(map, lid, idx_pt)) {
        continue;
      }
      const int64_t idx = static_cast<int64_t>(idx_pt);
      if (std::find(idx_old_points.begin(), idx_old_points.end(), idx) ==
        idx_old_points.end())
      {
        idx_old_points.push_back(idx);
      }
    }
  }

  for (int idx_KF : idx_vl_new_kf) {
    std::vector<int64_t> idx_matched_points;
    match_list_of_points_into_kf(
      map, cam, config, idx_old_points, idx_KF, idx_matched_points);
    update_visibility_graph_with_matched_points(
      map, idx_matched_points, idx_KF);
  }

  std::vector<int64_t> idx_new_points;
  for (int idx_kf : idx_vl_new_kf) {
    if (idx_kf < 0 ||
      static_cast<std::size_t>(idx_kf) >= map.KeyFDATA.size())
    {
      continue;
    }
    for (std::size_t j = 0;
         j < map.KeyFDATA[static_cast<std::size_t>(idx_kf)]
               .Matched_landmark_ids.size();
         ++j)
    {
      const LandmarkId lid =
        map.KeyFDATA[static_cast<std::size_t>(idx_kf)].Matched_landmark_ids[j];
      std::size_t idx_pt = 0;
      if (!gmap_landmark_id::resolve_landmark_index(map, lid, idx_pt)) {
        continue;
      }
      const int64_t idx = static_cast<int64_t>(idx_pt);
      if (std::find(idx_new_points.begin(), idx_new_points.end(), idx) ==
        idx_new_points.end())
      {
        idx_new_points.push_back(idx);
      }
    }
  }

  for (int idx_KF : idx_vl_old_kf) {
    std::vector<int64_t> idx_matched_points;
    match_list_of_points_into_kf(
      map, cam, config, idx_new_points, idx_KF, idx_matched_points);
    update_visibility_graph_with_matched_points(
      map, idx_matched_points, idx_KF);
  }
}

}  // namespace gmap_post_loop_refine
