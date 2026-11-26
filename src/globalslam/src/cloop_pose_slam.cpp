#include "cloop.hpp"
#include "gmap_graph_correction.hpp"

#include <iostream>

void CLOOP::Pose_SLAM(
  arma::vec::fixed<3> & pos,
  std::vector<arma::vec::fixed<3>> & delta_kf_pos,
  std::vector<int> & idx_kf_opt)
{
  const auto cfg = gmap_graph_correction::pose_slam_config_from_par(PAR);
  gmap_graph_correction::run_pose_slam(Gmap, pos, cfg, delta_kf_pos, idx_kf_opt);

  if (PAR.Stats && !store.cloop_stats.empty()) {
    for (const auto & dt_kf : delta_kf_pos) {
      try {
        store.cloop_stats.back().error_correction = arma::norm(dt_kf);
      } catch (const std::exception & e) {
        std::cout << "Caught exception: " << e.what() << std::endl;
      }
    }
  }
}
