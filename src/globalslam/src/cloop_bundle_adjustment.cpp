#include "cloop.hpp"
#include "gmap_graph_correction.hpp"

void CLOOP::Update_gmap_with_BA(
  std::vector<int> & idx_vl_new_kf, std::vector<int> & idx_vl_old_kf)
{
  const auto cfg = gmap_graph_correction::post_loop_ba_config_from_par(PAR);
  gmap_graph_correction::run_post_loop_ba(
    Gmap, idx_vl_new_kf, idx_vl_old_kf, cam_parameters, cfg);
}
