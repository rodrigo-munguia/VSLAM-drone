
#include "gmap.hpp"
#include "gmap_landmark_audit_log.hpp"
#include "gmap_map_mgmt_audit.hpp"
#include "gmap_landmark_identity.hpp"


///------------------------------------------------------------------------------
void GMAP::Delete_points(std::vector<std::vector<int>> &kf_vg_info)
{
  bool erased_any = false;

  for (int64 i = static_cast<int64>(Gmap.AnchorsDATA.size()) - 1; i >= 0; --i)
  {
      int &n_matches = Gmap.AnchorsDATA[i].n_kf_matched;
      int &n_m_tries = Gmap.AnchorsDATA[i].n_tries_matchs;

      if(n_m_tries > (PAR.Delete_min_kf_matched_for_keep_anchor-3) &&
         n_matches < PAR.Delete_min_kf_matched_for_keep_anchor )
      {
          if (gmap_landmark_audit::enabled()) {
            gmap_landmark_audit::log_delete(
              Gmap.AnchorsDATA[i].audit_id, i, n_matches, n_m_tries);
          }
          gmap_map_mgmt_audit::note_delete(
            i, Gmap.AnchorsDATA[i].audit_id, Gmap.AnchorsDATA[i].init_KF,
            n_matches, n_m_tries, static_cast<int>(Gmap.KeyFDATA.size()) - 1);

          std::vector<int> kf_vl;
          gmap_landmark_id::erase_landmark_at_storage_index(
            Gmap, static_cast<std::size_t>(i), &kf_vl);
          kf_vg_info.push_back(std::move(kf_vl));
          erased_any = true;
          if(PAR.Stats)total_delete_anchors++;
      }
  }

  if (erased_any) {
    gmap_landmark_id::rebuild_id_to_index(Gmap);
  }
}
