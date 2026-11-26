#include "cloop.hpp"
#include "gmap_landmark_identity.hpp"

// Inactive legacy fusion (call site commented). Minimal M3 compile compatibility only —
// known algorithm defects are intentionally NOT repaired here.


void CLOOP::Fuse_map_points(int &idx_kf_matched)
{
    int idx_new_kf = Gmap.KeyFDATA.size() -1;
    int idx_matched_kf = idx_kf_matched;    

    std::vector<int> idx_vl_new_kf = Get_n_visually_linked_kf(idx_new_kf ,10);
    idx_vl_new_kf.push_back(idx_new_kf);
    sort(idx_vl_new_kf.begin(), idx_vl_new_kf.end());

    std::vector<int> idx_vl_matched_kf = Get_n_visually_linked_kf(idx_matched_kf,10 );
    idx_vl_matched_kf.push_back(idx_matched_kf);
    sort(idx_vl_matched_kf.begin(), idx_vl_matched_kf.end());

    auto collect_storage_indices = [&](const std::vector<int> & idx_vl) {
      std::vector<int64> out;
      for (int idx_kf : idx_vl) {
        for (const LandmarkId lid : Gmap.KeyFDATA[idx_kf].Matched_landmark_ids) {
          std::size_t idx_pt = 0;
          if (!gmap_landmark_id::resolve_landmark_index(Gmap, lid, idx_pt)) {
            continue;
          }
          const int64 idx = static_cast<int64>(idx_pt);
          if (std::find(out.begin(), out.end(), idx) == out.end()) {
            out.push_back(idx);
          }
        }
      }
      return out;
    };

    std::vector<int64> idx_old_points = collect_storage_indices(idx_vl_matched_kf);
    std::vector<int64> idx_new_points = collect_storage_indices(idx_vl_new_kf);

    std::vector<int64> idx_matched_new_pts;
    std::vector<int64> idx_matched_old_pts;
    Match_a_points_with_b_points(idx_new_points,idx_old_points,idx_matched_new_pts,idx_matched_old_pts);

    std::vector<int> idx_vl_kf_A;
    std::vector<int> idx_vl_kf_B;
    
    for (int i = 0; i < idx_matched_new_pts.size(); i ++)
    {
        int64 idx_new_pt = idx_matched_new_pts[i];
        int64 idx_old_pt = idx_matched_old_pts[i];
        if (idx_new_pt < 0 || idx_old_pt < 0 ||
          static_cast<std::size_t>(idx_new_pt) >= Gmap.AnchorsDATA.size() ||
          static_cast<std::size_t>(idx_old_pt) >= Gmap.AnchorsDATA.size())
        {
          continue;
        }
        const LandmarkId id_new = Gmap.AnchorsDATA[idx_new_pt].id;
        const LandmarkId id_old = Gmap.AnchorsDATA[idx_old_pt].id;
        
        Gmap.AnchorsDATA[idx_old_pt].n_kf_matched += Gmap.AnchorsDATA[idx_new_pt].n_kf_matched; 
        // Legacy buggy erase retained (inactive path).
        Gmap.AnchorsDATA.erase(Gmap.AnchorsDATA.begin()+i);

        std::vector<int64> idx_kf_old_pt;
        for (int j = 0; j < idx_vl_matched_kf[idx_vl_matched_kf.size()-1] ; j++)
        {
            for( int k = 0; k < Gmap.KeyFDATA[j].Matched_landmark_ids.size(); k++)
            {
                if(Gmap.KeyFDATA[j].Matched_landmark_ids[k] == id_old)
                {
                   idx_kf_old_pt.push_back(j); 
                }
            }    
        }         
        
        for(int j = 0; j < idx_vl_new_kf.size() ; j++)
        {
            int idx_kf = idx_vl_new_kf[j];                        
            for( int k = 0; k < Gmap.KeyFDATA[idx_kf].Matched_landmark_ids.size(); k++)
            {
                if(Gmap.KeyFDATA[idx_kf].Matched_landmark_ids[k] == id_new)
                {
                     Gmap.KeyFDATA[idx_kf].Matched_landmark_ids[k] = id_old;
                     for(int l = 0; l < idx_kf_old_pt.size();l++)
                     {   
                         int idx_kf_B = idx_kf_old_pt[l];
                         idx_vl_kf_A.push_back(idx_kf);
                         idx_vl_kf_B.push_back(idx_kf_B);
                     }   
                }
            }            
        }
       
    }
    
    Update_Visibility_Graph_A_B_links(idx_vl_kf_A,idx_vl_kf_B);
    gmap_landmark_id::rebuild_id_to_index(Gmap);

    int q = 10;    

}
