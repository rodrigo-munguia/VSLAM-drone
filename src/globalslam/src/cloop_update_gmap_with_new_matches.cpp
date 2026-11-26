#include "cloop.hpp"
#include "gmap_landmark_identity.hpp"



void CLOOP::Update_gmap_with_new_matches(int &idx_kf_matched, std::vector<int> &idx_vl_new_kf, std::vector<int> &idx_vl_old_kf)
{

    int idx_new_kf = Gmap.KeyFDATA.size() -1;
    int idx_matched_kf = idx_kf_matched;    

    idx_vl_new_kf = Get_n_visually_linked_kf(idx_new_kf ,10);
    idx_vl_new_kf.push_back(idx_new_kf);
    sort(idx_vl_new_kf.begin(), idx_vl_new_kf.end());

    idx_vl_old_kf = Get_n_visually_linked_kf(idx_matched_kf,10 );
    idx_vl_old_kf.push_back(idx_matched_kf);
    sort(idx_vl_old_kf.begin(), idx_vl_old_kf.end());

    // Temporary storage indices for Match_list_of_points_into_KF (map stable).
    std::vector<int64> idx_old_points;
    for(int i = 0; i < idx_vl_old_kf.size(); i++)
    {
        int idx_kf = idx_vl_old_kf[i];

        for(int j = 0; j < Gmap.KeyFDATA[idx_kf].Matched_landmark_ids.size(); j++)
        {
            const LandmarkId lid = Gmap.KeyFDATA[idx_kf].Matched_landmark_ids[j];
            std::size_t idx_pt = 0;
            if (!gmap_landmark_id::resolve_landmark_index(Gmap, lid, idx_pt)) {
              continue;
            }
            const int64 idx = static_cast<int64>(idx_pt);
            if ( std::find(idx_old_points.begin(), idx_old_points.end(), idx) == idx_old_points.end() )
            {
                idx_old_points.push_back(idx);
            }
        }     
    }
    
    for(int i = 0; i < idx_vl_new_kf.size(); i++)
    {
        int idx_KF = idx_vl_new_kf[i];
        std::vector<int64> idx_matched_points; 
        Match_list_of_points_into_KF(idx_old_points, idx_KF,idx_matched_points);
        Update_Visibility_Graph_with_matched_points_into_KF(idx_matched_points, idx_KF);        

    }

    //-----------------------------------------------------------------------------
    std::vector<int64> idx_new_points;
    for(int i = 0; i < idx_vl_new_kf.size(); i++)
    {
        int idx_kf = idx_vl_new_kf[i];

        for(int j = 0; j < Gmap.KeyFDATA[idx_kf].Matched_landmark_ids.size(); j++)
        {
            const LandmarkId lid = Gmap.KeyFDATA[idx_kf].Matched_landmark_ids[j];
            std::size_t idx_pt = 0;
            if (!gmap_landmark_id::resolve_landmark_index(Gmap, lid, idx_pt)) {
              continue;
            }
            const int64 idx = static_cast<int64>(idx_pt);
            if ( std::find(idx_new_points.begin(), idx_new_points.end(), idx) == idx_new_points.end() )
            {
                idx_new_points.push_back(idx);
            }
        }     
    }

    for(int i = 0; i < idx_vl_old_kf.size(); i++)
    {
        int idx_KF = idx_vl_old_kf[i];
        std::vector<int64> idx_matched_points; 
        Match_list_of_points_into_KF(idx_new_points, idx_KF,idx_matched_points);
        Update_Visibility_Graph_with_matched_points_into_KF(idx_matched_points, idx_KF);        

    }



}
