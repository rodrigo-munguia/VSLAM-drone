#include "cloop.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_visibility_graph_exact.hpp"
#include "gmap_vgraph_runtime_diag.hpp"

//------------------------------------------------------------------------------------------------------
// LC-POLICY-A1.1: exact rebuild (legacy incremental increments retired).
void CLOOP::Update_Visibility_Graph_with_matched_points_into_KF(std::vector<int64> &idx_points, int idx_KF)
{
  (void)idx_points;
  (void)idx_KF;
  gmap_vgraph_runtime_diag::rebuild_with_log(
    Gmap, "cloop_matched_points",
    [](GLOBAL_MAP & m) {
      return gmap_visibility_graph_exact::rebuild_exact_visibility_graph(m);
    });
}




//---------------------------------------------------------------------------------
// Inactive fuse helper (callers commented out). Retained signature; no-op-safe.
void CLOOP::Update_Visibility_Graph_A_B_links(std::vector<int> &idx_vl_kf_A,std::vector<int> &idx_vl_kf_B)
{
  // Legacy fuse-era pairwise ++ is incompatible with exact semantics.
  // Prefer rebuild after any observation mutation that would have used this.
  (void)idx_vl_kf_A;
  (void)idx_vl_kf_B;
  gmap_vgraph_runtime_diag::rebuild_with_log(
    Gmap, "cloop_ab_links_inactive",
    [](GLOBAL_MAP & m) {
      return gmap_visibility_graph_exact::rebuild_exact_visibility_graph(m);
    });
}

//------------------------------------------------------------------------------------------------
std::vector<int> CLOOP::Get_n_visually_linked_kf(int idx_kf , int min_strenght)
{
     std::vector<int> idx_vl_kf;
    
    bool empty = true;

    int n_cols = Gmap.Vgraph.n_cols;
    
    for (int i = 0 ; i < n_cols ; i++)
    {
      int n_vlp = (int)Gmap.Vgraph.at(idx_kf,i);
      if(n_vlp > min_strenght)
      {
        idx_vl_kf.push_back(i);
        empty = false;
      }

    }
    
    if(empty == false)
    {
      return idx_vl_kf;
    }
    else
    {
      return std::vector<int>();
    }  


}
//------------------------------------------------------------------------------------------------
std::vector<int> CLOOP::Get_n_not_visually_linked_kf(int idx_kf, int idx_kf_sup_limit )
{
    std::vector<int> idx_vl_kf;   
    int n_added = 0;
    
    //for (int i = Gmap.idx_ref_pose_slam ; i < idx_kf_sup_limit ; i++)
    for (int i = 0 ; i < idx_kf_sup_limit ; i++)
    {
      int n_vlp = (int)Gmap.Vgraph.at(idx_kf,i);
      if(n_vlp == 0)
      {
        idx_vl_kf.push_back(i);
        n_added++;
      }
      if(n_vlp > 0)
      {
          break;
      } 
    }
    
    if(n_added > 0)
    {
      return idx_vl_kf;
    }
    else
    {
      return std::vector<int>();
    }  
}
//----------------------------------------------------------------
void CLOOP::Update_Visibility_Graph_with_new_KF(int idx_KF)
{
  (void)idx_KF;
  gmap_vgraph_runtime_diag::rebuild_with_log(
    Gmap, "cloop_new_kf",
    [](GLOBAL_MAP & m) {
      return gmap_visibility_graph_exact::rebuild_exact_visibility_graph(m);
    });
}
