#include "gmap.hpp"
#include "gmap_landmark_identity.hpp"
#include "gmap_visibility_graph_exact.hpp"
#include "gmap_vgraph_runtime_diag.hpp"

//------------------------------------------------------------------------------------------------
std::vector<int> GMAP::Get_n_oldest_visually_linked_kf(int idx_kf,int n_oldest, int min_v_strength )
{

  std::vector<int> idx_vl_kf;
    
   
    int n_added = 0;
    
    for (int i = 0 ; i < idx_kf ; i++)
    {
      int n_vlp = (int)Gmap.Vgraph.at(idx_kf,i);
      if(n_vlp > min_v_strength)
      {
        idx_vl_kf.push_back(i);
        n_added++;
      }
      if(n_added > n_oldest)
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


//------------------------------------------------------------------
// LC-POLICY-A1.1: exact rebuild replaces incremental pair decrements.
void GMAP::Update_Visibility_Graph_with_deleted_points(std::vector<std::vector<int>> &kf_vg_info)
{
  (void)kf_vg_info;
  gmap_vgraph_runtime_diag::rebuild_with_log(
    Gmap, "deleted_points",
    [](GLOBAL_MAP & m) {
      return gmap_visibility_graph_exact::rebuild_exact_visibility_graph(m);
    });
}


//------------------------------------------------------------------
std::vector<int> GMAP::Get_idx_visually_linked_to_KF(int idx_KF)
{
    std::vector<int> idx_vl_kf;
    
    bool empty = true;
    int n_cols = Gmap.Vgraph.n_cols;
    
    for (int i = 0 ; i < n_cols ; i++)
    {
      int n_vlp = (int)Gmap.Vgraph.at(idx_KF,i);
      if(n_vlp > 0)
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



//----------------------------------------------------------------
// LC-POLICY-A1.1: exact rebuild from authoritative observations.
void GMAP::Update_Visibility_Graph_with_new_KF()
{
  gmap_vgraph_runtime_diag::rebuild_with_log(
    Gmap, "new_kf_association",
    [](GLOBAL_MAP & m) {
      return gmap_visibility_graph_exact::rebuild_exact_visibility_graph(m);
    });
}

//--------------------------------------------------------------------------------
// LC-POLICY-A1.1: exact rebuild (birth / retro-match incremental path retired).
void GMAP::Update_Visibility_Graph_with_matches_of_new_points(std::vector<int64> &idx_new_points, std::vector<int> &idx_kf_m, std::vector<int64> &idx_pt_m )
{
  (void)idx_new_points;
  (void)idx_kf_m;
  (void)idx_pt_m;
  gmap_vgraph_runtime_diag::rebuild_with_log(
    Gmap, "new_points_matches",
    [](GLOBAL_MAP & m) {
      return gmap_visibility_graph_exact::rebuild_exact_visibility_graph(m);
    });
}


//-------------------------------------------------------------------------------------
void  GMAP::Check_visibiliy_graph(int &idx_direct_link, int &idx_indirect_link )
{

    idx_direct_link = 0;
  if (Gmap.Vgraph.n_cols < 4)
  {
    idx_indirect_link = 0;
  }
  else
  { 
    //Vgraph.print();
    bool flag_zero = false;
    //int idx_i;
    for(int i = Gmap.Vgraph.n_cols-2; i >= 0 ;  i-- )
    {      
      int vlink =  Gmap.Vgraph(Gmap.Vgraph.n_rows-1,i);    
      if (vlink == 0)
      {
       flag_zero = true;
       idx_direct_link = i + 1;
       break;        
      }    
    } 
    if (flag_zero == false)
    {
      idx_indirect_link = 0;
    }
    else
    { 
      bool flag_zero2 = false;
      //int idx_j;
      for(int j = idx_direct_link-1; j >= 0 ; j--)
      {
        int vlink =  Gmap.Vgraph(idx_direct_link,j);
        if (vlink == 0)
          {
          flag_zero2 = true;
          idx_indirect_link  = j + 1;
          break;        
          }            
      }
      if (flag_zero2 == false)
      {
        idx_indirect_link = 0;
      }
      else
      { 
//        int q = 10;
        //return idx_j;
      }
        
    } // else 2
      
  } // else 1

}
//-----------------------------------------------------------------------------------------
