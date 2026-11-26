#include "cloop.hpp"
#include "gmap_landmark_identity.hpp"
//------------------------------------------------------------------------------------------------------
void CLOOP::Update_Visibility_Graph_with_matched_points_into_KF(std::vector<int64> &idx_points, int idx_KF)
{
    
    for (int i = 0; i < idx_points.size(); i++)
    {
        int64 idx_pt = idx_points[i];
        if (idx_pt < 0 ||
          static_cast<std::size_t>(idx_pt) >= Gmap.AnchorsDATA.size())
        {
          continue;
        }
        const LandmarkId lid = Gmap.AnchorsDATA[idx_pt].id;
        
        for (int j = 0 ; j < Gmap.KeyFDATA.size(); j++)
        {
            if(j != idx_KF)
            {
              for( int k = 0; k < Gmap.KeyFDATA[j].Matched_landmark_ids.size(); k++)
              {
                  if(Gmap.KeyFDATA[j].Matched_landmark_ids[k] == lid)
                  {
                    Gmap.Vgraph.at(idx_KF,j)++;
                    Gmap.Vgraph.at(j,idx_KF)++;
                    break;
                  }
              }
            }      
        }


    }
   
   
    



}




//---------------------------------------------------------------------------------
void CLOOP::Update_Visibility_Graph_A_B_links(std::vector<int> &idx_vl_kf_A,std::vector<int> &idx_vl_kf_B)
{

  for (int i = 0; i < idx_vl_kf_A.size() ; i++)
  {
    int idx_kf_A = idx_vl_kf_A[i];
    int idx_kf_B = idx_vl_kf_B[i];

    Gmap.Vgraph.at(idx_kf_A,idx_kf_B)++;
    Gmap.Vgraph.at(idx_kf_B,idx_kf_A)++;
  }

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
  
  int vg_size = Gmap.KeyFDATA.size();
  Gmap.Vgraph.resize(vg_size,vg_size); // increase size of visibility graph by one
  Gmap.Vgraph(vg_size-1,vg_size-1) = 0;

  arma::rowvec vg_row;
  vg_row.zeros(vg_size);

  
  // -----for taking into account the visual link with 0-kf
  if(idx_KF == 2)
  {
    for(int i = 0; i < Gmap.KeyFDATA[idx_KF-1].Matched_landmark_ids.size();i++)
    {
      const LandmarkId lid = Gmap.KeyFDATA[idx_KF-1].Matched_landmark_ids[i];
      std::size_t idx_pt = 0;
      if (!gmap_landmark_id::resolve_landmark_index(Gmap, lid, idx_pt)) {
        continue;
      }

      int idx_init_kf = Gmap.AnchorsDATA[idx_pt].init_KF;

      if  (idx_init_kf == 1)
      {
        Gmap.Vgraph.at(0,1)++;
        Gmap.Vgraph.at(1,0)++;
      }
    }
  }
  //----------------------------------------------------------

  for(int i = 0; i < Gmap.KeyFDATA[idx_KF].Matched_landmark_ids.size();i++)
  {
      const LandmarkId lid = Gmap.KeyFDATA[idx_KF].Matched_landmark_ids[i];

      for (int j = 0; j < Gmap.KeyFDATA.size()-1 ; j++)
      {
        for(int k = 0; k < Gmap.KeyFDATA[j].Matched_landmark_ids.size();k++)
        {
           if (lid == Gmap.KeyFDATA[j].Matched_landmark_ids[k])
           {
             vg_row[j]++;
             break;
           } 

        }

      }

  }
  
  arma::vec vg_col = vg_row.t();      
  Gmap.Vgraph.row(vg_size-1)  = vg_row;
  Gmap.Vgraph.col(vg_size-1) = vg_col;




}

