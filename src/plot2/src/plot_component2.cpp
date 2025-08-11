#include "plot_component2.hpp"
#include "../../common/Transforms/quat2R.hpp"
#include "../../common/Transforms/Euler_to_Ra2b.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION


using std::placeholders::_1;
using std::placeholders::_2;
using namespace cv;

namespace plot2
{

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
} 



// Create a Talker "component" that subclasses the generic rclcpp::Node base class.
// Components get built into shared libraries and as such do not write their own main functions.
// The process using the component's shared library will instantiate the class as a ROS node.
PLOTscene2::PLOTscene2(const rclcpp::NodeOptions & options)
: Node("PLOTscene", options)
{ 

  setParameters();
  // Set subscribers
  sub_Frame_ = this->create_subscription<interfaces::msg::Frame>("Frame_topic", 10, std::bind(&PLOTscene2::Frame_callback, this, _1)); 
  sub_lslam_data_ = this->create_subscription<interfaces::msg::Lslam>("lslam_data_topic", 10, std::bind(&PLOTscene2::lslam_data_callback, this, _1)); 
  // sub_gmap_data_ = this->create_subscription<interfaces::msg::Gmap>("gmap_data_topic",10,std::bind(&PLOTscene2::gmap_data_callback, this, _1));
  sub_Gps_ = this->create_subscription<geometry_msgs::msg::PointStamped>("GT_topic", 10, std::bind(&PLOTscene2::GT_callback, this, _1));
  
  
  running_ = true;
  lslamstarted_ = false;
  plot_loop_ = thread(&PLOTscene2::PLOT_LOOP,this);
  distance_traveled_xy = 0;

  

}

//--------------------------------------------------------------------------
PLOTscene2::~PLOTscene2()
{  
  running_ = false;
        if (ui_thread_.joinable()) {
            ui_thread_.join();
        }
}
//------------------------------------------------------------------------
double computeXYDistance(const arma::vec3& a, const arma::vec3& b) {
    double dx = a(0) - b(0);  // x component
    double dy = a(1) - b(1);  // y component
    return std::sqrt(dx * dx + dy * dy);
}
//  Main Plot loop (this function runs in a separate thread)
void PLOTscene2::PLOT_LOOP()
{
  cout << "-> Plot2 thread running... " << endl;     

  if (!glfwInit()) return;
        
        
        GLFWwindow* window = glfwCreateWindow(960, 830, "SLAM output", NULL, NULL);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1);
       

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;

        ImGui::StyleColorsDark();
        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init("#version 130");

        bool lastKeyState = false;

        // set logger 
        Logger logger_(label_logs);
        std::chrono::steady_clock::time_point last_log_time = std::chrono::steady_clock::now();
        const std::chrono::milliseconds log_interval(250); // log every 500ms
        // -----

        while (!glfwWindowShouldClose(window) && running_) {
            glfwPollEvents();

            std::vector<arma::vec3> poses;            
            std::vector<arma::vec3> gt;            
            std::vector<arma::vec3> feats;
            std::vector<arma::vec3> anchors;
            get_positions(poses);
            get_gt(gt);
            get_localMap(feats,anchors);

            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            draw_plot(poses,gt,feats,anchors);
            draw_plot_xz(poses,gt,feats,anchors);
            draw_plot_zy(poses,gt,feats,anchors);

            RobotState robotS;
            RobotState robotGT;
            bool get_rs = get_robot_state(robotS);
            get_robot_stateGT(robotGT);
            draw_quad(robotS,robotGT);

            auto now = std::chrono::steady_clock::now();
            if (get_rs && (now - last_log_time) >= log_interval) {
                logger_.log(robotGT.position[0], robotS.position[0],
                            robotGT.position[1], robotS.position[1],
                            robotGT.position[2], robotS.position[2]);
                last_log_time = now;
            }
            
            std::vector<arma::vec2> matchedFeats;            
            std::vector<arma::vec2> unmatchedFeats;            
            std::vector<arma::vec2> matchedAnchors;
            std::vector<arma::vec2> unmatchedAnchors;
            get_FramePoints(matchedFeats,unmatchedFeats,matchedAnchors,unmatchedAnchors);
            
            draw_frame_window(matchedFeats,unmatchedFeats,matchedAnchors,unmatchedAnchors);
            
            bool currentKey = glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS;
            bool save_screenshot = currentKey && !lastKeyState;
            lastKeyState = currentKey;
            if (save_screenshot) {
                capture_frames_ = true;
                save_screen_shoot();
            }
            //-----------------
            

            //----------------

            ImGui::Render();
            int display_w, display_h;
            glfwGetFramebufferSize(window, &display_w, &display_h);
            glViewport(0, 0, display_w, display_h);
            glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            glfwSwapBuffers(window);
        }
        
        cout << "-> Plot2 thread closing... " << endl;  
        // Cleanup
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        glfwDestroyWindow(window);
        glfwTerminate();
}

   



void  PLOTscene2::lslam_data_callback(const interfaces::msg::Lslam & msg) 
{
  if(!lslamstarted_)lslamstarted_ = true;
  
  // Define your minimum Euclidean distance threshold
  const double MIN_DISTANCE = 0.2; // Adjust this value as needed (e.g., 0.1 meters)
  
  double quat[4];
  quat[0] = msg.n2r.rotation.w;
  quat[1] = msg.n2r.rotation.x;
  quat[2] = msg.n2r.rotation.y;
  quat[3] = msg.n2r.rotation.z;  
  double Rn2r_a[9];
  quat2R_row_major(quat,Rn2r_a);

  // Wrap raw array into Armadillo matrix
  //arma::mat Rn2r(Rn2r_a, 3, 3, false); // false = don't copy, use external memory (like cv::Mat)
  // Define identity matrix Rb
  // arma::mat Rb = arma::eye<arma::mat>(3, 3);
  // Transpose to get Rr2n
  // arma::mat Rr2n = Rn2r.t();
  // Translation vector t_r
  arma::vec3 t_r = {
      msg.n2r.translation.x,
      msg.n2r.translation.y,
      msg.n2r.translation.z};

    
    { // lock block
        std::lock_guard<std::mutex> lock(data_mutex_);
        
        RobotState_.position = t_r;
        RobotState_.orientation = {
            msg.robot_state[1],
            msg.robot_state[2],
            msg.robot_state[3],
        };
        RobotState_.velocity = {
            msg.robot_state[10],
            msg.robot_state[11],
            msg.robot_state[12],
        };

        // Only emplace the position if the Euclidean distance to the last position is above the threshold
        if (positions_.empty() || arma::norm(t_r - positions_.back(), 2) >= MIN_DISTANCE) {
            positions_.emplace_back(t_r);
        }
        
        mapFeat_.clear();
        mapAnchor_.clear();
        for(int i = 0; i < msg.map_state.size();i++)
        { 
        arma::vec3 featPoint = {
        msg.map_state[i].x,
        msg.map_state[i].y,
        msg.map_state[i].z};  
        mapFeat_.emplace_back(featPoint);      
        }
        for(int i = 0; i < msg.map_anchors.size();i++)
        { 
        arma::vec3 anchorPoint = {
        msg.map_anchors[i].x,
        msg.map_anchors[i].y,
        msg.map_anchors[i].z};  
        mapAnchor_.emplace_back(anchorPoint);        
        }
    }
    
    img_matched_feats_.clear();
    img_unmatched_feats_.clear();
    img_matched_anchors_.clear();
    img_unmatched_anchors_.clear();  
    
    std::lock_guard<std::mutex> lock(frame_mutex_);
    for(int i = 0; i < msg.matched_img_feats.size();i++)
      { 
        arma::vec2 pt = {
            msg.matched_img_feats[i].x,
            msg.matched_img_feats[i].y
        };        
        img_matched_feats_.push_back(pt);      
      } 
      for(int i = 0; i < msg.unmatched_img_feats.size();i++)
      {
        arma::vec2 pt = {
            msg.unmatched_img_feats[i].x,
            msg.unmatched_img_feats[i].y
        };        
        img_unmatched_feats_.push_back(pt);         
      }
      for(int i = 0; i < msg.matched_img_anchors.size();i++)
      { 
        arma::vec2 pt = {
            msg.matched_img_anchors[i].x,
            msg.matched_img_anchors[i].y
        };        
        img_matched_anchors_.push_back(pt);            
      } 
      for(int i = 0; i < msg.unmatched_img_anchors.size();i++)
      { 
        arma::vec2 pt = {
            msg.unmatched_img_anchors[i].x,
            msg.unmatched_img_anchors[i].y
        };        
        img_unmatched_anchors_.push_back(pt);              
      }  
    

}

void PLOTscene2::GT_callback(const geometry_msgs::msg::PointStamped & msg)
  { 
    static double x0, y0, z0;
    static bool init = false;

    // Define your minimum Euclidean distance threshold
    const double MIN_DISTANCE = 0.2; // Adjust this value as needed (e.g., 0.1 meters)

    double x = msg.point.x;
    double y = msg.point.y;
    double z = msg.point.z;

    if(lslamstarted_ == true)
    { 
        if (init == false)
        {
            init = true;
            x0 = x;
            y0 = y;
            z0 = z;
        }
        else{    
            std::lock_guard<std::mutex> lock(data_mutex_);
            arma::vec3 t_r = {
                            x - x0,
                            y - y0,
                            z - z0};
            RobotStateGT_.position = t_r;
            // Only emplace the position if the Euclidean distance to the last position is above the threshold
            if (gt_.empty() || arma::norm(t_r - gt_.back(), 2) >= MIN_DISTANCE) {
                gt_.emplace_back(t_r);
                distance_traveled_xy += computeXYDistance(gt_[gt_.size() - 2], gt_.back());     
                }          
                 
        }
    }  

  }

//------------------------------------------------------------------------------------------------------    
void PLOTscene2::Frame_callback(const interfaces::msg::Frame & msg) const {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg.img, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("PLOTscene2"), "cv_bridge exception: %s", e.what());
        return;
    }

    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = cv_ptr->image.clone();  // clone to decouple from ROS memory
    new_frame_available_ = true;
}

bool PLOTscene2::get_robot_state(RobotState &out){
        std::lock_guard<std::mutex> lock(data_mutex_);
        out = RobotState_;

        
        static RobotState last_checked_state;
        /*
        static auto last_update_time = std::chrono::steady_clock::now();
        static const auto timeout = std::chrono::seconds(5);

        if (arma::approx_equal(out.position, last_checked_state.position, "absdiff", 1e-8)) {
            auto now = std::chrono::steady_clock::now();
            if (now - last_update_time > timeout) {
                lslamstarted_ = false;
            }
        } else {
            last_checked_state = out;
            last_update_time = std::chrono::steady_clock::now();
            lslamstarted_ = true;
            }
        */
        if (arma::approx_equal(out.position, last_checked_state.position, "absdiff", 1e-8)) return false;

        last_checked_state = out;    
        
        return true;
    }
        
    

void PLOTscene2::get_robot_stateGT(RobotState &out){
        std::lock_guard<std::mutex> lock(data_mutex_);
        out = RobotStateGT_;
    }

void PLOTscene2::get_localMap(std::vector<arma::vec3>& out1, std::vector<arma::vec3>& out2){
        std::lock_guard<std::mutex> lock(data_mutex_);
        out1 = mapFeat_;
        out2 = mapAnchor_;
    }

void PLOTscene2::get_FramePoints(std::vector<arma::vec2>& mf, std::vector<arma::vec2>& uf,std::vector<arma::vec2>& ma, std::vector<arma::vec2>& ua){
        std::lock_guard<std::mutex> lock(frame_mutex_);
        mf = img_matched_feats_;
        uf = img_unmatched_feats_;
        ma = img_matched_anchors_;
        ua = img_unmatched_anchors_;         
    }

void PLOTscene2::get_gt(std::vector<arma::vec3>& out) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        out = gt_;
    }

void PLOTscene2::get_positions(std::vector<arma::vec3>& out) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        out = positions_;
    }

//--------------------------------------------------------
void PLOTscene2::draw_frame_window(const std::vector<arma::vec2>& matchedFeats_,
                                   const std::vector<arma::vec2>& unmatchedFeats_,
                                   const std::vector<arma::vec2>& matchedAnchors_,
                                   const std::vector<arma::vec2>& unmatchedAnchors_) {
    
    std::lock_guard<std::mutex> lock(frame_mutex_);

    if (new_frame_available_ && !latest_frame_.empty()) {
        if (latest_texture_id_ != 0) {
            glDeleteTextures(1, &latest_texture_id_);
        }

        glGenTextures(1, &latest_texture_id_);
        glBindTexture(GL_TEXTURE_2D, latest_texture_id_);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, latest_frame_.cols, latest_frame_.rows,
                     0, GL_BGR, GL_UNSIGNED_BYTE, latest_frame_.data);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        latest_img_width_ = latest_frame_.cols;
        latest_img_height_ = latest_frame_.rows;

        new_frame_available_ = false;
    }

    if (latest_texture_id_ != 0) {
        ImVec2 canvas_size = ImVec2(latest_img_width_, latest_img_height_ + 20);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImVec2 origin = viewport->Pos;
        ImGui::SetNextWindowPos(ImVec2(origin.x + 5, origin.y + 10), ImGuiCond_Once);
        ImGui::SetNextWindowSize(canvas_size, ImGuiCond_Once);

        ImGui::Begin("Camera Frame");

        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 image_pos = ImGui::GetCursorScreenPos();

        float aspect_ratio = static_cast<float>(latest_img_width_) / latest_img_height_;
        float max_width = 400.0f;
        float img_width = std::min(max_width, ImGui::GetContentRegionAvail().x);
        float img_height = img_width / aspect_ratio;

        ImGui::Image(reinterpret_cast<void*>(static_cast<intptr_t>(latest_texture_id_)),
                     ImVec2(img_width, img_height));

        auto draw_circle = [&](const arma::vec2& pt, ImU32 color, bool filled) {
            float x = image_pos.x + pt(0) * (img_width / latest_img_width_);
            float y = image_pos.y + pt(1) * (img_height / latest_img_height_);
            float radius = 3.0f;
            if (filled) {
                draw_list->AddCircleFilled(ImVec2(x, y), radius, color);
            } else {
                draw_list->AddCircle(ImVec2(x, y), radius, color, 12, 1.5f);
            }
        };

        // Matched features: solid blue
        for (const auto& pt : matchedFeats_)     draw_circle(pt, IM_COL32(0, 0, 255, 255), true);
        // Unmatched features: hollow blue
        for (const auto& pt : unmatchedFeats_)   draw_circle(pt, IM_COL32(0, 0, 255, 255), false);

        // Matched anchors: solid purple
        for (const auto& pt : matchedAnchors_)   draw_circle(pt, IM_COL32(150, 0, 255, 255), true);
        // Unmatched anchors: hollow purple
        for (const auto& pt : unmatchedAnchors_) draw_circle(pt, IM_COL32(150, 0, 255, 255), false);

        ImGui::End();
        ImGui::PopStyleVar();
    }
}

//------------------------------------------------------------------


void PLOTscene2::draw_plot(const std::vector<arma::vec3>& positions_, const std::vector<arma::vec3>& gt_, const std::vector<arma::vec3>& feats_, const std::vector<arma::vec3>& anchors_) {
    
    ImVec2 canvas_size = ImVec2(500, 500);   

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

    // Place window relative to main viewport
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImVec2 origin = viewport->Pos;
    ImGui::SetNextWindowPos(ImVec2(origin.x + 450, origin.y + 320), ImGuiCond_Once);
    ImGui::SetNextWindowSize(canvas_size, ImGuiCond_Once);
    
    
    ImGui::Begin("Robot Path", nullptr,
    ImGuiWindowFlags_NoScrollbar |
    ImGuiWindowFlags_NoScrollWithMouse);

    //ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 window_pos = ImGui::GetWindowPos();
    ImVec2 canvas_pos = window_pos;
    
    ImVec2 canvas_end = ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    static float scale = 10.0f;
    static ImVec2 pan_offset = ImVec2(0, 0);
    static bool is_panning = false;
    static ImVec2 last_mouse_pos;
    const float scale_speed = 1.1f;

    ImGui::InvisibleButton("canvas", canvas_size, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    bool is_hovered = ImGui::IsItemHovered();
    ImVec2 mouse_pos = ImGui::GetIO().MousePos;

    if (is_hovered) {
        float wheel = ImGui::GetIO().MouseWheel;
        if (wheel != 0.0f) {
            scale *= (wheel > 0.0f) ? scale_speed : (1.0f / scale_speed);
            scale = std::clamp(scale, 1.0f, 1000.0f);
        }
    }

    if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        is_panning = true;
        last_mouse_pos = mouse_pos;
    }

    if (is_panning && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImVec2 delta = ImVec2(mouse_pos.x - last_mouse_pos.x, mouse_pos.y - last_mouse_pos.y);
        pan_offset.x += delta.x;
        pan_offset.y += delta.y;
        last_mouse_pos = mouse_pos;
    } else {
        is_panning = false;
    }

    draw_list->AddRectFilled(canvas_pos, canvas_end, IM_COL32(255, 255, 255, 255));

    ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5f + pan_offset.x,
                           canvas_pos.y + canvas_size.y * 0.5f + pan_offset.y);

    // Coordinate transform
    auto world_to_screen = [&](const arma::vec3& pt) -> ImVec2 {
        float plot_x = pt(1);        // robot Y → plot X
        float plot_y = -pt(0);       // robot X → plot Y (inverted)
        return ImVec2(center.x + plot_x * scale,
                      center.y + plot_y * scale);
    };

    auto screen_to_world = [&](const ImVec2& screen) -> arma::vec3 {
        float dx = (screen.x - center.x) / scale;
        float dy = (screen.y - center.y) / scale;
        return arma::vec3({ -dy, dx, 0.0 });  // Inverse of world_to_screen
    };

    // ───── Draw Grid ─────
    const float spacing = 1.0f; // 1 meter grid
    ImU32 grid_color = IM_COL32(200, 200, 200, 64);
    float line_thickness = 1.0f;

    // Convert canvas bounds to world coords
    arma::vec3 world_min = screen_to_world(canvas_pos);
    arma::vec3 world_max = screen_to_world(canvas_end);

    float x_start = std::floor(std::min(world_min(0), world_max(0)));
    float x_end   = std::ceil(std::max(world_min(0), world_max(0)));
    float y_start = std::floor(std::min(world_min(1), world_max(1)));
    float y_end   = std::ceil(std::max(world_min(1), world_max(1)));

    // Horizontal lines (constant X world)
    for (float x = x_start; x <= x_end; x += spacing) {
        arma::vec3 a = { x, y_start, 0 };
        arma::vec3 b = { x, y_end, 0 };
        draw_list->AddLine(world_to_screen(a), world_to_screen(b), grid_color, line_thickness);
    }

    // Vertical lines (constant Y world)
    for (float y = y_start; y <= y_end; y += spacing) {
        arma::vec3 a = { x_start, y, 0 };
        arma::vec3 b = { x_end, y, 0 };
        draw_list->AddLine(world_to_screen(a), world_to_screen(b), grid_color, line_thickness);
    }
    
    // ───── Draw Feature Points ─────
    for (const auto& pt : feats_) {
        ImVec2 p = world_to_screen(pt);
        draw_list->AddCircleFilled(p, 1.5f, IM_COL32(0, 0, 255, 200)); // Blue-ish for features
    }

    // ───── Draw Anchor Points ─────
    for (const auto& pt : anchors_) {
        ImVec2 p = world_to_screen(pt);
        draw_list->AddCircleFilled(p, 1.5f, IM_COL32(150, 0, 255, 200)); // 
    }

    // ───── Draw gt ─────
    for (size_t i = 1; i < gt_.size(); ++i) {
        ImVec2 p1 = world_to_screen(gt_[i - 1]);
        ImVec2 p2 = world_to_screen(gt_[i]);
        draw_list->AddLine(p1, p2, IM_COL32(0, 0, 0, 255), 1.5f);

        
    }

    // ───── Draw Path ─────
    for (size_t i = 1; i < positions_.size(); ++i) {
        ImVec2 p1 = world_to_screen(positions_[i - 1]);
        ImVec2 p2 = world_to_screen(positions_[i]);
        draw_list->AddLine(p1, p2, IM_COL32(255, 100, 100, 255), 1.5f);
    }

    
    // ───── Draw Current Positions (Estimated and Ground Truth) ─────
    if (!positions_.empty() && !gt_.empty()) {
        // Ground truth position (last in gt_)
        ImVec2 gt_pos = world_to_screen(gt_.back());
        float cross_size = 6.0f; // Cross size

        // Angle for the cross rotation (45 degrees)
        float angle = .785; // 45 degrees in radians

        // Calculate the rotated positions for the X shape
        ImVec2 gt_line1_start = ImVec2(gt_pos.x - cross_size * cos(angle), gt_pos.y - cross_size * sin(angle));
        ImVec2 gt_line1_end = ImVec2(gt_pos.x + cross_size * cos(angle), gt_pos.y + cross_size * sin(angle));

        ImVec2 gt_line2_start = ImVec2(gt_pos.x - cross_size * cos(angle), gt_pos.y + cross_size * sin(angle));
        ImVec2 gt_line2_end = ImVec2(gt_pos.x + cross_size * cos(angle), gt_pos.y - cross_size * sin(angle));

        // Draw ground truth position as a rotated "X" (black)
        draw_list->AddLine(gt_line1_start, gt_line1_end, IM_COL32(0, 0, 0, 255)); // First diagonal line
        draw_list->AddLine(gt_line2_start, gt_line2_end, IM_COL32(0, 0, 0, 255)); // Second diagonal line

        // Estimated position (last in positions_)
        ImVec2 estimated_pos = world_to_screen(positions_.back());

        // Calculate the rotated positions for the X shape
        ImVec2 est_line1_start = ImVec2(estimated_pos.x - cross_size * cos(angle), estimated_pos.y - cross_size * sin(angle));
        ImVec2 est_line1_end = ImVec2(estimated_pos.x + cross_size * cos(angle), estimated_pos.y + cross_size * sin(angle));

        ImVec2 est_line2_start = ImVec2(estimated_pos.x - cross_size * cos(angle), estimated_pos.y + cross_size * sin(angle));
        ImVec2 est_line2_end = ImVec2(estimated_pos.x + cross_size * cos(angle), estimated_pos.y - cross_size * sin(angle));

        // Draw estimated position as a rotated "X" (red)
        draw_list->AddLine(est_line1_start, est_line1_end, IM_COL32(255, 0, 0, 255)); // First diagonal line
        draw_list->AddLine(est_line2_start, est_line2_end, IM_COL32(255, 0, 0, 255)); // Second diagonal line
    }

    
    // ───── Draw Coordinate Frame ─────
    const float lc = 5.0f; // 1 meter grid
    // X-axis
    ImVec2 x_axis_start = world_to_screen(arma::vec3{0.0, 0.0, 0.0});  // Origin
    ImVec2 x_axis_end = world_to_screen(arma::vec3{lc, 0.0, 0.0});    // 10 units along X
    draw_list->AddLine(x_axis_start, x_axis_end, IM_COL32(0, 255, 0, 255), 1.0f);  // Red for X axis

    // Y-axis
    ImVec2 y_axis_start = world_to_screen(arma::vec3{0, 0, 0});  // Origin
    ImVec2 y_axis_end = world_to_screen(arma::vec3{0, lc, 0});    // 10 units along Y
    draw_list->AddLine(y_axis_start, y_axis_end, IM_COL32(0, 255, 0, 255), 1.0f);  // Green for Y axis

    // X-axis label
    ImVec2 x_label_pos = world_to_screen(arma::vec3{lc, 0, 0}); // Position label at X axis end
    draw_list->AddText(x_label_pos, IM_COL32(0, 255, 0, 255), "X");

    // Y-axis label
    ImVec2 y_label_pos = world_to_screen(arma::vec3{0, lc, 0}); // Position label at Y axis end
    draw_list->AddText(y_label_pos, IM_COL32(0, 255, 0, 255), "Y");

    ImGui::End();
    ImGui::PopStyleVar();
}

//--------------------------------------------------------------------------------------

void PLOTscene2::draw_plot_xz(const std::vector<arma::vec3>& positions_, const std::vector<arma::vec3>& gt_, const std::vector<arma::vec3>& feats_, const std::vector<arma::vec3>& anchors_) {
    
    ImVec2 canvas_size = ImVec2(440, 220);   

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

    // Place window relative to main viewport
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImVec2 origin = viewport->Pos;
    ImGui::SetNextWindowPos(ImVec2(origin.x + 5, origin.y + 320), ImGuiCond_Once);
    ImGui::SetNextWindowSize(canvas_size, ImGuiCond_Once);
    
    
    ImGui::Begin("Robot Lateral View (X-Z)", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_end = ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    static float scale = 10.0f;
    static ImVec2 pan_offset = ImVec2(0, 0);
    static bool is_panning = false;
    static ImVec2 last_mouse_pos;
    const float scale_speed = 1.1f;

    ImGui::InvisibleButton("canvas_xz", canvas_size,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    bool is_hovered = ImGui::IsItemHovered();
    ImVec2 mouse_pos = ImGui::GetIO().MousePos;

    if (is_hovered) {
        float wheel = ImGui::GetIO().MouseWheel;
        if (wheel != 0.0f) {
            scale *= (wheel > 0.0f) ? scale_speed : (1.0f / scale_speed);
            scale = std::clamp(scale, 1.0f, 1000.0f);
        }
    }

    if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        is_panning = true;
        last_mouse_pos = mouse_pos;
    }

    if (is_panning && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImVec2 delta = ImVec2(mouse_pos.x - last_mouse_pos.x, mouse_pos.y - last_mouse_pos.y);
        pan_offset.x += delta.x;
        pan_offset.y -= delta.y;
        last_mouse_pos = mouse_pos;
    } else {
        is_panning = false;
    }

    draw_list->AddRectFilled(canvas_pos, canvas_end, IM_COL32(255, 255, 255, 255));

    ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5f + pan_offset.x,
                           canvas_pos.y + canvas_size.y * 0.5f - pan_offset.y);

    // Coordinate transform for X-Z view
    auto world_to_screen = [&](const arma::vec3& pt) -> ImVec2 {
        float plot_x = pt(0);         // robot X → plot X
        float plot_y = pt(2);        // robot Z → plot Y (inverted so up is up)
        return ImVec2(center.x + plot_x * scale,
                      center.y + plot_y * scale);
    };

    auto screen_to_world = [&](const ImVec2& screen) -> arma::vec3 {
        float dx = (screen.x - center.x) / scale;
        float dz = (screen.y - center.y) / scale; 
        return arma::vec3({ dx, 0.0, dz });  // only X and Z matter here
    };

    // ───── Draw Grid ─────
    const float spacing = 1.0f;
    ImU32 grid_color = IM_COL32(200, 200, 200, 64);
    float line_thickness = 1.0f;

    arma::vec3 world_min = screen_to_world(canvas_pos);
    arma::vec3 world_max = screen_to_world(canvas_end);

    float x_start = std::floor(std::min(world_min(0), world_max(0)));
    float x_end   = std::ceil(std::max(world_min(0), world_max(0)));
    float z_start = std::floor(std::min(world_min(2), world_max(2)));
    float z_end   = std::ceil(std::max(world_min(2), world_max(2)));

    for (float x = x_start; x <= x_end; x += spacing) {
        arma::vec3 a = { x, 0, z_start };
        arma::vec3 b = { x, 0, z_end };
        draw_list->AddLine(world_to_screen(a), world_to_screen(b), grid_color, line_thickness);
    }

    for (float z = z_start; z <= z_end; z += spacing) {
        arma::vec3 a = { x_start, 0, z };
        arma::vec3 b = { x_end, 0, z };
        draw_list->AddLine(world_to_screen(a), world_to_screen(b), grid_color, line_thickness);
    }

    // ───── Draw Features and Anchors ─────
    for (const auto& pt : feats_) {
        ImVec2 p = world_to_screen(pt);
        draw_list->AddCircleFilled(p, 1.5f, IM_COL32(0, 0, 255, 200));
    }

    for (const auto& pt : anchors_) {
        ImVec2 p = world_to_screen(pt);
        draw_list->AddCircleFilled(p, 1.5f, IM_COL32(150, 0, 255, 200));
    }

    // ───── Draw Ground Truth Path ─────
    for (size_t i = 1; i < gt_.size(); ++i) {
        ImVec2 p1 = world_to_screen(gt_[i - 1]);
        ImVec2 p2 = world_to_screen(gt_[i]);
        draw_list->AddLine(p1, p2, IM_COL32(0, 0, 0, 255), 1.5f);
    }

    // ───── Draw Estimated Path ─────
    for (size_t i = 1; i < positions_.size(); ++i) {
        ImVec2 p1 = world_to_screen(positions_[i - 1]);
        ImVec2 p2 = world_to_screen(positions_[i]);
        draw_list->AddLine(p1, p2, IM_COL32(255, 100, 100, 255), 1.5f);
    }

    // ───── Draw Current Positions ─────
    if (!positions_.empty() && !gt_.empty()) {
        ImVec2 gt_pos = world_to_screen(gt_.back());
        ImVec2 est_pos = world_to_screen(positions_.back());
        float cross_size = 6.0f;

        // Ground truth (black +)
        draw_list->AddLine(
            ImVec2(gt_pos.x - cross_size, gt_pos.y),
            ImVec2(gt_pos.x + cross_size, gt_pos.y),
            IM_COL32(0, 0, 0, 255));
        draw_list->AddLine(
            ImVec2(gt_pos.x, gt_pos.y - cross_size),
            ImVec2(gt_pos.x, gt_pos.y + cross_size),
            IM_COL32(0, 0, 0, 255));

        // Estimated (red +)
        draw_list->AddLine(
            ImVec2(est_pos.x - cross_size, est_pos.y),
            ImVec2(est_pos.x + cross_size, est_pos.y),
            IM_COL32(255, 0, 0, 255));
        draw_list->AddLine(
            ImVec2(est_pos.x, est_pos.y - cross_size),
            ImVec2(est_pos.x, est_pos.y + cross_size),
            IM_COL32(255, 0, 0, 255));
    }

    // ───── Coordinate Frame (X-Z) ─────
    ImVec2 origin_o = world_to_screen(arma::vec3({0.0, 0.0, 0.0}));
    ImVec2 x_axis_end = world_to_screen(arma::vec3({2.0, 0.0, 0.0}));
    ImVec2 z_axis_end = world_to_screen(arma::vec3({0.0, 0.0, 2.0}));

    draw_list->AddLine(origin_o, x_axis_end, IM_COL32(0, 255, 0, 255), 1.5f); // X axis
    draw_list->AddLine(origin_o, z_axis_end, IM_COL32(0, 255, 0, 255), 1.5f); // Z axis

    draw_list->AddText(ImVec2(x_axis_end.x + 5, x_axis_end.y), IM_COL32(0, 255, 0, 255), "x");
    draw_list->AddText(ImVec2(z_axis_end.x + 5, z_axis_end.y), IM_COL32(0, 255, 0, 255), "z");

    ImGui::End();
    ImGui::PopStyleVar();
}
//------------------------------------------------------------------------

void PLOTscene2::draw_plot_zy(const std::vector<arma::vec3>& positions_, const std::vector<arma::vec3>& gt_, const std::vector<arma::vec3>& feats_, const std::vector<arma::vec3>& anchors_) {
    
    
    
    
    ImVec2 canvas_size = ImVec2(440, 220);   

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImVec2 origin = viewport->Pos;
    ImGui::SetNextWindowPos(ImVec2(origin.x + 5, origin.y + 550), ImGuiCond_Once);
    ImGui::SetNextWindowSize(canvas_size, ImGuiCond_Once);
    
    ImGui::Begin("Robot Frontal View (Z-Y)", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 canvas_end = ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y);
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    static float scale = 10.0f;
    static ImVec2 pan_offset = ImVec2(0, 0);
    static bool is_panning = false;
    static ImVec2 last_mouse_pos;
    const float scale_speed = 1.1f;

    ImGui::InvisibleButton("canvas_zy", canvas_size,
        ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
    bool is_hovered = ImGui::IsItemHovered();
    ImVec2 mouse_pos = ImGui::GetIO().MousePos;

    if (is_hovered) {
        float wheel = ImGui::GetIO().MouseWheel;
        if (wheel != 0.0f) {
            scale *= (wheel > 0.0f) ? scale_speed : (1.0f / scale_speed);
            scale = std::clamp(scale, 1.0f, 1000.0f);
        }
    }

    if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        is_panning = true;
        last_mouse_pos = mouse_pos;
    }

    if (is_panning && ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
        ImVec2 delta = ImVec2(mouse_pos.x - last_mouse_pos.x, mouse_pos.y - last_mouse_pos.y);
        pan_offset.x += delta.x;
        pan_offset.y -= delta.y;
        last_mouse_pos = mouse_pos;
    } else {
        is_panning = false;
    }

    draw_list->AddRectFilled(canvas_pos, canvas_end, IM_COL32(255, 255, 255, 255));

    ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5f + pan_offset.x,
                           canvas_pos.y + canvas_size.y * 0.5f - pan_offset.y);

    // Z-Y view: invert Z and Y
    auto world_to_screen = [&](const arma::vec3& pt) -> ImVec2 {
        float plot_x = pt(1);  // Y inverted
        float plot_y = pt(2);   // Z normal
        return ImVec2(center.x + plot_x * scale,
                      center.y + plot_y * scale);
    };

    auto screen_to_world = [&](const ImVec2& screen) -> arma::vec3 {
        float dy = -(screen.x - center.x) / scale;
        float dz = (screen.y - center.y) / scale;
        return arma::vec3({0.0, dy, dz});
    };

    // Grid drawing
    const float spacing = 1.0f;
    ImU32 grid_color = IM_COL32(200, 200, 200, 64);
    float line_thickness = 1.0f;

    arma::vec3 world_min = screen_to_world(canvas_pos);
    arma::vec3 world_max = screen_to_world(canvas_end);

    float y_start = std::floor(std::min(world_min(1), world_max(1)));
    float y_end   = std::ceil(std::max(world_min(1), world_max(1)));
    float z_start = std::floor(std::min(world_min(2), world_max(2)));
    float z_end   = std::ceil(std::max(world_min(2), world_max(2)));

    for (float y = y_start; y <= y_end; y += spacing) {
        arma::vec3 a = { 0, y, z_start };
        arma::vec3 b = { 0, y, z_end };
        draw_list->AddLine(world_to_screen(a), world_to_screen(b), grid_color, line_thickness);
    }

    for (float z = z_start; z <= z_end; z += spacing) {
        arma::vec3 a = { 0, y_start, z };
        arma::vec3 b = { 0, y_end, z };
        draw_list->AddLine(world_to_screen(a), world_to_screen(b), grid_color, line_thickness);
    }

    // Draw points
    for (const auto& pt : feats_) {
        draw_list->AddCircleFilled(world_to_screen(pt), 1.5f, IM_COL32(0, 0, 255, 200));
    }

    for (const auto& pt : anchors_) {
        draw_list->AddCircleFilled(world_to_screen(pt), 1.5f, IM_COL32(150, 0, 255, 200));
    }

    for (size_t i = 1; i < gt_.size(); ++i) {
        draw_list->AddLine(world_to_screen(gt_[i - 1]), world_to_screen(gt_[i]), IM_COL32(0, 0, 0, 255), 1.5f);
    }

    for (size_t i = 1; i < positions_.size(); ++i) {
        draw_list->AddLine(world_to_screen(positions_[i - 1]), world_to_screen(positions_[i]), IM_COL32(255, 100, 100, 255), 1.5f);
    }

    if (!positions_.empty() && !gt_.empty()) {
        ImVec2 gt_pos = world_to_screen(gt_.back());
        ImVec2 est_pos = world_to_screen(positions_.back());
        float cross_size = 6.0f;

        draw_list->AddLine({gt_pos.x - cross_size, gt_pos.y}, {gt_pos.x + cross_size, gt_pos.y}, IM_COL32(0, 0, 0, 255));
        draw_list->AddLine({gt_pos.x, gt_pos.y - cross_size}, {gt_pos.x, gt_pos.y + cross_size}, IM_COL32(0, 0, 0, 255));

        draw_list->AddLine({est_pos.x - cross_size, est_pos.y}, {est_pos.x + cross_size, est_pos.y}, IM_COL32(255, 0, 0, 255));
        draw_list->AddLine({est_pos.x, est_pos.y - cross_size}, {est_pos.x, est_pos.y + cross_size}, IM_COL32(255, 0, 0, 255));
    }

    // Coordinate frame (Z-Y)
    ImVec2 origin_o = world_to_screen(arma::vec3({0.0, 0.0, 0.0}));
    ImVec2 z_axis_end = world_to_screen(arma::vec3({0.0, 0.0, 2.0}));
    ImVec2 y_axis_end = world_to_screen(arma::vec3({0.0, 2.0, 0.0}));

    draw_list->AddLine(origin_o, z_axis_end, IM_COL32(0, 255, 0, 255), 1.5f); // Z
    draw_list->AddLine(origin_o, y_axis_end, IM_COL32(0, 255, 0, 255), 1.5f); // Y (inverted)

    draw_list->AddText({z_axis_end.x + 5, z_axis_end.y}, IM_COL32(0, 255, 0, 255), "z");
    draw_list->AddText({y_axis_end.x + 5, y_axis_end.y}, IM_COL32(0, 255, 0, 255), "y");

    ImGui::End();
    ImGui::PopStyleVar();

    
    
}


//------------------------------------------------------------------------
void PLOTscene2::draw_quad(const RobotState &robotState_, const RobotState &robotStateGT_) {
    
    // Set initial size of the window
    float window_width = 3 * 150.0f + 2 * 10.0f + 30.0f;
    float window_height = 300.0f;
    

    ImVec2 canvas_size = ImVec2(window_width, window_height);   

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));

    // Place window relative to main viewport
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImVec2 origin = viewport->Pos;
    ImGui::SetNextWindowPos(ImVec2(origin.x + 450, origin.y + 10), ImGuiCond_Once);
    ImGui::SetNextWindowSize(canvas_size, ImGuiCond_Once);

    ImGui::Begin("Quadrotor Views");

    const ImVec2 view_size(150, 150);
    const ImVec2 spacing(10, 0);
    const float scale = 40.0f;
    const float title_space = 30.0f;

    ImVec2 base_cursor = ImGui::GetCursorScreenPos();
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Lambda to draw a single view box
    auto draw_view = [&](ImVec2 origin, const std::string& title, const std::string& label,
                     arma::vec2 forward_dir, arma::vec2 right_dir, bool draw_quad = false) {
        const float title_space = 30.0f;
        ImVec2 padded_origin = ImVec2(origin.x, origin.y + title_space);
        ImVec2 center = ImVec2(padded_origin.x + view_size.x * 0.5f,
                            padded_origin.y + view_size.y * 0.5f);

        draw_list->AddRectFilled(padded_origin,
                                ImVec2(padded_origin.x + view_size.x, padded_origin.y + view_size.y),
                                IM_COL32(255, 255, 255, 255));
        draw_list->AddRect(padded_origin,
                        ImVec2(padded_origin.x + view_size.x, padded_origin.y + view_size.y),
                        IM_COL32(0, 0, 0, 255));
        draw_list->AddCircleFilled(center, 3.0f, IM_COL32(0, 0, 0, 255));

        ImVec2 fwd = ImVec2(center.x + forward_dir(0) * scale,
                            center.y - forward_dir(1) * scale);
        ImVec2 right = ImVec2(center.x + right_dir(0) * scale,
                            center.y - right_dir(1) * scale);

        draw_list->AddLine(center, fwd, IM_COL32(255, 0, 0, 255), 2.0f);
        draw_list->AddLine(center, right, IM_COL32(0, 0, 255, 255), 2.0f);

        draw_list->AddText(ImVec2(origin.x, origin.y), IM_COL32(255, 255, 255, 255), title.c_str());
        draw_list->AddText(ImVec2(origin.x, origin.y + 15), IM_COL32(200, 200, 200, 255), label.c_str());

        // --- Draw quadcopter body (X configuration) ---
        if (draw_quad) {
            float arm_length = 30.0f;
            float rotor_radius = 15.0f;

            arma::vec2 dir1 = arma::normalise(forward_dir + right_dir);
            arma::vec2 dir2 = arma::normalise(forward_dir - right_dir);

            ImVec2 arm1_start = ImVec2(center.x - dir1(0) * arm_length, center.y + dir1(1) * arm_length);
            ImVec2 arm1_end   = ImVec2(center.x + dir1(0) * arm_length, center.y - dir1(1) * arm_length);

            ImVec2 arm2_start = ImVec2(center.x - dir2(0) * arm_length, center.y + dir2(1) * arm_length);
            ImVec2 arm2_end   = ImVec2(center.x + dir2(0) * arm_length, center.y - dir2(1) * arm_length);

            draw_list->AddLine(arm1_start, arm1_end, IM_COL32(0, 0, 0, 255), 2.0f);
            draw_list->AddLine(arm2_start, arm2_end, IM_COL32(0, 0, 0, 255), 2.0f);

            draw_list->AddCircle(arm1_start, rotor_radius, IM_COL32(100, 100, 100, 255), 16, 1.5f);
            draw_list->AddCircle(arm1_end, rotor_radius, IM_COL32(100, 100, 100, 255), 16, 1.5f);
            draw_list->AddCircle(arm2_start, rotor_radius, IM_COL32(100, 100, 100, 255), 16, 1.5f);
            draw_list->AddCircle(arm2_end, rotor_radius, IM_COL32(100, 100, 100, 255), 16, 1.5f);
        }
    };


    // Orientation
    float roll  = -robotState_.orientation(0);
    float pitch = robotState_.orientation(1);
    float yaw   = -robotState_.orientation(2);

    // Format strings with angle values in degrees
    auto rad2deg = [](float rad) { return rad * 180.0f / float(M_PI); };
    char yaw_str[64], pitch_str[64], roll_str[64];
    snprintf(yaw_str, sizeof(yaw_str), "Yaw: %.1f°", rad2deg(yaw));
    snprintf(pitch_str, sizeof(pitch_str), "Pitch: %.1f°", rad2deg(pitch));
    snprintf(roll_str, sizeof(roll_str), "Roll: %.1f°", rad2deg(roll));

    // Axis projections
    arma::vec2 fwd_xy   = { std::cos(yaw), std::sin(yaw) };
    arma::vec2 right_xy = { -std::sin(yaw), std::cos(yaw) };

    arma::vec2 fwd_xz   = { std::cos(pitch), std::sin(pitch) };
    arma::vec2 up_xz    = { -std::sin(pitch), std::cos(pitch) };

    arma::vec2 fwd_yz   = { std::cos(roll), std::sin(roll) };
    arma::vec2 up_yz    = { -std::sin(roll), std::cos(roll) };

    // Draw each view
    //ImVec2 cursor = ImGui::GetCursorScreenPos();
    base_cursor.x = base_cursor.x + 10;
    draw_view(ImVec2(base_cursor.x , base_cursor.y), 
             yaw_str, "Top View (XY)", fwd_xy, right_xy, true); 
    draw_view(ImVec2(base_cursor.x + view_size.x + spacing.x, base_cursor.y),
              pitch_str, "Front View (XZ)", fwd_xz, up_xz);
    draw_view(ImVec2(base_cursor.x + 2 * (view_size.x + spacing.x), base_cursor.y),
              roll_str,  "Side View (YZ)",  fwd_yz, up_yz);

    ImGui::Dummy(ImVec2(3 * (view_size.x + spacing.x), view_size.y + title_space + 10));

     // ───── Position Info Table ─────
    ImGui::Separator();
    if (ImGui::BeginTable("PositionTable", 4, ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) {
        // Setup columns
        ImGui::TableSetupColumn("Axis", ImGuiTableColumnFlags_WidthFixed, 100.0f);
        ImGui::TableSetupColumn("X (m)", ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("Y (m)", ImGuiTableColumnFlags_WidthFixed, 70.0f);
        ImGui::TableSetupColumn("Z (m)", ImGuiTableColumnFlags_WidthFixed, 70.0f);

        // Show header row
        ImGui::TableHeadersRow();

        // Estimated
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("Estimated:");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", robotState_.position(0));
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.2f", robotState_.position(1));
        ImGui::TableSetColumnIndex(3); ImGui::Text("%.2f", robotState_.position(2));

        // Ground Truth
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0); ImGui::Text("Ground Truth:");
        ImGui::TableSetColumnIndex(1); ImGui::Text("%.2f", robotStateGT_.position(0));
        ImGui::TableSetColumnIndex(2); ImGui::Text("%.2f", robotStateGT_.position(1));
        ImGui::TableSetColumnIndex(3); ImGui::Text("%.2f", robotStateGT_.position(2));

    ImGui::EndTable();
    }

    // Optional: Add distance error
    arma::vec3 err = robotState_.position - robotStateGT_.position;
    double dist = arma::norm(err);
    ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "Position Error: %.3f m", dist);

    ImGui::TextColored(ImVec4(0.7f, 0.9f, 0.7f, 1.0f), "Distance Traveled (XY): %.3f m", distance_traveled_xy);


    ImGui::End();
    ImGui::PopStyleVar();
}

void PLOTscene2::save_screen_shoot(){

    bool capture = capture_frames_;
    if (capture) capture_frames_ = false; // Reset
        
    if (capture) {
        GLint viewport[4];
        glGetIntegerv(GL_VIEWPORT, viewport);  // x, y, width, height
        int width = viewport[2];
        int height = viewport[3];

        // Allocate OpenCV matrix
        cv::Mat img(height, width, CV_8UC3);

        // Setup pixel alignment
        glPixelStorei(GL_PACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
        glPixelStorei(GL_PACK_ROW_LENGTH, img.step / img.elemSize());

        // Read pixels from framebuffer (backbuffer)
        glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, img.data);

        // Flip vertically because OpenGL origin is bottom-left, OpenCV is top-left
        cv::flip(img, img, 0);

        // Save to file
        cv::imwrite("plots_capture.png", img);
    }

}

void PLOTscene2::setParameters()
{  

   //  Declare node parameters (and default values)
   this->declare_parameter<string>("label_logs", "CONF" );

   this->get_parameter("label_logs",label_logs);


}



}  // namespace composition



#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(plot2::PLOTscene2)

