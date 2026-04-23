#include <ros/package.h>
#include <ros/ros.h>
#include <dmce_msgs/RobotPosition.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <filesystem>
#include <iomanip>

struct Pose2D {
  double x{0.0}, y{0.0}, theta{0.0};
  bool valid{false};
};

static inline double deg2rad(double d) { return d * M_PI / 180.0; }

class OverlapLogger {
public:
  OverlapLogger() : nh_(), pnh_("~") {
    pnh_.param<int>("nRobots", n_robots_, 1);
    pnh_.param<int>("robot_id_start", robot_id_start_, 1);
    pnh_.param<std::string>("plannerType", planner_type_, "dmcts");
    pnh_.param<int>("run_id", run_id_, 0);
    pnh_.param<double>("max_range", max_range_m_, 8.0);
    pnh_.param<int>("num_rays", num_rays_, 720); 
    pnh_.param<double>("fov_deg", fov_deg_, 360.0);
    pnh_.param<double>("comm_range", comm_range_m_, 15.0); 
    
    nh_.param<double>("/logger/logFrequency", log_frequency_, 1.0);
    pnh_.param<int>("free_pixel_threshold", free_px_thresh_, 0); 
    pnh_.param<std::string>("pos_topic_format", pos_fmt_, "/robot%d/RobotPosition");

    pnh_.param<double>("map_offset_x", map_offset_x_, 0.0);
    pnh_.param<double>("map_offset_y", map_offset_y_, 0.0);

    if (!nh_.getParam("/globalMap/groundTruthImage", gt_image_path_)) {
      pnh_.param<std::string>("groundTruthImage", gt_image_path_, std::string(""));
    }
    if (!nh_.getParam("/globalMap/resolution", resolution_m_per_px_)) {
      pnh_.param<double>("map_resolution", resolution_m_per_px_, 0.16);
    }

    loadMap(gt_image_path_);

    poses_.resize(n_robots_);
    pos_subs_.resize(n_robots_);
    init_pos_recorded_.resize(n_robots_, false);
    last_robot_positions_.resize(n_robots_, cv::Point2d(0,0));
    
    // Initialize cumulative trackers
    cum_overlap_cells_.resize(n_robots_ + 1, 0); 
    cum_robot_cells_.resize(n_robots_, 0);

    // Initialize personal map memories for each robot
    personal_maps_.resize(n_robots_);
    for(int i = 0; i < n_robots_; i++) {
        personal_maps_[i] = cv::Mat1b(height_, width_, (uint8_t)0);
    }

    robot_colors_ = {
        cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),
        cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 255, 0),
        cv::Scalar(0, 165, 255), cv::Scalar(130, 0, 75), cv::Scalar(203, 192, 255), cv::Scalar(128, 128, 0)
    };

    for (int k = 0; k < n_robots_; k++) {
      int rid = robot_id_start_ + k;
      std::string topic = formatTopic(pos_fmt_, rid);
      pos_subs_[k] = nh_.subscribe<dmce_msgs::RobotPosition>(
        topic, 50, boost::bind(&OverlapLogger::posCb, this, _1, k)
      );
    }

    openLog();
    start_time_ = ros::Time::now().toSec();
    timer_ = nh_.createTimer(ros::Duration(1.0 / log_frequency_), &OverlapLogger::timerCb, this);
  }

private:
  static std::string formatTopic(std::string fmt, int idx) {
    std::string key = "%d"; size_t pos = 0;
    while ((pos = fmt.find(key, pos)) != std::string::npos) {
      fmt.replace(pos, key.size(), std::to_string(idx)); pos += std::to_string(idx).size();
    }
    return fmt;
  }

  void loadMap(const std::string& path) {
    gt_img_ = cv::imread(path, cv::IMREAD_GRAYSCALE); 
    if (gt_img_.empty()) throw std::runtime_error("Failed to load map");
    height_ = gt_img_.rows; width_  = gt_img_.cols;
    cv::compare(gt_img_, free_px_thresh_, traversable_, cv::CMP_EQ);
    cx_ = width_ / 2.0; cy_ = height_ / 2.0;
  }

  inline bool worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>(std::lround((wx - map_offset_x_) / resolution_m_per_px_ + cx_));
    gy = static_cast<int>(std::lround(cy_ - (wy - map_offset_y_) / resolution_m_per_px_));
    return (gx >= 0 && gx < width_ && gy >= 0 && gy < height_);
  }

  bool snapToFreeSpace(int& x, int& y, int max_radius = 5) const {
      if (traversable_(y, x) != 0) return true; 
      for (int r = 1; r <= max_radius; ++r) {
          for (int dx = -r; dx <= r; ++dx) {
              for (int dy = -r; dy <= r; ++dy) {
                  if (std::abs(dx) == r || std::abs(dy) == r) {
                      int nx = x + dx, ny = y + dy;
                      if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                          if (traversable_(ny, nx) != 0) {
                              x = nx; y = ny; return true;
                          }
                      }
                  }
              }
          }
      }
      return false; 
  }

  cv::Point getRayEndpoint(int x0, int y0, int x1, int y1) const {
    int dx = std::abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -std::abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy, x = x0, y = y0;
    cv::Point last_valid(x0, y0);
    while (true) {
      if (x < 0 || x >= width_ || y < 0 || y >= height_) break;
      if (traversable_(y, x) == 0) break; 
      last_valid = cv::Point(x, y);
      if (x == x1 && y == y1) break;
      int e2 = 2 * err;
      if (e2 >= dy) { err += dy; x += sx; }
      if (e2 <= dx) { err += dx; y += sy; }
    }
    return last_valid;
  }

  void openLog() {
    std::string base_dir = ros::package::getPath("dmce_sim") + "/logs/" + std::to_string(n_robots_) + "robots/" + planner_type_ + "/";
    std::filesystem::create_directories(base_dir);
    
    int file_index = 1;
    while (true) {
      std::stringstream ss;
      ss << "view_overlap" << std::setfill('0') << std::setw(4) << file_index << ".csv";
      log_path_ = base_dir + ss.str();
      if (!std::filesystem::exists(log_path_)) {
          std::stringstream fs;
          fs << base_dir << "frames" << std::setfill('0') << std::setw(4) << file_index << "/";
          frames_dir_ = fs.str();
          std::filesystem::create_directories(frames_dir_);
          break;
      }
      file_index++;
    }

    log_.open(log_path_, std::ios::out);
    
    // Write ALL headers
    log_ << "time_s,total_dist_m,valid_robots";
    
    // Agent instantaneous and cumulative views
    for (int i = 1; i <= n_robots_; i++) log_ << ",robot_" << i << "_cells";
    for (int i = 1; i <= n_robots_; i++) log_ << ",cum_robot_" << i << "_cells";
    
    // Multi-agent overlaps
    for (int i = 2; i <= n_robots_; i++) log_ << ",overlap_" << i << "_cells";
    for (int i = 2; i <= n_robots_; i++) log_ << ",cum_overlap_" << i;
    
    // Global metrics
    log_ << ",single_coverage,cum_single_coverage";
    log_ << ",weighted_multi_overlap,cum_weighted_multi_overlap";
    
    // Persistent Map Knowledge (The new feature)
    for (int i = 1; i <= n_robots_; i++) log_ << ",known_map_agent_" << i;
    
    log_ << "\n"; 
    log_.flush();
  }

  void posCb(const dmce_msgs::RobotPositionConstPtr& msg, int k) {
    poses_[k].x = msg->x_position; poses_[k].y = msg->y_position;
    if (!init_pos_recorded_[k]) {
      last_robot_positions_[k] = cv::Point2d(poses_[k].x, poses_[k].y);
      poses_[k].theta = 0.0; init_pos_recorded_[k] = true; poses_[k].valid = true;
      return;
    }
    double dx = poses_[k].x - last_robot_positions_[k].x;
    double dy = poses_[k].y - last_robot_positions_[k].y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist > 0.05) {
      poses_[k].theta = std::atan2(dy, dx);
      last_robot_positions_[k] = cv::Point2d(poses_[k].x, poses_[k].y);
      total_distance_covered_ += dist; 
    }
    poses_[k].valid = true; 
  }

  void timerCb(const ros::TimerEvent& event) {
    double current_time = ros::Time::now().toSec();
    int elapsed_sec = static_cast<int>(current_time - start_time_);
    if (elapsed_sec > last_logged_sec_) {
      last_logged_sec_ = elapsed_sec; 
      calculateAndLogMetrics(current_time);
    }
  }

  void calculateAndLogMetrics(double current_time) {
    cv::Mat1s temp_grid(height_, width_, (int16_t)0);
    int valid_robots = 0;
    
    std::vector<long> robot_cells(n_robots_, 0);
    std::vector<long> overlap_cells(n_robots_ + 1, 0);
    std::vector<long> known_map_counts(n_robots_, 0);
    long current_single_coverage = 0;
    long current_weighted_multi = 0;

    cv::Mat3b display_img;
    cv::cvtColor(gt_img_, display_img, cv::COLOR_GRAY2BGR);
    cv::Mat3b overlay_img(height_, width_, cv::Vec3b(0, 0, 0)); 

    // Step 1: Generate agent views, add to temp grid, and stamp into personal maps
    for (int i = 0; i < n_robots_; i++) {
      if (!poses_[i].valid) continue;
      int x0, y0;
      if (!worldToGrid(poses_[i].x, poses_[i].y, x0, y0)) continue;
      if (!snapToFreeSpace(x0, y0)) continue; 
      
      valid_robots++;
      cv::circle(display_img, cv::Point(x0, y0), 5, cv::Scalar(0, 255, 0), -1); 

      cv::Mat1b agent_view(height_, width_, (uint8_t)0);
      double start = -0.5 * deg2rad(fov_deg_);
      double step = deg2rad(fov_deg_) / (num_rays_ - 1);
      std::vector<cv::Point> fov_poly = {cv::Point(x0, y0)};

      for (int r = 0; r < num_rays_; r++) {
        double ang = poses_[i].theta + start + step * r;
        int x1, y1;
        worldToGrid(poses_[i].x + max_range_m_ * cos(ang), poses_[i].y + max_range_m_ * sin(ang), x1, y1);
        fov_poly.push_back(getRayEndpoint(x0, y0, x1, y1));
      }

      std::vector<std::vector<cv::Point>> polys{fov_poly};
      cv::fillPoly(agent_view, polys, cv::Scalar(1));
      cv::bitwise_and(agent_view, traversable_, agent_view);

      cv::Scalar color = robot_colors_[i % robot_colors_.size()];
      overlay_img.setTo(color, agent_view);

      // Add to temp grid and count instantaneous area
      for (int y = 0; y < height_; y++) {
        const uint8_t* av = agent_view.ptr<uint8_t>(y);
        int16_t* tg = temp_grid.ptr<int16_t>(y);
        for (int x = 0; x < width_; x++) {
          if (av[x]) {
              tg[x] += 1;
              robot_cells[i]++;
          }
        }
      }

      // Add instantaneous view to persistent memory
      cv::bitwise_or(personal_maps_[i], agent_view, personal_maps_[i]);
    }

    if (valid_robots < 1) return;

    double alpha = 0.5;
    cv::addWeighted(overlay_img, alpha, display_img, 1.0 - alpha, 0, display_img);

    const double elapsed_t = current_time - start_time_;
    std::stringstream ss;
    ss << "frame_" << std::fixed << std::setprecision(2) << std::setfill('0') << std::setw(8) << elapsed_t << ".png";
    cv::imwrite(frames_dir_ + ss.str(), display_img);

    // Step 2: Calculate Global Overlap Metrics from temp_grid
    for (int y = 0; y < height_; y++) {
      const uint8_t* tr = traversable_.ptr<uint8_t>(y);
      const int16_t* tg = temp_grid.ptr<int16_t>(y);
      for (int x = 0; x < width_; x++) {
        if (!tr[x]) continue; 
        
        int16_t count = tg[x];
        if (count == 1) {
            current_single_coverage++;
        }
        else if (count >= 2 && count <= n_robots_) {
            overlap_cells[count]++;
            current_weighted_multi += count;
        }
      }
    }

    // Step 3: Simulate Communication & Update Persistent Maps
    for (int i = 0; i < n_robots_; i++) {
        for (int j = i + 1; j < n_robots_; j++) {
            if (!poses_[i].valid || !poses_[j].valid) continue;
            
            double dist = std::hypot(poses_[i].x - poses_[j].x, poses_[i].y - poses_[j].y);
            if (dist <= comm_range_m_) {
                cv::Mat1b merged_mind;
                cv::bitwise_or(personal_maps_[i], personal_maps_[j], merged_mind);
                personal_maps_[i] = merged_mind.clone();
                personal_maps_[j] = merged_mind.clone();
            }
        }
    }

    // Step 4: Count total known map cells per agent
    for (int i = 0; i < n_robots_; i++) {
        known_map_counts[i] = cv::countNonZero(personal_maps_[i]);
    }

    // Update cumulative overlap counters
    for (int i = 0; i < n_robots_; i++) cum_robot_cells_[i] += robot_cells[i];
    for (int i = 2; i <= n_robots_; i++) cum_overlap_cells_[i] += overlap_cells[i];
    cum_single_coverage_ += current_single_coverage;
    cum_weighted_multi_ += current_weighted_multi;

    // Log EVERYTHING to CSV
    log_ << elapsed_t << "," << total_distance_covered_ << "," << valid_robots;
    
    // Agent instantaneous and cumulative
    for (int i = 0; i < n_robots_; i++) log_ << "," << robot_cells[i];
    for (int i = 0; i < n_robots_; i++) log_ << "," << cum_robot_cells_[i];
    
    // Overlap instantaneous and cumulative
    for (int i = 2; i <= n_robots_; i++) log_ << "," << overlap_cells[i];
    for (int i = 2; i <= n_robots_; i++) log_ << "," << cum_overlap_cells_[i];
    
    // Global metrics
    log_ << "," << current_single_coverage << "," << cum_single_coverage_;
    log_ << "," << current_weighted_multi << "," << cum_weighted_multi_;
    
    // Persistent Known Map counts
    for (int i = 0; i < n_robots_; i++) log_ << "," << known_map_counts[i];
    
    log_ << "\n";
    log_.flush();
  }

private:
  ros::NodeHandle nh_, pnh_;
  int n_robots_{1}, robot_id_start_{1}, run_id_{0}, num_rays_{720}, free_px_thresh_{0};
  double max_range_m_{8.0}, fov_deg_{360.0}, log_frequency_{1.0}, comm_range_m_{15.0}, total_distance_covered_{0.0}, start_time_{0.0}, resolution_m_per_px_{0.16}, cx_{0.0}, cy_{0.0};
  
  std::string planner_type_{"dmcts"}, pos_fmt_{"/robot%d/RobotPosition"}, gt_image_path_, log_path_, frames_dir_;
  double map_offset_x_{0.0}, map_offset_y_{0.0};
  int last_logged_sec_{-1}; 

  std::vector<Pose2D> poses_;
  std::vector<bool> init_pos_recorded_;
  std::vector<cv::Point2d> last_robot_positions_;
  std::vector<cv::Scalar> robot_colors_;
  
  // Trackers
  std::vector<long long> cum_overlap_cells_;
  std::vector<long long> cum_robot_cells_;
  long long cum_single_coverage_{0};
  long long cum_weighted_multi_{0};
  std::vector<cv::Mat1b> personal_maps_;
  
  cv::Mat1b traversable_;
  cv::Mat gt_img_; 
  int width_{0}, height_{0};
  
  std::vector<ros::Subscriber> pos_subs_;
  ros::Timer timer_; 
  std::ofstream log_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "overlap_logger");
  OverlapLogger node;
  ros::spin();
  return 0;
}
