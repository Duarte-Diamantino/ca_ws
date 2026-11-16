#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <vector>
#include <cmath>
#include <algorithm>
#include <tuple>
#include <functional>
#include <mutex>
#include <memory>
#include <cstdint>
#include <map>
#include <string>

struct Pt { double x, y; };

class DwaGlobalFollower : public rclcpp::Node {
public:
  DwaGlobalFollower();

private:
  // State variables
  double x_, y_, yaw_;
  bool have_odom_;
  bool have_path_;
  bool joy_override_;
  double last_gamma_;
  double last_v_;
  int last_direction_; // 1 for forward, -1 for backward
  std::vector<Pt> path_pts_;
  std::vector<Pt> obstacles_;
  mutable std::mutex obstacles_mutex_; // Keep access thread-safe in const methods
  double min_safe_distance_;
  double lookahead_dist_;

  // Parameters
  double w_path_;
  double w_vel_;
  double w_goal_;
  double w_obs_;
  double w_gamma_;
  double w_index_;     // prioritize smaller path indices
  double w_pose_snap_; // prefer trajectories that snap onto path fast (incl. k=0)
  double horizon_s_;
  int window_pts_;
  int first_level_samples_;
  int second_level_samples_;

  // Point-hit constraint parameters
  bool require_two_hits_;
  int required_hits_;      // how many consecutive path points must be hit (default 2)
  double path_hit_radius_; // distance threshold to consider a path point "hit"
  bool relax_if_none_;     // if no candidate satisfies the hit constraint, relax it

  // Pose-on-path preference
  bool prefer_pose_on_path_;
  double pose_hit_radius_;

  // Constants
  static constexpr double DT = 0.1;
  static constexpr double V_MAX = 3.0;
  static constexpr double GAMMA_MAX = M_PI/6.0;
  static constexpr double WHEEL_BASE = 0.335;
  static constexpr double MAX_RAW = 23070.0;
  static constexpr double SERVO_C = 0.5304;
  static constexpr double SERVO_MAX = 0.94299;
  static constexpr double SERVO_MIN = 0.11781;
  static constexpr double GOAL_TOL_DEFAULT = 0.05;  // parametro "goal_tol"
  double goal_tol_ = GOAL_TOL_DEFAULT;
  // [ca_ws] Parametros novos (ver README). Por defeito o comportamento e o de antes, exceto as
  // correcoes do progresso e do fim do caminho; o simulador usa config/dwa.yaml.
  static constexpr double V_CAL = 3.0;   // calibracao do VESC: MAX_RAW corresponde a V_CAL m/s
  double v_max_ = V_MAX;                 // velocidade maxima amostrada (era sempre V_MAX)
  double obs_influence_ = 1.0e9;         // [m] a recompensa da distancia aos obstaculos satura aqui
  bool use_marker_size_ = false;         // raio do obstaculo = scale.x/2 do marker
  double car_half_width_ = 0.10;         // com use_marker_size: distancia minima = raio + isto + margem
  double obstacle_margin_ = 0.10;
  int search_ahead_ = 30;                // pontos a frente onde se procura o ponto mais proximo
  std::vector<double> obstacle_r_;       // raio de cada obstaculo (0 = sem tamanho)
  size_t progress_idx_ = 0;              // progresso ao longo do caminho (so avanca)
  bool closed_loop_ = false;             // caminho fechado (primeiro ponto ~ ultimo): da voltas
  bool finished_ = false;                // chegou ao fim: parado ate chegar um caminho novo
  std::vector<Pt> last_path_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
  double safeDistance(size_t i) const;
  size_t windowClosestIdx(const Pt& p) const;
  void updateProgress();
  // [ca_ws] Desvio do caminho a volta dos obstaculos (como o USE_REF_SHIFT do MPC). Sem isto o DWA so
  // via os obstaculos no horizonte de 1 s e o custo do caminho puxava-o para cima deles: com um
  // obstaculo em cima do caminho ficava sem trajetorias seguras e parava (ou batia).
  std::vector<Pt> raw_path_;             // caminho recebido; path_pts_ = este, desviado dos obstaculos
  bool use_ref_shift_ = true;
  double avoid_margin_ = 0.10;           // o caminho desviado passa a safeDistance + isto do obstaculo
  double avoid_ramp_ = 2.0;              // m de caminho para entrar/sair do desvio
  double avoid_default_side_ = 1.0;      // obstaculo mesmo em cima do caminho: +1 esquerda, -1 direita
  double avoid_center_tol_ = 0.05;       // |desvio lateral do obstaculo| abaixo disto = "em cima"
  std::vector<double> shift_sig_;        // obstaculos do ultimo desvio (x, y, r): so refaz se mudarem
  void rebuildPath();
  std::vector<Pt> shiftPathAroundObstacles(const std::vector<Pt>& P, const std::vector<Pt>& obs,
                                           const std::vector<double>& clear) const;
  double freeSide(const std::vector<Pt>& P, const std::vector<double>& s, size_t ic, const Pt& left,
                  double C, size_t self, const std::vector<Pt>& obs) const;

  // ROS interfaces
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obstacles_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_markers_pub_; // gradient markers for path
  rclcpp::TimerBase::SharedPtr timer_;

  // Callbacks
  void odomCb(const nav_msgs::msg::Odometry::SharedPtr msg);
  void pathCb(const nav_msgs::msg::Path::SharedPtr msg);
  void joyCb(const sensor_msgs::msg::Joy::SharedPtr msg);
  void obstaclesCb(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
  
  // Helper functions
  size_t findClosestIdx() const;
  size_t closestPathIdxToPoint(const Pt& p) const;
  std::vector<Pt> simulateTrajectory(double v, double g, int steps) const;
  bool isTrajectorySafe(const std::vector<Pt>& traj) const;
  std::pair<double, double> getLocalGoal() const;
  bool coversNextNPointsOrdered(const std::vector<Pt>& traj, size_t start_idx, int N, double radius) const; // Constraint
  double minDistanceToPath(const Pt& p) const; // distance from a point to nearest path sample
  int earliestPathHit(const std::vector<Pt>& traj, double radius) const; // first time we touch path
  double computeCost(const std::vector<Pt>& traj, const std::pair<double, double>& local_goal, double v, double g) const;
  void publishPathMarkers() const;
  void update();
};

DwaGlobalFollower::DwaGlobalFollower()
: Node("dwa_global_follower"),
  x_(0), y_(0), yaw_(0),
  have_odom_(false), have_path_(false),
  joy_override_(false), last_gamma_(0),
  last_v_(0), last_direction_(1),
  min_safe_distance_(0.5), // safety radius around obstacles
  lookahead_dist_(2.0),    // local-goal distance along path
  w_path_(2.0), w_vel_(0.5), w_goal_(2.0), w_obs_(1.0), w_gamma_(0.5),
  w_index_(3.0),           // default weight favoring smaller indices
  w_pose_snap_(3.0),       // prefer snapping onto path quickly
  horizon_s_(1.0),
  window_pts_(50),
  first_level_samples_(6),
  second_level_samples_(7),
  require_two_hits_(true),
  required_hits_(2),
  path_hit_radius_(0.18),
  relax_if_none_(true),
  prefer_pose_on_path_(true),
  pose_hit_radius_(0.18)
{
  // Declare parameters
  declare_parameter("w_path", w_path_);
  declare_parameter("w_vel", w_vel_);
  declare_parameter("w_goal", w_goal_);
  declare_parameter("w_obs", w_obs_);
  declare_parameter("w_gamma", w_gamma_);
  declare_parameter("w_index", w_index_);
  declare_parameter("w_pose_snap", w_pose_snap_);
  declare_parameter("horizon_s", horizon_s_);
  declare_parameter("lookahead_dist", lookahead_dist_);
  declare_parameter("window_pts", window_pts_);
  declare_parameter("first_level_samples", first_level_samples_);
  declare_parameter("second_level_samples", second_level_samples_);
  declare_parameter("min_safe_distance", min_safe_distance_);
  declare_parameter("require_two_hits", require_two_hits_);
  declare_parameter("required_hits", required_hits_);
  declare_parameter("path_hit_radius", path_hit_radius_);
  declare_parameter("relax_if_none", relax_if_none_);
  declare_parameter("prefer_pose_on_path", prefer_pose_on_path_);
  declare_parameter("pose_hit_radius", pose_hit_radius_);
  declare_parameter("goal_tol", goal_tol_);
  declare_parameter("v_max", v_max_);                       // [ca_ws]
  declare_parameter("obs_influence", obs_influence_);       // [ca_ws]
  declare_parameter("use_marker_size", use_marker_size_);   // [ca_ws]
  declare_parameter("car_half_width", car_half_width_);     // [ca_ws]
  declare_parameter("obstacle_margin", obstacle_margin_);   // [ca_ws]
  declare_parameter("search_ahead", search_ahead_);         // [ca_ws]
  declare_parameter("use_ref_shift", use_ref_shift_);       // [ca_ws]
  declare_parameter("avoid_margin", avoid_margin_);         // [ca_ws]
  declare_parameter("avoid_ramp", avoid_ramp_);             // [ca_ws]
  declare_parameter("avoid_default_side", avoid_default_side_);  // [ca_ws]
  declare_parameter("avoid_center_tol", avoid_center_tol_); // [ca_ws]

  // Get parameters
  get_parameter("w_path", w_path_);
  get_parameter("w_vel", w_vel_);
  get_parameter("w_goal", w_goal_);
  get_parameter("w_obs", w_obs_);
  get_parameter("w_gamma", w_gamma_);
  get_parameter("w_index", w_index_);
  get_parameter("w_pose_snap", w_pose_snap_);
  get_parameter("horizon_s", horizon_s_);
  get_parameter("lookahead_dist", lookahead_dist_);
  get_parameter("window_pts", window_pts_);
  get_parameter("first_level_samples", first_level_samples_);
  get_parameter("second_level_samples", second_level_samples_);
  get_parameter("min_safe_distance", min_safe_distance_);
  get_parameter("require_two_hits", require_two_hits_);
  get_parameter("required_hits", required_hits_);
  get_parameter("path_hit_radius", path_hit_radius_);
  get_parameter("relax_if_none", relax_if_none_);
  get_parameter("prefer_pose_on_path", prefer_pose_on_path_);
  get_parameter("pose_hit_radius", pose_hit_radius_);
  get_parameter("goal_tol", goal_tol_);
  get_parameter("v_max", v_max_);
  get_parameter("obs_influence", obs_influence_);
  get_parameter("use_marker_size", use_marker_size_);
  get_parameter("car_half_width", car_half_width_);
  get_parameter("obstacle_margin", obstacle_margin_);
  get_parameter("search_ahead", search_ahead_);
  get_parameter("use_ref_shift", use_ref_shift_);
  get_parameter("avoid_margin", avoid_margin_);
  get_parameter("avoid_ramp", avoid_ramp_);
  get_parameter("avoid_default_side", avoid_default_side_);
  get_parameter("avoid_center_tol", avoid_center_tol_);

  // [ca_ws] parametros ajustaveis com o DWA a correr (ros2 param set /dwa_global_follower v_max 1.5)
  param_cb_ = add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& ps) {
    std::map<std::string, double*> d = {
      {"w_path", &w_path_}, {"w_vel", &w_vel_}, {"w_goal", &w_goal_}, {"w_obs", &w_obs_},
      {"w_gamma", &w_gamma_}, {"w_index", &w_index_}, {"w_pose_snap", &w_pose_snap_},
      {"horizon_s", &horizon_s_}, {"lookahead_dist", &lookahead_dist_},
      {"min_safe_distance", &min_safe_distance_}, {"path_hit_radius", &path_hit_radius_},
      {"pose_hit_radius", &pose_hit_radius_}, {"goal_tol", &goal_tol_}, {"v_max", &v_max_},
      {"obs_influence", &obs_influence_}, {"car_half_width", &car_half_width_},
      {"obstacle_margin", &obstacle_margin_}, {"avoid_margin", &avoid_margin_},
      {"avoid_ramp", &avoid_ramp_}, {"avoid_default_side", &avoid_default_side_},
      {"avoid_center_tol", &avoid_center_tol_}};
    std::map<std::string, int*> i = {{"window_pts", &window_pts_}, {"required_hits", &required_hits_},
      {"first_level_samples", &first_level_samples_}, {"second_level_samples", &second_level_samples_},
      {"search_ahead", &search_ahead_}};
    std::map<std::string, bool*> b = {{"require_two_hits", &require_two_hits_},
      {"relax_if_none", &relax_if_none_}, {"prefer_pose_on_path", &prefer_pose_on_path_},
      {"use_marker_size", &use_marker_size_}, {"use_ref_shift", &use_ref_shift_}};
    rcl_interfaces::msg::SetParametersResult r;
    r.successful = true;
    for (const auto& p : ps) {
      const auto& n = p.get_name();
      if (d.count(n)) *d[n] = p.as_double();
      else if (i.count(n)) *i[n] = static_cast<int>(p.as_int());
      else if (b.count(n)) *b[n] = p.as_bool();
      else continue;
      RCLCPP_INFO(get_logger(), "parametro %s = %s", n.c_str(), p.value_to_string().c_str());
    }
    v_max_ = std::clamp(v_max_, 0.1, V_MAX);
    first_level_samples_ = std::clamp(first_level_samples_, 1, 6);
    second_level_samples_ = std::max(2, second_level_samples_);
    avoid_ramp_ = std::max(avoid_ramp_, 0.05);
    shift_sig_.clear();  // raios/margens podem ter mudado: refaz o desvio
    rebuildPath();
    return r;
  });
  avoid_ramp_ = std::max(avoid_ramp_, 0.05);
  v_max_ = std::clamp(v_max_, 0.1, V_MAX);
  first_level_samples_ = std::clamp(first_level_samples_, 1, 6);
  second_level_samples_ = std::max(2, second_level_samples_);

  // Subscribers
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&DwaGlobalFollower::odomCb, this, std::placeholders::_1));
  
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/reference_path", 10,
    std::bind(&DwaGlobalFollower::pathCb, this, std::placeholders::_1));
  
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
    "/joy", 10,
    std::bind(&DwaGlobalFollower::joyCb, this, std::placeholders::_1));
  
  obstacles_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
    "/obstacles_marker", 10,
    std::bind(&DwaGlobalFollower::obstaclesCb, this, std::placeholders::_1));

  // Publishers
  speed_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/motor/speed", 10);
  steer_pub_ = create_publisher<std_msgs::msg::Float64>("/commands/servo/position", 10);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/predicted_trajectories", 10);
  path_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/reference_path_markers", 10);

  // Timer
  timer_ = create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(DT * 1000)),
    std::bind(&DwaGlobalFollower::update, this));
}

void DwaGlobalFollower::odomCb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  x_ = msg->pose.pose.position.x;
  y_ = msg->pose.pose.position.y;
  auto& q = msg->pose.pose.orientation;
  yaw_ = std::atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z));
  have_odom_ = true;
}

void DwaGlobalFollower::pathCb(const nav_msgs::msg::Path::SharedPtr msg) {
  // Reorder the received path by an inferred "ordem": prefer header.seq, then timestamp, then fallback to arrival index
  struct OrderedPt { Pt pt; long long ordem; };
  std::vector<OrderedPt> ordered;
  ordered.reserve(msg->poses.size());

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    const auto &ps = msg->poses[i];
    long long ordem = 0;
    /* ROS 2 Humble: std_msgs/Header has no 'seq'. Use stamp or arrival index instead. */
    if (ordem == 0) {
      long long stamp = static_cast<long long>(ps.header.stamp.sec) * 1000000000LL + ps.header.stamp.nanosec;
      if (stamp > 0) ordem = stamp;
    }
    if (ordem == 0) {
      ordem = static_cast<long long>(i);
    }
    ordered.push_back({{ps.pose.position.x, ps.pose.position.y}, ordem});
  }

  // stable_sort: com stamps iguais (q_path_publisher, marker_publisher...) o std::sort baralhava
  // o caminho a partir de 17 pontos. stable_sort mantem a ordem de chegada nos empates.
  std::stable_sort(ordered.begin(), ordered.end(), [](const OrderedPt &a, const OrderedPt &b){
    return a.ordem < b.ordem;
  });

  raw_path_.clear();
  raw_path_.reserve(ordered.size());
  for (const auto &op : ordered) raw_path_.push_back(op.pt);
  have_path_ = !raw_path_.empty();

  // [ca_ws] o GUI/q_path republicam o mesmo caminho (1-10 Hz): so um caminho NOVO repoe o progresso
  bool same = last_path_.size() == raw_path_.size() && path_pts_.size() == raw_path_.size();
  for (size_t k = 0; same && k < raw_path_.size(); k += std::max<size_t>(1, raw_path_.size() / 16)) {
    same = std::hypot(last_path_[k].x - raw_path_[k].x, last_path_[k].y - raw_path_[k].y) < 1e-3;
  }
  if (!same && have_path_) {
    last_path_ = raw_path_;
    shift_sig_.clear();
    rebuildPath();  // path_pts_ = raw_path_ desviado dos obstaculos (mesmos indices)
    closed_loop_ = raw_path_.size() > 10 &&
      std::hypot(raw_path_.front().x - raw_path_.back().x, raw_path_.front().y - raw_path_.back().y) < 0.5;
    progress_idx_ = have_odom_ ? closestPathIdxToPoint({x_, y_}) : 0;
    finished_ = false;
    RCLCPP_INFO(get_logger(), "Caminho novo: %zu pontos%s", path_pts_.size(), closed_loop_ ? " (volta fechada)" : "");
  }

  // Publish gradient markers along the ordered path to visualize index/order
  publishPathMarkers();
}

void DwaGlobalFollower::joyCb(const sensor_msgs::msg::Joy::SharedPtr msg) {
  joy_override_ = std::any_of(
    msg->buttons.begin(), msg->buttons.end(), [](int b){return b>0;});
}

void DwaGlobalFollower::obstaclesCb(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    obstacles_.clear();
    obstacle_r_.clear();

    for (const auto& marker : msg->markers) {
      // [ca_ws] so ADD: o DELETEALL que a GUI manda antes dos cilindros (ns "obstacles", pose 0) era
      // lido como um obstaculo em (0, 0)
      if (marker.action != visualization_msgs::msg::Marker::ADD) continue;
      if (marker.ns == "obstacle" || marker.ns == "obstacles") {
        obstacles_.push_back({marker.pose.position.x, marker.pose.position.y});
        obstacle_r_.push_back(marker.scale.x > 0 ? 0.5 * marker.scale.x : 0.0);  // [ca_ws]
      }
    }
  }
  rebuildPath();  // [ca_ws] so refaz o desvio se os obstaculos mudaram
}

// [ca_ws] path_pts_ = raw_path_ desviado dos obstaculos. Os indices sao os mesmos (cada ponto so anda
// para o lado), por isso o progresso continua valido quando os obstaculos mudam.
void DwaGlobalFollower::rebuildPath() {
  if (raw_path_.empty()) return;
  std::vector<Pt> obs;
  std::vector<double> clear, sig;
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    for (size_t i = 0; i < obstacles_.size(); ++i) {
      obs.push_back(obstacles_[i]);
      clear.push_back(safeDistance(i) + avoid_margin_);
      sig.insert(sig.end(), {obstacles_[i].x, obstacles_[i].y, clear.back()});
    }
  }
  sig.push_back(use_ref_shift_ ? 1.0 : 0.0);
  bool same = sig.size() == shift_sig_.size() && path_pts_.size() == raw_path_.size();
  for (size_t k = 0; same && k < sig.size(); ++k) same = std::abs(sig[k] - shift_sig_[k]) < 1e-4;
  if (same) return;
  shift_sig_ = sig;
  path_pts_ = (use_ref_shift_ && !obs.empty() && raw_path_.size() > 2)
    ? shiftPathAroundObstacles(raw_path_, obs, clear) : raw_path_;
  if (progress_idx_ >= path_pts_.size()) progress_idx_ = path_pts_.size() - 1;
  double max_off = 0.0;
  for (size_t k = 0; k < path_pts_.size(); ++k)
    max_off = std::max(max_off, std::hypot(path_pts_[k].x - raw_path_[k].x, path_pts_[k].y - raw_path_[k].y));
  if (max_off > 1e-3)
    RCLCPP_INFO(get_logger(), "Caminho desviado dos obstaculos (ate %.2f m para o lado)", max_off);
  publishPathMarkers();
}

// [ca_ws] Igual ao shift_path_around_obstacles do MPC. Em cada passagem do caminho perto de um obstaculo
// (a menos de C = distancia de seguranca + avoid_margin), no ponto mais proximo mede-se o desvio lateral
// do obstaculo (lat > 0: a esquerda) e o caminho e empurrado para o lado contrario ate ficar a C dele:
// deslocamento total enquanto o caminho original estaria a < C e rampa em cosseno de avoid_ramp m antes
// e depois. Com varios obstaculos fica, em cada ponto, o maior deslocamento.
std::vector<Pt> DwaGlobalFollower::shiftPathAroundObstacles(const std::vector<Pt>& P, const std::vector<Pt>& obs,
                                                            const std::vector<double>& clear) const {
  const size_t n = P.size();
  std::vector<double> s(n, 0.0);
  for (size_t i = 1; i < n; ++i) s[i] = s[i - 1] + std::hypot(P[i].x - P[i - 1].x, P[i].y - P[i - 1].y);
  std::vector<Pt> left(n);
  for (size_t i = 0; i < n; ++i) {
    size_t a = i == 0 ? 0 : i - 1, b = i + 1 < n ? i + 1 : n - 1;   // diferencas centrais (np.gradient)
    double tx = P[b].x - P[a].x, ty = P[b].y - P[a].y;
    double L = std::max(std::hypot(tx, ty), 1e-9);
    left[i] = {-ty / L, tx / L};
  }
  std::vector<double> offset(n, 0.0), d(n);
  for (size_t o = 0; o < obs.size(); ++o) {
    const double C = clear[o];
    for (size_t i = 0; i < n; ++i) d[i] = std::hypot(P[i].x - obs[o].x, P[i].y - obs[o].y);
    size_t i = 0;
    while (i < n) {
      if (d[i] >= C) { ++i; continue; }
      size_t j = i, ic = i;                       // passagem = pontos seguidos a menos de C
      for (; j < n && d[j] < C; ++j) if (d[j] < d[ic]) ic = j;
      double lat = (obs[o].x - P[ic].x) * left[ic].x + (obs[o].y - P[ic].y) * left[ic].y;
      double side = std::abs(lat) > avoid_center_tol_ ? -std::copysign(1.0, lat)
                                                      : freeSide(P, s, ic, left[ic], C, o, obs);
      double amount = lat + side * C;             // o ponto fica a C do obstaculo, do lado "side"
      double hold = std::sqrt(std::max(C * C - lat * lat, 0.0));
      for (size_t k = 0; k < n; ++k) {
        double ds = std::abs(s[k] - s[ic]);
        double u = std::clamp((ds - hold) / avoid_ramp_, 0.0, 1.0);
        double c = ds <= hold ? amount : amount * 0.5 * (1.0 + std::cos(M_PI * u));
        if (std::abs(c) > std::abs(offset[k])) offset[k] = c;
      }
      i = j;
    }
  }
  std::vector<Pt> Q(n);
  for (size_t k = 0; k < n; ++k) Q[k] = {P[k].x + offset[k] * left[k].x, P[k].y + offset[k] * left[k].y};
  return Q;
}

// [ca_ws] Obstaculo mesmo em cima do caminho: o lado com mais espaco (mais longe de outras partes do
// caminho, ex. o fim do Q que volta a reta inicial, e dos outros obstaculos). Empate -> avoid_default_side.
double DwaGlobalFollower::freeSide(const std::vector<Pt>& P, const std::vector<double>& s, size_t ic,
                                   const Pt& left, double C, size_t self, const std::vector<Pt>& obs) const {
  const double sides[2] = {1.0, -1.0};
  double room[2];
  for (int k = 0; k < 2; ++k) {
    double cx = P[ic].x + sides[k] * C * left.x, cy = P[ic].y + sides[k] * C * left.y;
    double r = 1e18;
    for (size_t i = 0; i < P.size(); ++i)
      if (std::abs(s[i] - s[ic]) > 2.0 * (C + avoid_ramp_)) r = std::min(r, std::hypot(P[i].x - cx, P[i].y - cy));
    for (size_t o = 0; o < obs.size(); ++o)
      if (o != self) r = std::min(r, std::hypot(obs[o].x - cx, obs[o].y - cy));
    room[k] = r;
  }
  if (std::abs(room[0] - room[1]) < 0.05 || std::min(room[0], room[1]) > 2.0)
    return avoid_default_side_ >= 0.0 ? 1.0 : -1.0;
  return room[0] > room[1] ? 1.0 : -1.0;
}

// [ca_ws] Era o ponto mais proximo de TODO o caminho: num caminho que passa duas vezes perto do mesmo
// sitio (ou cujo fim volta ao inicio) saltava para o outro troco. Agora e o progresso, que so avanca
// (updateProgress, procura so search_ahead pontos a frente).
size_t DwaGlobalFollower::findClosestIdx() const {
  return std::min(progress_idx_, path_pts_.empty() ? 0 : path_pts_.size() - 1);
}

size_t DwaGlobalFollower::windowClosestIdx(const Pt& p) const {
  size_t N = path_pts_.size(), best = progress_idx_;
  double md = 1e18;
  for (int k = 0; k <= search_ahead_ && N > 0; ++k) {
    size_t i = closed_loop_ ? (progress_idx_ + k) % N : std::min(progress_idx_ + k, N - 1);
    double d = std::hypot(path_pts_[i].x - p.x, path_pts_[i].y - p.y);
    if (d < md) { md = d; best = i; }
  }
  return best;
}

void DwaGlobalFollower::updateProgress() {
  if (!path_pts_.empty()) progress_idx_ = windowClosestIdx({x_, y_});
}

double DwaGlobalFollower::safeDistance(size_t i) const {
  // [ca_ws] com use_marker_size: o obstaculo e o que e (raio do marker) + meia largura do carro + margem
  if (use_marker_size_ && i < obstacle_r_.size() && obstacle_r_[i] > 0.0)
    return obstacle_r_[i] + car_half_width_ + obstacle_margin_;
  return min_safe_distance_;
}

size_t DwaGlobalFollower::closestPathIdxToPoint(const Pt& p) const {
  if (path_pts_.empty()) return 0;
  size_t idx = 0;
  double md = 1e9;
  for (size_t i = 0; i < path_pts_.size(); ++i) {
    double d = std::hypot(path_pts_[i].x - p.x, path_pts_[i].y - p.y);
    if (d < md) { md = d; idx = i; }
  }
  return idx;
}

std::vector<Pt> DwaGlobalFollower::simulateTrajectory(double v, double g, int steps) const {
  std::vector<Pt> traj;
  traj.reserve(steps+1);
  double sx = x_, sy = y_, syaw = yaw_;
  for(int k = 0; k <= steps; ++k) {
    traj.push_back({sx, sy});
    sx += v * std::cos(syaw) * DT;
    sy += v * std::sin(syaw) * DT;
    double gamma_effective = (v >= 0) ? g : -g;
    syaw += (v / WHEEL_BASE) * std::tan(gamma_effective) * DT;
  }
  return traj;
}

bool DwaGlobalFollower::isTrajectorySafe(const std::vector<Pt>& traj) const {
  std::lock_guard<std::mutex> lock(obstacles_mutex_);
  
  for (const auto& traj_pt : traj) {
    for (size_t oi = 0; oi < obstacles_.size(); ++oi) {
      const auto& obs = obstacles_[oi];
      double dx = traj_pt.x - obs.x;
      double dy = traj_pt.y - obs.y;
      double distance = std::hypot(dx, dy);
      
      if (distance < safeDistance(oi)) {  // [ca_ws] era min_safe_distance_ para todos
        return false;
      }
    }
  }
  return true;
}

std::pair<double, double> DwaGlobalFollower::getLocalGoal() const {
  if (path_pts_.empty()) {
    return {x_, y_};
  }
  
  size_t closest_idx = findClosestIdx();
  double accumulated_dist = 0.0;
  
  size_t N = path_pts_.size();
  size_t steps_max = closed_loop_ ? N : N - 1 - std::min(closest_idx, N - 1);  // [ca_ws] volta fechada
  for (size_t s = 0; s < steps_max; s++) {
    size_t i = (closest_idx + s) % N, j = (i + 1) % N;
    double dx = path_pts_[j].x - path_pts_[i].x;
    double dy = path_pts_[j].y - path_pts_[i].y;
    double seg_dist = std::hypot(dx, dy);
    
    if (accumulated_dist + seg_dist >= lookahead_dist_) {
      double ratio = (lookahead_dist_ - accumulated_dist) / seg_dist;
      double goal_x = path_pts_[i].x + ratio * dx;
      double goal_y = path_pts_[i].y + ratio * dy;
      return {goal_x, goal_y};
    }
    
    accumulated_dist += seg_dist;
  }
  
  return {path_pts_.back().x, path_pts_.back().y};
}

bool DwaGlobalFollower::coversNextNPointsOrdered(const std::vector<Pt>& traj, size_t start_idx, int N, double radius) const {
  if (path_pts_.empty() || N <= 0) return true;
  size_t last_traj_hit = 0;
  size_t end_idx = std::min(path_pts_.size(), start_idx + static_cast<size_t>(N));
  for (size_t j = start_idx; j < end_idx; ++j) {
    bool hit = false;
    for (size_t ti = last_traj_hit; ti < traj.size(); ++ti) {
      if (std::hypot(traj[ti].x - path_pts_[j].x, traj[ti].y - path_pts_[j].y) <= radius) {
        last_traj_hit = ti + 1; // enforce order along trajectory
        hit = true;
        break;
      }
    }
    if (!hit) return false; // failed to hit this path point in order
  }
  return true;
}

// Helpers for pose-on-path snap preference

double DwaGlobalFollower::minDistanceToPath(const Pt& p) const {
  double md = 1e9;
  for (const auto& q : path_pts_) {
    double d = std::hypot(p.x - q.x, p.y - q.y);
    if (d < md) md = d;
  }
  return md;
}

int DwaGlobalFollower::earliestPathHit(const std::vector<Pt>& traj, double radius) const {
  for (size_t k = 0; k < traj.size(); ++k) {
    if (minDistanceToPath(traj[k]) <= radius) return static_cast<int>(k);
  }
  return -1; // never touched within radius
}

void DwaGlobalFollower::publishPathMarkers() const {
  if (!have_path_ || path_pts_.empty()) return;

  visualization_msgs::msg::MarkerArray ma;
  auto now = this->now();

  for (size_t i = 0; i < path_pts_.size(); ++i) {
    double t = (path_pts_.size() > 1) ? (static_cast<double>(i) / (path_pts_.size() - 1)) : 0.0; // 0..1
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now;
    m.ns = "reference_path_points";
    m.id = static_cast<int>(i);
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = path_pts_[i].x;
    m.pose.position.y = path_pts_[i].y;
    m.pose.position.z = 0.0;
    m.scale.x = 0.07; // small dots on top of the path
    m.scale.y = 0.07;
    m.scale.z = 0.07;
    // Blue (low index) -> Red (high index)
    m.color.r = t;
    m.color.g = 0.0;
    m.color.b = 1.0 - t;
    m.color.a = 0.9;
    ma.markers.push_back(m);
  }

  path_markers_pub_->publish(ma);
}

double DwaGlobalFollower::computeCost(const std::vector<Pt>& traj, 
                                     const std::pair<double, double>& local_goal, 
                                     double v, double g) const {
  double cost = 0.0;
  
  // 1. Path following cost (distance to path points)
  if (!path_pts_.empty()) {
    double sumd = 0;
    size_t N = path_pts_.size();
    size_t start = findClosestIdx();
    for(int k = 0; k < window_pts_; ++k) {
      // [ca_ws] era (start + k) % N: perto do fim de um caminho aberto a janela dava a volta para os
      // primeiros pontos e puxava o carro de volta ao inicio (repetia o percurso)
      const Pt &t = path_pts_[closed_loop_ ? (start + k) % N : std::min(start + k, N - 1)];
      double dmin = 1e9;
      for(const auto &p : traj) {
        dmin = std::min(dmin, std::hypot(p.x - t.x, p.y - t.y));
      }
      sumd += dmin;
    }
    cost += -w_path_ * sumd;
  }
  
  // 2. Goal distance cost
  double dx_end = traj.back().x - local_goal.first;
  double dy_end = traj.back().y - local_goal.second;
  double dist_to_goal = std::hypot(dx_end, dy_end);
  cost += -w_goal_ * dist_to_goal;
  
  // 3. Obstacle clearance cost (maximize min distance)
  double min_obstacle_dist = 1e9;
  {
    std::lock_guard<std::mutex> lock(obstacles_mutex_);
    for (const auto& pt : traj) {
      for (const auto& obs : obstacles_) {
        double d = std::hypot(pt.x - obs.x, pt.y - obs.y);
        if (d < min_obstacle_dist) min_obstacle_dist = d;
      }
    }
  }
  cost += w_obs_ * std::min(min_obstacle_dist, obs_influence_);  // [ca_ws] longe = indiferente
  
  // 4. Velocity cost (prefer lower |v| to be smoother)
  cost += -w_vel_ * std::abs(v);
  
  // 5. Steering change penalty (prefer not to jerk)
  cost -= w_gamma_ * std::abs(g - last_gamma_);
  
  // 6. Direction change penalty
  if ((v >= 0) != (last_v_ >= 0)) {
    cost -= 1000.0;
  }

  // 7. Index-bias cost — favor trajectories that end near LOWER path indices
  if (!path_pts_.empty()) {
    size_t idx_end = windowClosestIdx(traj.back());  // [ca_ws] era o mais proximo de todo o caminho
    double denom = std::max<size_t>(1, path_pts_.size() - 1);
    double idx_score = 1.0 - static_cast<double>(idx_end) / static_cast<double>(denom);
    cost += w_index_ * idx_score;
  }

  // 8. Pose-on-path snap reward — reward trajectories that touch the path early (k=0 best)
  if (!path_pts_.empty() && prefer_pose_on_path_) {
    int k = earliestPathHit(traj, pose_hit_radius_);
    if (k >= 0) {
      int steps = std::max(1, (int)std::round(horizon_s_ / DT));
      double score = 1.0 - static_cast<double>(k) / static_cast<double>(steps); // 1 at k=0 .. 0 at end
      cost += w_pose_snap_ * score;
    }
  }
  
  return cost;
}

void DwaGlobalFollower::update() {
  if(!have_odom_ || !have_path_ || joy_override_ || path_pts_.empty()) {
    return;
  }

  updateProgress();  // [ca_ws]
  // Check if reached final goal
  // [ca_ws] Antes: so a distancia ao ultimo ponto < goal_tol. A alta velocidade o carro passava o ponto
  // sem entrar na janela e repetia o percurso. Agora tambem acaba quando o progresso chega ao fim, e
  // fica parado (com a direcao ao centro) ate chegar um caminho novo. Caminho fechado: nunca acaba.
  // A distancia ao ultimo ponto so conta depois de 80% do caminho (como o MPC): o caminho Q acaba em
  // cima da reta inicial e o carro "chegava" logo no arranque.
  double dist_to_goal = std::hypot(x_ - path_pts_.back().x, y_ - path_pts_.back().y);
  bool near_end = progress_idx_ >= static_cast<size_t>(0.8 * (path_pts_.size() - 1));
  if (!closed_loop_ && (finished_ || (near_end && dist_to_goal < goal_tol_) || progress_idx_ + 1 >= path_pts_.size())) {
    if (!finished_) RCLCPP_INFO(get_logger(), "Fim do caminho: carro parado (a espera de um caminho novo).");
    finished_ = true;
    std_msgs::msg::Float64 stop_speed;
    stop_speed.data = 0.0;
    speed_pub_->publish(stop_speed);
    std_msgs::msg::Float64 center;
    center.data = SERVO_C;
    steer_pub_->publish(center);
    marker_pub_->publish(visualization_msgs::msg::MarkerArray());
    return;
  }

  // Get local goal
  auto local_goal = getLocalGoal();
  size_t start_idx = findClosestIdx();

  // Prepare visualization
  visualization_msgs::msg::MarkerArray ma;
  auto now = this->now();
  int id = 0;
  int steps = static_cast<int>(horizon_s_ / DT);

  // 1. Add green ball at robot position
  visualization_msgs::msg::Marker robot_marker;
  robot_marker.header.frame_id = "map";
  robot_marker.header.stamp = now;
  robot_marker.ns = "robot_position";
  robot_marker.id = id++;
  robot_marker.type = visualization_msgs::msg::Marker::SPHERE;
  robot_marker.action = visualization_msgs::msg::Marker::ADD;
  robot_marker.pose.position.x = x_;
  robot_marker.pose.position.y = y_;
  robot_marker.pose.position.z = 0.0;
  robot_marker.scale.x = 0.3;
  robot_marker.scale.y = 0.3;
  robot_marker.scale.z = 0.3;
  robot_marker.color.r = 0.0;
  robot_marker.color.g = 1.0;
  robot_marker.color.b = 0.0;
  robot_marker.color.a = 1.0;
  ma.markers.push_back(robot_marker);

  // 2. First level: main directions
  std::vector<std::tuple<double, double, double, std::vector<Pt>>> first_level;
  bool any_with_hits_first = false;
  
  // Main directions (forward and backward)
  std::vector<std::pair<double, double>> directions = {
    {v_max_, 0.0},          // Forward straight      [ca_ws] v_max_ (era V_MAX = 3 m/s fixo)
    {v_max_, GAMMA_MAX/2},  // Forward left
    {v_max_, -GAMMA_MAX/2}, // Forward right
    {-v_max_, 0.0},         // Backward straight
    {-v_max_, GAMMA_MAX/2}, // Backward left
    {-v_max_, -GAMMA_MAX/2} // Backward right
  };

  for(int i = 0; i < first_level_samples_; ++i) {
    double v = directions[i].first;
    double g = directions[i].second;
    auto traj = simulateTrajectory(v, g, steps);
    
    // Check safety
    if (!isTrajectorySafe(traj)) {
      // Visualize unsafe trajectories in red
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now;
      m.ns = "unsafe_traj";
      m.id = id++;
      m.type = m.LINE_STRIP;
      m.action = m.ADD;
      m.scale.x = 0.02;
      m.color.r = 1.0;
      m.color.g = 0.0;
      m.color.b = 0.0;
      m.color.a = 0.6;
      for(auto &p : traj) {
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0;
        m.points.push_back(point);
      }
      ma.markers.push_back(m);
      continue;
    }

    // Check ordered 2-point coverage
    bool cover_ok = !require_two_hits_ || coversNextNPointsOrdered(traj, start_idx, required_hits_, path_hit_radius_);
    if (!cover_ok) {
      // Visualize skipped-coverage trajectories in orange
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp = now;
      m.ns = "skip_traj";
      m.id = id++;
      m.type = m.LINE_STRIP;
      m.action = m.ADD;
      m.scale.x = 0.02;
      m.color.r = 1.0;
      m.color.g = 0.5;
      m.color.b = 0.0;
      m.color.a = 0.6;
      for(auto &p : traj) {
        geometry_msgs::msg::Point point;
        point.x = p.x;
        point.y = p.y;
        point.z = 0;
        m.points.push_back(point);
      }
      ma.markers.push_back(m);
    } else {
      any_with_hits_first = true;
      double cost = computeCost(traj, local_goal, v, g);
      first_level.emplace_back(cost, v, g, traj);
    }
  }

  // If none satisfied the coverage, we allow all safe ones
  if (!any_with_hits_first && relax_if_none_) {
    first_level.clear();
    for(int i = 0; i < first_level_samples_; ++i) {
      double v = directions[i].first;
      double g = directions[i].second;
      auto traj = simulateTrajectory(v, g, steps);
      if (!isTrajectorySafe(traj)) continue;
      double cost = computeCost(traj, local_goal, v, g);
      first_level.emplace_back(cost, v, g, traj);
    }
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Relaxing two-hit constraint at first level (blocked segment?)");
  }

  // Sort by cost (best first)
  std::sort(first_level.begin(), first_level.end(), 
    [](const auto& a, const auto& b){ return std::get<0>(a) > std::get<0>(b); });

  // Visualize best first-level trajectories (yellow)
  for(size_t i = 0; i < 2 && i < first_level.size(); ++i) {
    const auto& traj = std::get<3>(first_level[i]);
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = now;
    m.ns = "main_traj";
    m.id = id++;
    m.type = m.LINE_STRIP;
    m.action = m.ADD;
    m.scale.x = 0.03;
    m.color.r = 1.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.8;
    for(auto &p : traj) {
      geometry_msgs::msg::Point point;
      point.x = p.x;
      point.y = p.y;
      point.z = 0;
      m.points.push_back(point);
    }
    ma.markers.push_back(m);
  }

  // 3. Second level: refinement around best directions
  double best_cost = -1e9;
  double best_v = 0;
  double best_g = 0;
  std::vector<Pt> best_traj;
  bool any_with_hits_second = false;

  for(size_t i = 0; i < 2 && i < first_level.size(); ++i) {
    double base_v = std::get<1>(first_level[i]);
    double base_g = std::get<2>(first_level[i]);
    
    double v_range = (base_v > 0) ? v_max_/2 : v_max_/3;
    double g_range = GAMMA_MAX/2;
    
    double dv = 2 * v_range / (second_level_samples_ - 1);
    double dg = 2 * g_range / (second_level_samples_ - 1);
    
    for(int iv = 0; iv < second_level_samples_; ++iv) {
      double v = std::clamp(base_v - v_range + iv * dv, -v_max_, v_max_);
      for(int ig = 0; ig < second_level_samples_; ++ig) {
        double g = std::clamp(base_g - g_range + ig * dg, -GAMMA_MAX, GAMMA_MAX);
        
        auto traj = simulateTrajectory(v, g, steps);
        
        // Skip unsafe trajectories
        if (!isTrajectorySafe(traj)) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = "map";
          m.header.stamp = now;
          m.ns = "unsafe_traj";
          m.id = id++;
          m.type = m.LINE_STRIP;
          m.action = m.ADD;
          m.scale.x = 0.015;
          m.color.r = 1.0;
          m.color.g = 0.0;
          m.color.b = 0.0;
          m.color.a = 0.4;
          for(auto &p : traj) {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0;
            m.points.push_back(point);
          }
          ma.markers.push_back(m);
          continue;
        }

        // Check ordered two-hit coverage
        bool cover_ok = !require_two_hits_ || coversNextNPointsOrdered(traj, start_idx, required_hits_, path_hit_radius_);
        if (!cover_ok) {
          visualization_msgs::msg::Marker m;
          m.header.frame_id = "map";
          m.header.stamp = now;
          m.ns = "skip_traj";
          m.id = id++;
          m.type = m.LINE_STRIP;
          m.action = m.ADD;
          m.scale.x = 0.015;
          m.color.r = 1.0;
          m.color.g = 0.5;
          m.color.b = 0.0;
          m.color.a = 0.4;
          for(auto &p : traj) {
            geometry_msgs::msg::Point point;
            point.x = p.x;
            point.y = p.y;
            point.z = 0;
            m.points.push_back(point);
          }
          ma.markers.push_back(m);
          continue;
        }
        any_with_hits_second = true;
        
        double cost = computeCost(traj, local_goal, v, g);
        
        // Visualize safe trajectories (green)
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = now;
        m.ns = "second_level";
        m.id = id++;
        m.type = m.LINE_STRIP;
        m.action = m.ADD;
        m.scale.x = 0.015;
        m.color.r = 0.0;
        m.color.g = 1.0;
        m.color.b = 0.0;
        m.color.a = 0.4;
        for(auto &p : traj) {
          geometry_msgs::msg::Point point;
          point.x = p.x;
          point.y = p.y;
          point.z = 0;
          m.points.push_back(point);
        }
        ma.markers.push_back(m);
        
        if(cost > best_cost) {
          best_cost = cost;
          best_v = v;
          best_g = g;
          best_traj = traj;
        }
      }
    }
  }

  // Relaxation fallback if nothing satisfied the hit constraint
  if (!any_with_hits_second && relax_if_none_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Relaxing two-hit constraint at second level (blocked segment?)");
    best_cost = -1e9;
    best_traj.clear();
    for(size_t i = 0; i < 2 && i < first_level.size(); ++i) {
      double base_v = std::get<1>(first_level[i]);
      double base_g = std::get<2>(first_level[i]);
      double v_range = (base_v > 0) ? v_max_/2 : v_max_/3;
      double g_range = GAMMA_MAX/2;
      double dv = 2 * v_range / (second_level_samples_ - 1);
      double dg = 2 * g_range / (second_level_samples_ - 1);
      for(int iv = 0; iv < second_level_samples_; ++iv) {
        double v = std::clamp(base_v - v_range + iv * dv, -v_max_, v_max_);
        for(int ig = 0; ig < second_level_samples_; ++ig) {
          double g = std::clamp(base_g - g_range + ig * dg, -GAMMA_MAX, GAMMA_MAX);
          auto traj = simulateTrajectory(v, g, steps);
          if (!isTrajectorySafe(traj)) continue;
          double cost = computeCost(traj, local_goal, v, g);
          if (cost > best_cost) {
            best_cost = cost;
            best_v = v;
            best_g = g;
            best_traj = traj;
          }
        }
      }
    }
  }

  // Update state for next iteration
  last_gamma_ = best_g;
  last_v_ = best_v;
  last_direction_ = (best_v >= 0) ? 1 : -1;

  // Visualize best trajectory (cyan)
  if (!best_traj.empty()) {
    visualization_msgs::msg::Marker mb;
    mb.header.frame_id = "map";
    mb.header.stamp = now;
    mb.ns = "best_traj";
    mb.id = id++;
    mb.type = mb.LINE_STRIP;
    mb.action = mb.ADD;
    mb.scale.x = 0.04;
    mb.color.r = 0;
    mb.color.g = 1;
    mb.color.b = 1;
    mb.color.a = 1;
    for (auto &p : best_traj) {
      geometry_msgs::msg::Point P;
      P.x = p.x;
      P.y = p.y;
      P.z = 0;
      mb.points.push_back(P);
    }
    ma.markers.push_back(mb);
  }

  // Visualize local goal (purple)
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = "map";
  goal_marker.header.stamp = now;
  goal_marker.ns = "local_goal";
  goal_marker.id = id++;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position.x = local_goal.first;
  goal_marker.pose.position.y = local_goal.second;
  goal_marker.pose.position.z = 0;
  goal_marker.scale.x = 0.2;
  goal_marker.scale.y = 0.2;
  goal_marker.scale.z = 0.2;
  goal_marker.color.r = 1.0;
  goal_marker.color.g = 0.0;
  goal_marker.color.b = 1.0;
  goal_marker.color.a = 1.0;
  ma.markers.push_back(goal_marker);

  marker_pub_->publish(ma);

  // Publish commands
  if (!best_traj.empty()) {
    std_msgs::msg::Float64 sp, st;
    sp.data = std::clamp(best_v / V_CAL * MAX_RAW, -MAX_RAW, MAX_RAW);  // [ca_ws] calibracao fixa (era / V_MAX)
    
    double steer_angle = (best_v >= 0) ? -best_g : best_g;
    double span = SERVO_MAX - SERVO_C;
    st.data = std::clamp(SERVO_C + (steer_angle / GAMMA_MAX) * span,
                        SERVO_MIN, SERVO_MAX);
    
    speed_pub_->publish(sp);
    steer_pub_->publish(st);
  } else {
    // [ca_ws] nenhuma trajetoria segura: antes nao publicava nada e o VESC ficava com o ultimo comando
    // (o carro seguia para o obstaculo). Agora trava.
    std_msgs::msg::Float64 stop_speed;
    stop_speed.data = 0.0;
    speed_pub_->publish(stop_speed);
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Sem trajetoria segura (obstaculo demasiado perto): carro parado");
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DwaGlobalFollower>());
  rclcpp::shutdown();
  return 0;
}
