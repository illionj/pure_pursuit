#include <algorithm>
#include <cmath>
// #include <iostream>
// #include <limits>
#include <vector>

namespace pursuit {

inline constexpr double ONE_PI = 3.14159265358979323846;
inline constexpr auto TWO_PI = ONE_PI * 2;

// using PathType = std::vector<std::tuple<double, double, double>>;
using Point2D = std::tuple<double, double>;
using PathType = std::vector<Point2D>;
using PathSumType = std::vector<double>;

double adjustAngle(double angle);

struct PurePursuitParams {
  /// 基于当前前向速度的预瞄距离增益
  /// 速度越高，预瞄距离越大，可提高跟踪平滑度
  double ld_velocity_ratio = 0.1;
  double ld_reverse_velocity_ratio = 0.1;

  /// 对航向角计算的额外增强
  double extra_reverse_streeing_angle_ratio = 0.75;
  double extra_streeing_angle_ratio = 1.8;

  /// 横向误差相关的预瞄距离增益
  /// 当车辆偏离轨迹时，根据误差增大预瞄距离，平滑回正
  double ld_lateral_error_ratio = 3.6;

  /// 基于路径曲率的预瞄距离调整增益
  /// 曲率越大，减小预瞄距离以提高拐弯精度
  double ld_curvature_ratio = 120.0;

  /// 横向误差阈值（米），当绝对误差超过该值时才启用横向误差增益
  /// 避免小抖动触发不必要的预瞄距离调整
  double long_ld_lateral_error_threshold = 0.5;

  /// 最小预瞄距离（米），防止在低速或急转弯时预瞄距离过小导致振荡
  double min_lookahead_distance = 3.35;

  /// 最大预瞄距离（米），防止高速下预瞄距离过大导致跟踪精度下降
  double max_lookahead_distance = 10.0;

  /// 转向收敛阈值（弧度），当目标转向角与当前方向盘角差值小于该值时，认为转向已收敛
  double converged_steer_rad = 0.1;

  /// 倒车时的最小预瞄距离（米），保证倒退跟踪时预瞄半径不低于该值，减少抖动
  double reverse_min_lookahead_distance = 3.5;

  /// 预测轨迹生成时的单步距离（米）
  double prediction_ds = 0.3;

  /// 预测轨迹的总长度（米）
  double prediction_distance_length = 21.0;

  /// 重采样间隔（米），在进行曲率和预瞄点计算前对路径进行等距重采样
  double resampling_ds = 0.1;

  /// 曲率计算时使用的距离跨度（米），用于在重采样点之间估算局部曲率
  double curvature_calculation_distance = 4.0;

  /// 是否启用路径平滑（移动平均滤波）
  bool enable_path_smoothing = false;

  /// 移动平均滤波窗口大小（每侧样本数），总窗口大小为 2 * N + 1
  int path_filter_moving_ave_num = 25;
};

struct VehicleParam {
  /// 车轮半径（米）
  double wheel_radius = 0.39;

  /// 车轮宽度（米）
  double wheel_width = 0.42;

  /// 轴距：前后轮中心之间的距离（米）
  double wheel_base = 2.74;

  /// 轮距：左右轮中心之间的距离（米）
  double wheel_tread = 1.63;

  /// 前悬：前轮中心到车头之间的距离（米）
  double front_overhang = 1.0;

  /// 后悬：后轮中心到车尾之间的距离（米）
  double rear_overhang = 1.03;

  /// 左悬：左轮中心到车身左侧之间的距离（米）
  double left_overhang = 0.1;

  /// 右悬：右轮中心到车身右侧之间的距离（米）
  double right_overhang = 0.1;

  /// 车高：车辆整体高度（米）
  double vehicle_height = 2.5;

  /// 最大转向角（弧度）
  double max_steer_angle = 0.70;
};

struct ControlMsg {
  /// 时间戳 暂时不用

  /// 期望的转向轮转角（弧度）
  double steering_tire_angle = 0.0f;

  /// 转向轮角速度（弧度/秒），即转角变化率
  double steering_tire_rotation_rate = 0.0f;
};

/// 车辆状态信息，包括位置、姿态、速度和后轴中心位置
struct VehicleState {
  /// 车辆在世界坐标系下的 X 轴位置（米）
  double x;

  /// 车辆在世界坐标系下的 Y 轴位置（米）
  double y;

  /// 车辆航向角（弧度），相对于世界坐标系 X 轴的偏航角
  double yaw;

  /// 车辆线速度（米/秒）
  double velocity;

  /// 车辆后轴中心在世界坐标系下的 X 轴位置（米）
  double rear_x;

  /// 车辆后轴中心在世界坐标系下的 Y 轴位置（米）
  double rear_y;

  /// 当前转向角（弧度），即前轮相对于车辆纵向的倾斜角
  double steering_angle;
};

class PurePursuitControl {
  PurePursuitParams param;
  VehicleState veh_state;
  VehicleParam veh_info;
  PathType path;
  // PathType inter_path;

  PathSumType getRoadCumDist();

  /// @brief 速度的正负决定正向行驶还是反向
  /// @return
  bool isReverseMotion() const;

  double motionHeading(const VehicleState &vs, bool reverse);

  size_t find_min_distance_point(const PathType &path, double x, double y);

  Point2D findTargetPoint(const PathSumType &cum, double rear_x, double rear_y, double ld);
  Point2D findTargetPointReverse(const PathSumType &cum, double x, double y, double ld);

  double calcLookaheadDistance(const double lateral_error,
                               const double curvature,
                               const double velocity,
                               const double min_ld);

  double calcSteeringAngle(Point2D target, double x, double y, double ld);

  double applySteeringLag(double delta_filtered,double cmd_delta, double dt);

 public:
  // debug用的 以后删
  size_t last_index = 0;

  PurePursuitControl() = default;

  void setVehcleParam(const VehicleParam &s) { veh_info = s; };
  void setPurePursuitParam(const PurePursuitParams &s) { param = s; };
  void setVehcleState(const VehicleState &s) { veh_state = s; };
  void setVehcleVelocity(double v) { veh_state.velocity = v; };

  template <typename Container>
  void setPath(const Container &pathPlanningPointList) {
    path.clear();
    auto pathLen = pathPlanningPointList.size();
    PathType temp_path;
    temp_path.reserve(pathLen);
    using Elem = typename Container::value_type;
    for (const Elem &p : pathPlanningPointList) {
      temp_path.emplace_back(p.x(), p.y());
    }
    path = std::move(temp_path);
  }

  VehicleState &getVehcleState() { return veh_state; }

  ControlMsg run();

  void updateVehicleState(const ControlMsg &msg, double dt);
};
}  // namespace pursuit