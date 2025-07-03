#include "pure_pursuit.h"

#include <algorithm>
namespace pursuit {

double adjustAngle(double angle) {
  auto adjusted_angle = std::fmod(angle + ONE_PI, TWO_PI);
  if (adjusted_angle < 0.0) adjusted_angle += TWO_PI;
  return adjusted_angle - ONE_PI;
}

PathSumType PurePursuitControl::getRoadCumDist() {
  PathSumType cum_dist;
  auto pathLen = path.size();
  cum_dist.resize(pathLen);
  cum_dist[0] = 0;
  auto current_cum = cum_dist[0];
  for (int i = 0; i < pathLen - 1; i++) {
    auto [px, py] = path[i];
    auto [next_px, next_py] = path[i + 1];
    auto path_dx = next_px - px;
    auto path_dy = next_py - py;
    auto path_dist = std::sqrt(path_dx * path_dx + path_dy * path_dy);
    current_cum += path_dist;
    cum_dist[i + 1] = current_cum;
  }
  return cum_dist;
}

bool PurePursuitControl::isReverseMotion() const { return veh_state.velocity < -1e-4; }

double PurePursuitControl::motionHeading(const VehicleState &vs, bool reverse) {
  return reverse ? adjustAngle(vs.yaw + ONE_PI)  // 车尾朝向
                 : vs.yaw;                       // 车头朝向
}

size_t PurePursuitControl::find_min_distance_point(const PathType &path, double x, double y) {
  size_t near_index = 0;
  auto pathLen = path.size();
  double min_near_distance = std::numeric_limits<double>::max();
  for (int i = 0; i < pathLen; i++) {
    auto [px, py] = path[i];
    auto path_dx = x - px;
    auto path_dy = y - py;
    auto path_dist = std::sqrt(path_dx * path_dx + path_dy * path_dy);
    if (path_dist < min_near_distance) {
      near_index = i;
      min_near_distance = path_dist;
    }
  }
  return near_index;
}

Point2D PurePursuitControl::findTargetPoint(const PathSumType &cum, double rear_x, double rear_y, double ld) {
  // 最近点 index
  const size_t i_near = find_min_distance_point(path, rear_x, rear_y);
  // 目标弧长
  const double s_target = cum[i_near] + ld;

  // 二分 / 线性扫描到 cum[j] ≤ s_target < cum[j+1]
  size_t j = i_near;
  while (j + 1 < cum.size() && cum[j + 1] < s_target) ++j;

  last_index = j;
  if (j + 1 == cum.size()) return path.back();  // 已到终点

  // 计算段内比例 lerp_ratio
  const double lerp_ratio = (s_target - cum[j]) / (cum[j + 1] - cum[j]);

  // 线性插值得到连续目标点
  const auto [x0, y0] = path[j];
  const auto [x1, y1] = path[j + 1];
  return {x0 + lerp_ratio * (x1 - x0), y0 + lerp_ratio * (y1 - y0)};
}

Point2D PurePursuitControl::findTargetPointReverse(const PathSumType &cum, double rear_x, double rear_y, double ld) {
  const size_t i_near = find_min_distance_point(path, rear_x, rear_y);

  const double s_target = cum[i_near] - ld;
  if (s_target <= 0.0)  // 已经到路径起点以前
    return path.front();

  size_t j = i_near;
  while (j > 0 && cum[j - 1] > s_target) --j;

  last_index = j;
  if (j == 0) return path.front();

  const double lerp_ratio = (cum[j] - s_target) / (cum[j] - cum[j - 1]);

  const auto [x0, y0] = path[j];
  const auto [x1, y1] = path[j - 1];
  return {x0 + lerp_ratio * (x1 - x0), y0 + lerp_ratio * (y1 - y0)};
}

/// @brief 严谨的算法应该考虑道路曲率,横向误差和速度,现在也是简化版本
/// @param lateral_error
/// @param curvature
/// @param velocity
/// @param min_ld
/// @return
double PurePursuitControl::calcLookaheadDistance(const double lateral_error,
                                                 const double curvature,
                                                 const double velocity,
                                                 const double min_ld) {
  auto velocity_ratio = isReverseMotion() ? param.ld_reverse_velocity_ratio : param.ld_velocity_ratio;

  const double vel_ld = std::abs(velocity_ratio * velocity);
  const double curvature_ld = -std::abs(param.ld_curvature_ratio * curvature);
  double lateral_error_ld = 0.0;
#if 0  // 优化车道中心保持的毛刺问题
  lateral_error_ld = std::abs(param.ld_lateral_error_ratio * lateral_error);
#else
  if (std::abs(lateral_error) >= param.long_ld_lateral_error_threshold) {
    lateral_error_ld = std::abs(param.ld_lateral_error_ratio * lateral_error);
  }
#endif

  if (isReverseMotion()) {
    lateral_error_ld = 0.0;
  }
  const double total_ld = std::min(vel_ld + min_ld + lateral_error_ld, param.max_lookahead_distance);

  return total_ld;
};

double PurePursuitControl::calcSteeringAngle(Point2D target, double x, double y, double ld) {
  const auto [tx, ty] = target;
  const bool rev = isReverseMotion();

  // a用运动方向做基准
  const double heading = motionHeading(veh_state, rev);
  const double alpha = adjustAngle(std::atan2(ty - y, tx - x) - heading);

  // 曲率按常规公式求，让符号由a决定
  double curvature = 2.0 * std::sin(alpha) / ld;

  // 倒车时：前轮在车辆几何中心前方，转向对曲率符号反向
  auto streeing_angle_ratio = param.extra_streeing_angle_ratio;
  if (rev) {
    curvature = -curvature;
    streeing_angle_ratio = param.extra_reverse_streeing_angle_ratio;
  }

  double steer = std::atan(curvature * veh_info.wheel_base);
  steer = adjustAngle(steer * streeing_angle_ratio);

  return std::clamp(steer, -veh_info.max_steer_angle, veh_info.max_steer_angle);
}

// double PurePursuitControl::applySteeringLag(double delta_filtered,double cmd_delta, double dt) {

//     constexpr double tau = 0.15;                 // 方向盘到轮胎的一阶惯性常数
//     delta_filtered = (tau * delta_filtered + dt * cmd_delta) / (tau + dt);
//     return delta_filtered;
// }

double PurePursuitControl::applySteeringLag(double delta_filtered, double cmd_delta, double dt) {
  // 假设每秒只能转0.7rad
  auto max_steer_angle_dt = 0.7 * dt;
  // 区分正负
  auto steer_difference = cmd_delta - delta_filtered;
  int sign = (steer_difference >= 0) ? 1 : -1;
  if (std::abs(steer_difference) > max_steer_angle_dt) steer_difference = sign * max_steer_angle_dt;

  auto limited_steer = steer_difference + delta_filtered;
  return std::clamp(limited_steer, -veh_info.max_steer_angle, veh_info.max_steer_angle);
}

double PurePursuitControl::calculateLateralError(double rear_x, double rear_y, double yaw) {
  double lateral_error = 0.0;
  const size_t i_near = find_min_distance_point(path, rear_x, rear_y);
  const auto [path_x, path_y] = path[i_near];

  const std::size_t idx1 = (i_near + 1 < path.size()) ? i_near + 1                   // 常规：用后一点
                                                      : (i_near == 0 ? i_near        // 特殊：轨迹长度只有 1
                                                                     : i_near - 1);  // 尾点：用前一点

  const auto [path_x2, path_y2] = path[idx1];
  const double ex = path_x2 - path_x;
  const double ey = path_y2 - path_y;

  const double rx = rear_x - path_x;
  const double ry = rear_y - path_y;

  const double cross_er = ex * ry - ey * rx;
  const double denom = std::hypot(ex, ey);
  if (denom > 1e-6)
    lateral_error = cross_er / denom;
  else
    lateral_error = 0.0;

  return lateral_error;
}

double PurePursuitControl::getCurrentLD() {
  auto min_ld = isReverseMotion() ? param.reverse_min_lookahead_distance : param.min_lookahead_distance;
  auto lateral_error = calculateLateralError(veh_state.x, veh_state.y, veh_state.yaw);
  auto total_ld = calcLookaheadDistance(lateral_error, 0, veh_state.velocity, min_ld);
  return total_ld;
}

double PurePursuitControl::getCurrentLD(double lateral_error) {
  auto min_ld = isReverseMotion() ? param.reverse_min_lookahead_distance : param.min_lookahead_distance;
  auto total_ld = calcLookaheadDistance(lateral_error, 0, veh_state.velocity, min_ld);

  return total_ld;
}
ControlMsg PurePursuitControl::run() {
  ControlMsg msg;
  auto cum_dist = getRoadCumDist();
  auto total_ld = getCurrentLD();
  //// 如果倒车入参点顺序相同则使用此代码
  // auto target = isReverseMotion() ? findTargetPointReverse(cum_dist, veh_state.rear_x, veh_state.rear_y, total_ld)
  //                                 : findTargetPoint(cum_dist, veh_state.rear_x, veh_state.rear_y, total_ld);
  // 根据需求,倒车入参点集顺序会与前进时不同
  auto target = findTargetPoint(cum_dist, veh_state.rear_x, veh_state.rear_y, total_ld);
  msg.steering_tire_angle = calcSteeringAngle(target, veh_state.rear_x, veh_state.rear_y, total_ld);
  // auto real_curvature = std::tan(msg.steering_tire_angle) / veh_info.wheel_base;
  // msg.lateral_acceleration = veh_state.velocity * veh_state.velocity * real_curvature;
  return msg;
}

ControlMsg PurePursuitControl::fastRun(Point2D target, double total_ld) {
  ControlMsg msg;
  msg.steering_tire_angle = calcSteeringAngle(target, veh_state.rear_x, veh_state.rear_y, total_ld);
  // auto real_curvature = std::tan(msg.steering_tire_angle) / veh_info.wheel_base;
  // msg.lateral_acceleration = veh_state.velocity * veh_state.velocity * real_curvature;
  return msg;
}

void PurePursuitControl::updateVehicleState(const ControlMsg &msg, double dt) {
  auto cos_yaw = std::cos(veh_state.yaw);
  auto sin_yaw = std::sin(veh_state.yaw);
  // veh_state.steering_angle=applySteeringLag(veh_state.steering_angle,msg.steering_tire_angle,dt);
  veh_state.steering_angle = msg.steering_tire_angle;
  /**
   * @brief
   注意这里的顺序,根据运动学模型假设,车辆的几何中心会沿着当前航向角前进
   所以先完成几何中心更新
   */
  veh_state.x = veh_state.x + veh_state.velocity * cos_yaw * dt;
  veh_state.y = veh_state.y + veh_state.velocity * sin_yaw * dt;

  veh_state.yaw =
      adjustAngle(veh_state.yaw + veh_state.velocity / veh_info.wheel_base * std::tan(veh_state.steering_angle) * dt);

  auto new_cos_yaw = std::cos(veh_state.yaw);
  auto new_sin_yaw = std::sin(veh_state.yaw);
  // 根据新的航向角和新的几何中心反推后轴中心
  // 也只有这样做才能模拟出前轮转弯的效果
  auto half_wb = veh_info.wheel_base / 2;
  veh_state.rear_x = veh_state.x - (half_wb * new_cos_yaw);
  veh_state.rear_y = veh_state.y - (half_wb * new_sin_yaw);
}

}  // namespace pursuit
