#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <list>
#include <type_traits>

#include "cubic_spline.h"
#include "pure_pursuit.h"
// #include "fmt/format.h"
// template <>
// struct fmt::formatter<pursuit::State> {
//   constexpr auto parse(fmt::format_parse_context &ctx) { return ctx.begin(); }

//   template <typename FormatContext>
//   auto format(const pursuit::State &s, FormatContext &ctx) {
//     return fmt::format_to(ctx.out(),
//                           "State(x={:.6f}, y={:.6f}, yaw={:.6f}, v={:.6f}, rear_x={:.6f}, rear_y={:.6f})",
//                           s.x,
//                           s.y,
//                           s.yaw,
//                           s.velocity,
//                           s.rear_x,
//                           s.rear_y);
//   }
// };

#include <iomanip>
#include <iostream>

inline std::ostream &operator<<(std::ostream &os, const pursuit::VehicleState &s) {
  os << "State(x=" << std::fixed << std::setprecision(6) << s.x << ", y=" << s.y << ", yaw=" << s.yaw
     << ", v=" << s.velocity << ", rear_x=" << s.rear_x << ", rear_y=" << s.rear_y
     << ", steering_angle=" << s.steering_angle << ")";
  return os;
}

std::pair<std::vector<double>, std::vector<double>> loadXYfromCSV(const std::string &fname) {
  std::ifstream fin(fname);
  if (!fin) throw std::runtime_error("cannot open " + fname);

  std::string line;
  std::getline(fin, line);  // ① 读掉表头

  std::vector<double> xs, ys;
  xs.reserve(1024);
  ys.reserve(1024);

  while (std::getline(fin, line)) {
    if (line.empty()) continue;  // 忽略空行
    std::stringstream ss(line);
    std::string cell;

    // ② 解析第一列 X
    if (!std::getline(ss, cell, ',')) break;
    xs.push_back(std::stod(cell));

    // ③ 解析第二列 Y
    if (!std::getline(ss, cell, ',')) break;
    ys.push_back(std::stod(cell));
  }

  if (xs.size() != ys.size()) throw std::runtime_error("loadXYfromCSV: x/y size mismatch");
  return {xs, ys};
}

void saveXYtoCSV(const std::string &fname,
                 const std::vector<double> &xs,
                 const std::vector<double> &ys,
                 const std::string &header = "x,y") {
  if (xs.size() != ys.size()) throw std::runtime_error("saveXYtoCSV: x/y size mismatch");

  std::ofstream fout(fname, std::ios::trunc);
  if (!fout) throw std::runtime_error("cannot open " + fname);

  // ① 写表头
  fout << header << '\n';

  // ② 写数据——用足够精度防止往返误差
  fout << std::setprecision(15) << std::scientific;
  for (std::size_t i = 0; i < xs.size(); ++i) fout << xs[i] << ',' << ys[i] << '\n';

  // ③ 检查写入是否成功（可选）
  if (!fout.good()) throw std::runtime_error("failed to write " + fname);
}

int main() {
  double wb = 2.9;
  double k = 0.1;
  double ld = 4.0;
  double dt = 0.02;
  //   10
  double velocity = 20.0 / 3.6;
  pursuit::PurePursuitControl ppc;

  auto [csx, csy] = loadXYfromCSV("xy.csv");

  struct V3D {
    double _x;
    double _y;
    double _z;

    double x() const { return _x; };
    double y() const { return _y; };
    double z() const { return _z; };
  };
  bool smooth = true;
  bool fast = true;
  bool reverse = false;
  // if (reverse) {
  //   std::reverse(csx.begin(), csx.end());
  //   std::reverse(csy.begin(), csy.end());
  // }
  double ds = 0.01;
  const unsigned long len = csx.size();
  std::vector<double> s(len);
  std::unique_ptr<trajectory_smooth::CubicSpline2D> cc2;
  cc2 = std::make_unique<trajectory_smooth::CubicSpline2D>(csx.data(), csy.data(), len);
  auto c2 = trajectory_smooth::CubicSpline2D(csx.data(), csy.data(), len);
  // std::cout<<"N:"<<N<<'\n';
  auto N = c2.getSamplePointsCount(ds);
  std::vector<double> cx(N, 0.0);
  std::vector<double> cy(N, 0.0);
  if (smooth) {
    c2.cubicSpline(cx.data(), cy.data(), ds);
  } else {
    cx = csx;
    cy = csy;
  }
  auto path_len = cy.size();

  // 模拟vector3d
  std::list<V3D> _pathPlanningPointList;
  for (size_t i = 0; i < path_len; i++) {
    _pathPlanningPointList.push_back({cx[i], cy[i], 0.0});
  }

  std::cout << path_len << '\n';
  // std::cout << "is pod:" << std::is_pod_v<pursuit::PurePursuitControl> << "\n";
  // std::cout << "is trivial:" << std::is_trivial_v<pursuit::PurePursuitControl> << "\n";
  // std::cout << "is trivial_default:" << std::is_trivially_default_constructible_v<pursuit::PurePursuitControl> <<
  // "\n"; std::cout << "is standard:" << std::is_standard_layout_v<pursuit::PurePursuitControl> << "\n";

  pursuit::VehicleState state;
  pursuit::VehicleParam veh_info;
  pursuit::PurePursuitParams param;
  // 初始化 车辆状态
  if (reverse) {
    // state.x = 2363.4668295918327;
    // state.y = -1044.2733335638607;
    state.x = cx[0];
    state.y = cy[0];
    state.yaw = -pursuit::ONE_PI;
    state.velocity = -velocity;
  } else {
    state.x = cx[0];
    state.y = cy[0];
    // state.yaw =0;
    state.yaw = -pursuit::ONE_PI/2;
    // state.yaw = -pursuit::ONE_PI;
    state.velocity = velocity;
  }
  state.rear_x = state.x - ((wb / 2) * std::cos(state.yaw));
  state.rear_y = state.y - ((wb / 2) * std::sin(state.yaw));
  state.steering_angle = 0;

  // 顺序不分先后,设置状态,车辆参数,算法参数,路径
  // ppc.setVehcleState(state);
  // ppc.setVehcleParam(veh_info);
  // ppc.setPurePursuitParam(param);
  // ppc.setPath(_pathPlanningPointList);

  std::ofstream log_file("log.txt");

  ppc.setVehcleState(state);
  ppc.setVehcleParam(veh_info);
  ppc.setPurePursuitParam(param);

  int count = 0;
  auto start = std::chrono::high_resolution_clock::now();
  while (true) {
    pursuit::ControlMsg msg;

    auto &p_state = ppc.getVehcleState();

    if (fast) {
      // 如果希望每帧插值则在循环中调用
      // auto c2 = trajectory_smooth::CubicSpline2D(csx.data(), csy.data(), len);
      cc2 = std::make_unique<trajectory_smooth::CubicSpline2D>(csx.data(), csy.data(), len);
      // std::string info;
      // info.resize(8000);
      // cc2->debug(info.data(), info.size());
      // std::cout<<info<<'\n';

      // auto close = cc2->continuousClosest(x1, y1);
      auto smax=cc2->getSmax();
      auto close = cc2->continuousClosest(p_state.x,p_state.y);
      double ld = ppc.getCurrentLD(std::abs(close.lat_err));
      // exit(0);
      if (std::abs(smax - close.s) < 1e-4) break;
      pursuit::Point2D target{0, 0};
      cc2->findTargetPoint(close, ld, target.first, target.second);

      // std::cout << "target.first=" << target.first << "  target.second=" << target.second << '\n';
      // std::cout <<"smax1="<< smax <<"  close.lat_err=" << close.lat_err << "  close.s=" << close.s << " ld=" << ld << '\n';
      msg = ppc.fastRun(target,ld);

      
    } else {
      // 如果希望每帧插值则在循环中调用
      ppc.setPath(_pathPlanningPointList);
      msg = ppc.run();
    }

    // 使用控制信号和dt更新状态
    ppc.updateVehicleState(msg, dt);

    // 更新完毕后获取状态引用
    // p_state.velocity=velocity- p_state.steering_angle/veh_info.max_steer_angle*2;
    count++;
    // 如果状态改变,通常是速度
    // 直接调用setVelocity去设置速度
    // 路径改变调用ppc.setPath(_pathPlanningPointList); 去设置路径

    // std::cout << p_state << "\n\n";
    log_file << p_state << '\n';
    if (count >= 500) break;
    // if (ppc.last_index >= path_len - 1 || ppc.last_index <= 1) break;
  }
  auto end = std::chrono::high_resolution_clock::now();

  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

  auto us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();

  double fms = std::chrono::duration<double, std::milli>(end - start).count();

  std::cout << "计算轮次: " << count << " 次\n"
            << "millis: " << ms << " ms\n"
            << "micros: " << us << " μs\n"
            << "nanos : " << ns << " ns\n"
            << "float ms: " << fms << " ms\n";
}
