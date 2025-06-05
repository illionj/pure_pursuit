#include "pure_pursuit.h"

#include <chrono>
#include <fstream>
#include <list>
#include <type_traits>
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

int main() {
  double wb = 2.9;
  double k = 0.1;
  double ld = 4.0;
  double dt = 0.02;
  //   10
  double velocity = 30.0 / 3.6;
  pursuit::PurePursuitControl ppc;

  /**    cx = np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]*/
  std::vector<double> cx = {
      205.2597382338551, 205.25973872404742, 205.25973921423977, 205.25973970443212,
      205.25974019462447, 205.25974068481682, 205.25974117500917, 205.25974166520152,
      205.25974215539384, 205.2597426455862, 205.25974313577854, 205.25974362597088,
      205.25974411616323, 205.25974460635558, 205.25974509654793, 205.25974558674028,
      205.2597460769326, 205.25974656712495, 205.2597470573173, 205.25974754750965,
      205.259748037702, 205.25974852789435, 205.2597490180867, 205.25974950827901,
      205.25974999847136, 205.1377202276385, 204.86828537329237, 204.5246461654523,
      204.1319813483608, 203.69779882732092, 203.22588423343402, 202.70113386090685,
      202.1132682617176, 201.4356033145372, 200.62157584836785, 199.82633707164717,
      199.32633707164717, 198.82633707164717, 198.32633707164717, 197.82633707164717,
      197.32633707164717, 196.82633707164717, 196.32633707164717, 195.82633707164717,
      195.32633707164717, 194.82633707164717, 194.32633707164717, 193.82633707164717,
      193.32633707164717, 192.82633707164717, 192.32633707164717, 191.82633707164717,
      191.32633707164717, 190.82633707164717, 190.32633707164717, 189.82633707164717,
      189.32633707164717, 188.82633707164717, 188.32633707164717, 187.82633707164717,
      187.32633707164717, 186.82633707164717
  };

  std::vector<double> cy = {
      -15.501560064917355, -15.001560064917594, -14.501560064917834, -14.001560064918076,
      -13.501560064918316, -13.001560064918555, -12.501560064918797, -12.001560064919037,
      -11.501560064919277, -11.001560064919518, -10.501560064919756, -10.001560064919998,
      -9.501560064920238, -9.001560064920477, -8.501560064920719, -8.001560064920959,
      -7.501560064921199, -7.0015600649214385, -6.501560064921679, -6.001560064921919,
      -5.50156006492216, -5.0015600649224, -4.50156006492264, -4.001560064922881,
      -3.5015600649231207, -2.5680843378482066, -1.8101016700594803, -1.168914182569483,
      -0.6059494183279761, -0.0998004568689208, 0.35683384277383645, 0.7765905426191794,
      1.153399178367869, 1.4751874347296938, 1.704749890949171, 1.75975,
      1.75975, 1.75975, 1.75975, 1.75975,
      1.75975, 1.75975, 1.75975, 1.75975,
      1.75975, 1.75975, 1.75975, 1.75975,
      1.75975, 1.75975, 1.75975, 1.75975,
      1.75975, 1.75975, 1.75975, 1.75975,
      1.75975, 1.75975, 1.75975, 1.75975,
      1.75975, 1.75975
  };
  // std::vector<double> cx = {0,  0.5,  1,  1.5,  2,  2.5,  3,  3.5,  4,  4.5,  5,  5.5,  6,  6.5,  7,  7.5,  8,  8.5,
  //                           9,  9.5,  10, 10.5, 11, 11.5, 12, 12.5, 13, 13.5, 14, 14.5, 15, 15.5, 16, 16.5, 17, 17.5,
  //                           18, 18.5, 19, 19.5, 20, 20.5, 21, 21.5, 22, 22.5, 23, 23.5, 24, 24.5, 25, 25.5, 26, 26.5,
  //                           27, 27.5, 28, 28.5, 29, 29.5, 30, 30.5, 31, 31.5, 32, 32.5, 33, 33.5, 34, 34.5, 35, 35.5,
  //                           36, 36.5, 37, 37.5, 38, 38.5, 39, 39.5, 40, 40.5, 41, 41.5, 42, 42.5, 43, 43.5, 44, 44.5,
  //                           45, 45.5, 46, 46.5, 47, 47.5, 48, 48.5, 49, 49.5};

  // std::vector<double> cy = {0,
  //                           0.024958354161707,
  //                           0.0993346653975306,
  //                           0.221640154996005,
  //                           0.389418342308651,
  //                           0.599281923255254,
  //                           0.846963710092553,
  //                           1.12738095266596,
  //                           1.43471218179905,
  //                           1.76248554666184,
  //                           2.10367746201974,
  //                           2.45082024016895,
  //                           2.79611725790168,
  //                           3.13156410260588,
  //                           3.44907405495961,
  //                           3.7406061997652,
  //                           3.99829441216602,
  //                           4.21457544442299,
  //                           4.38231433895188,
  //                           4.49492541651522,
  //                           4.54648713412841,
  //                           4.53184917490659,
  //                           4.44673022100775,
  //                           4.28780497001614,
  //                           4.05277908330691,
  //                           3.74045090064973,
  //                           3.35075891683952,
  //                           2.88481419157835,
  //                           2.34491705109134,
  //                           1.73455763680137,
  //                           1.058400060449,
  //                           0.322250133858001,
  //                           -0.466993147420641,
  //                           -1.3014019766818,
  //                           -2.17209936722807,
  //                           -3.06935324228417,
  //                           -3.98268398965367,
  //                           -4.90098430340356,
  //                           -5.81264996395583,
  //                           -6.70572005204374,
  //                           -7.56802495307928,
  //                           -8.38734038841021,
  //                           -9.15154561034268,
  //                           -9.84878382005664,
  //                           -10.4676228127847,
  //                           -10.9972138237323,
  //                           -11.4274465417848,
  //                           -11.7490982763782,
  //                           -11.9539753060301,
  //                           -12.0350445046481,
  //                           -11.9865534332892,
  //                           -11.8041371996786,
  //                           -11.484910524362,
  //                           -11.0275436094667,
  //                           -10.4323205820058,
  //                           -9.70117947659289,
  //                           -8.8377329302125,
  //                           -7.84726898201634,
  //                           -6.73673160149948,
  //                           -5.51468080624598,
  //                           -4.19123247298389,
  //                           -2.77797819014946,
  //                           -1.28788574367119,
  //                           0.264818932628508,
  //                           1.8647872776079,
  //                           3.495699806427,
  //                           5.14043249797073,
  //                           6.78123617032802,
  //                           8.39992696935634,
  //                           9.97808593569645,
  //                           11.4972654775788,
  //                           12.9392004622343,
  //                           14.2860215492848,
  //                           15.5204683264713,
  //                           16.6260997725151,
  //                           17.5874995645264,
  //                           18.3904737685982,
  //                           19.0222385021323,
  //                           19.4715952348048,
  //                           19.7290915013355,
  //                           19.7871649324676,
  //                           19.640268669613,
  //                           19.2849764119354,
  //                           18.7200655504431,
  //                           17.9465770698539,
  //                           16.9678511432492,
  //                           15.7895376042934,
  //                           14.4195807542875,
  //                           12.8681782436188,
  //                           11.1477140561879,
  //                           9.27266591793952,
  //                           7.25948774344776,
  //                           5.1264680243057,
  //                           2.89356534653918,
  //                           0.582222498153907,
  //                           -1.78483911096797,
  //                           -4.18384274935151,
  //                           -6.59019519046535,
  //                           -8.97873866667225,
  //                           -11.3240133709392};
  struct V3D {
    double _x;
    double _y;
    double _z;

    double x() const { return _x; };
    double y() const { return _y; };
    double z() const { return _z; };
  };

  auto path_len = cy.size();
  // 模拟vector3d
  std::list<V3D> _pathPlanningPointList;
  for (size_t i = 0; i < path_len; i++) {
    _pathPlanningPointList.push_back({cx[i], cy[i], 0.0});
  }

  std::cout << path_len << '\n';
  std::cout << "is pod:" << std::is_pod_v<pursuit::PurePursuitControl> << "\n";
  std::cout << "is trivial:" << std::is_trivial_v<pursuit::PurePursuitControl> << "\n";
  std::cout << "is trivial_default:"
            << std::is_trivially_default_constructible_v<pursuit::PurePursuitControl> << "\n";
  std::cout << "is standard:" << std::is_standard_layout_v<pursuit::PurePursuitControl> << "\n";

  pursuit::VehicleState state;
  pursuit::VehicleParam veh_info;
  pursuit::PurePursuitParams param;
  // 初始化 车辆状态
  bool reverse = false;
  if (reverse) {
    std::reverse(_pathPlanningPointList.begin(),_pathPlanningPointList.end());
    state.x = cx[path_len - 2];
    state.y = cy[path_len - 2];
    state.yaw =pursuit::ONE_PI;
    state.velocity = -velocity;
  } else {
    state.x = cx[0];
    state.y = cy[0];
    state.yaw = pursuit::ONE_PI/2;
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

  auto start = std::chrono::high_resolution_clock::now();
  ppc.setVehcleState(state);
  ppc.setVehcleParam(veh_info);
  ppc.setPurePursuitParam(param);
  int count = 0;
  while (true) {
    ppc.setPath(_pathPlanningPointList);
    // std::cout << ppc.last_index << '\n';
    // 获取控制信ss
    auto msg = ppc.run();
    // 使用控制信号和dt更新状态
    ppc.updateVehicleState(msg, dt);
    // 更新完毕后获取状态引用
    auto &p_state = ppc.getVehcleState();
    // p_state.velocity=velocity- p_state.steering_angle/veh_info.max_steer_angle*2;
    count++;
    // 如果状态改变,通常是速度
    // 直接调用setVelocity去设置速度
    // 路径改变调用ppc.setPath(_pathPlanningPointList); 去设置路径

    std::cout << p_state << '\n';
    log_file << p_state << '\n';
    // if(count>=30)break;
    if (ppc.last_index >= path_len - 1 || ppc.last_index <= 1) break;
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
