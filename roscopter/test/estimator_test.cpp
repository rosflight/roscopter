#include "gtest/gtest.h"
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include "ekf/estimator_continuous_discrete.hpp"
#include "ekf/geomag.h"

class Estimator_tester : public roscopter::EstimatorContinuousDiscrete{
public:
  Estimator_tester()
{
  // roscopter::EstimatorContinuousDiscrete();
};

  Input input;
  Output output;

  auto test_estimate(){
    estimate(input, output) ;
    return output;
  };
  
};

TEST(estimator_test, mem_test)
{
  auto estimator = Estimator_tester();

  estimator.input.gps_n = 0;
  estimator.input.gps_e = 0;
  estimator.input.gps_h = 0;
  estimator.input.mag_x = 0.489764;
  estimator.input.mag_y = 0.128677;
  estimator.input.mag_z = 0.922567;
  estimator.input.gps_Vg = 0;
  estimator.input.gps_vn = 0;
  estimator.input.gps_ve = 0;
  estimator.input.gps_vd = 0;
  estimator.input.gyro_x = -8.35013e-15;
  estimator.input.gyro_y = -1.22497e-15;
  estimator.input.gyro_z = 8.63518e-18;
  estimator.input.accel_x = -5.26576e-16;
  estimator.input.accel_y = -2.38951e-16;
  estimator.input.accel_z = -9.80665;
  estimator.input.gps_alt = 0;
  estimator.input.gps_lat = 0;
  estimator.input.gps_lon = 0;
  estimator.input.gps_day = 0;
  estimator.input.gps_month = 0;
  estimator.input.gps_year = 0;
  estimator.input.diff_pres = 0;
  estimator.input.armed_init = 0;
  estimator.input.static_pres = -64.827;
  estimator.input.gps_course = 0;
  estimator.input.gps_new = 0;
  estimator.input.status_armed = 0;

  estimator.test_estimate();
  
  // std::cout << "output.pn: " << estimator.output.pn << "\n";
  // std::cout << "output.pe: " << estimator.output.pe << "\n";
  // std::cout << "output.pd: " << estimator.output.pd << "\n";
  // std::cout << "output.vn: " << estimator.output.vn << "\n";
  // std::cout << "output.ve: " << estimator.output.ve << "\n";
  // std::cout << "output.vd: " << estimator.output.vd << "\n";
  // std::cout << "output.p: " << estimator.output.p << "\n";
  // std::cout << "output.q: " << estimator.output.q << "\n";
  // std::cout << "output.r: " << estimator.output.r << "\n";
  // std::cout << "output.phi: " << estimator.output.phi << "\n";
  // std::cout << "output.theta: " << estimator.output.theta << "\n";
  // std::cout << "output.psi: " << estimator.output.psi << "\n";
  // std::cout << "output.bx: " << estimator.output.bx << "\n";
  // std::cout << "output.by: " << estimator.output.by << "\n";
  // std::cout << "output.bz: " << estimator.output.bz << "\n";

  ASSERT_EQ(0, 0);
}

// [INFO] [1724098960.625067166] [estimator_ros]: output.pn: -1.05258e-09
// [INFO] [1724098960.625087311] [estimator_ros]: output.pe: -2.76547e-10
// [INFO] [1724098960.625109495] [estimator_ros]: output.pd: 0.109633
// [INFO] [1724098960.625131694] [estimator_ros]: output.vn: -8.21015e-07
// [INFO] [1724098960.625151328] [estimator_ros]: output.ve: -2.15707e-07
// [INFO] [1724098960.625170377] [estimator_ros]: output.vd: 0.112132
// [INFO] [1724098960.625196734] [estimator_ros]: output.p: -1.00475e-15
// [INFO] [1724098960.625217830] [estimator_ros]: output.q: -1.47397e-16
// [INFO] [1724098960.625238461] [estimator_ros]: output.r: 1.03905e-18
// [INFO] [1724098960.625256586] [estimator_ros]: output.phi: -8.57844e-06
// [INFO] [1724098960.625291518] [estimator_ros]: output.theta: 3.26509e-05
// [INFO] [1724098960.625317711] [estimator_ros]: output.psi: 2.07164e-20
// [INFO] [1724098960.625338439] [estimator_ros]: output.bx: 0
// [INFO] [1724098960.625365140] [estimator_ros]: output.by: 0
// [INFO] [1724098960.625386305] [estimator_ros]: output.bz: 0

TEST(estimator_test, mag_test)
{

  geomag_init();

  float gps_altitude = 1489.0;
  float gps_lat = 40.0;
  float gps_lon = 111.0;
  float decimal_year = 2024.563;

  double declination;
  double inclination;
  double total_intensity;
  double grid_variation;

  int mag_success = geomag_calc(gps_altitude/1000.0,
                                gps_lat,
                                gps_lon,
                                decimal_year,
                                &declination,
                                &inclination,
                                &total_intensity,
                                &grid_variation);

  std::cout << "incl: " << inclination << std::endl;
  std::cout << "declination: " << declination << std::endl;
  std::cout << "total_intensity: " << total_intensity << std::endl;
  std::cout << "grid_variation: " << grid_variation << std::endl;

  ASSERT_EQ(0, mag_success);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  int return_code = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return return_code;
}
