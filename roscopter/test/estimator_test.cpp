#include "gtest/gtest.h"
#include <iostream>
#include "ekf/estimator_continuous_discrete.hpp"
#include "ekf/geomag.h"

class Estimator_tester : public roscopter::EstimatorContinuousDiscrete{
public:
  Estimator_tester()
{
  roscopter::EstimatorContinuousDiscrete();
};

  Input input;
  Output output;

  auto test_estimate(){
    estimate(input, output) ;
    return output;
  };
  
};

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
  return RUN_ALL_TESTS();
}
