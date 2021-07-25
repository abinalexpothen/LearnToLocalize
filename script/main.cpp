#include <math.h>

#include <iostream>
#include <tuple>

double calc_gaussian_probability(double x, double mean, double variance) {
  return exp(-pow((x - mean), 2) / (2 * variance)) / sqrt(2 * M_PI * variance);
}

std::tuple<double, double> measurement_update(double mean1, double variance1,
                                              double mean2, double variance2) {
  double new_mean =
      (variance2 * mean1 + mean2 * variance1) / (variance1 + variance2);
  double new_variance = 1.0 / (1.0 / variance1 + 1.0 / variance2);
  return std::make_tuple(new_mean, new_variance);
}

std::tuple<double, double> state_prediction(double mean1, double variance1,
                                            double mean2, double variance2) {
  double new_mean = mean1 + mean2;
  double new_variance = variance1 + variance2;
  return std::make_tuple(new_mean, new_variance);
}

int main() {
  double measurements[5] = {5, 6, 7, 9, 10};
  double measurement_sig = 4;

  double motion[5] = {1, 1, 2, 1, 1};
  double motion_sig = 2;

  double mu = -100000;
  double var = 1000000;

  // 1D Kalman filter
  for (std::size_t i = 0; i < sizeof(measurements) / sizeof(measurements[0]);
       i++) {
    std::tie(mu, var) =
        measurement_update(mu, var, measurements[i], measurement_sig);
    printf("Measurement update mean: %f, variance: %f\n", mu, var);
    std::tie(mu, var) = state_prediction(mu, var, motion[i], motion_sig);
    printf("State prediction mean: %f, variance: %f\n", mu, var);
  }

  return 0;
}