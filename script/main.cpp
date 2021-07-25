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

int main() {
  double new_mean, new_variance;
  std::tie(new_mean, new_variance) = measurement_update(20.0, 5.0, 30.0, 5.0);
  printf("new_mean: %f, new_variance: %f", new_mean, new_variance);
  return 0;
}