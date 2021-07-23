#include <math.h>

#include <iostream>
#include <tuple>

double calc_gaussian_probability(double x, double mu, double sigma2) {
  return exp(-pow((x - mu), 2) / (2 * sigma2)) / sqrt(2 * M_PI * sigma2);
}

/*
 mu - mean of prior belief
 sigma2 - variance of prior belief

 v - mean of measurement
 r2 - variance of measurement

 tau - mean of posterior
 s2 - variance of measurement

*/
std::tuple<double, double> measurement_update(double mu, double sigma2,
                                              double v, double r2) {
  double tau = (r2 * mu + sigma2 * v) / (r2 + sigma2);
  double s2 = 1.0 / (1.0 / r2 + 1.0 / sigma2);
  return std::make_tuple(tau, s2);
}

int main() {
  double tau, s2;
  std::tie(tau, s2) = measurement_update(30.0, 6.0, 5.0, 1.0);
  printf("tau: %f, s^2: %f", tau, s2);
  return 0;
}