#include <math.h>

#include <iostream>

double calc_gaussian_probability(double x, double mu, double sigma2) {
  return exp(-pow((x - mu), 2) / (2 * sigma2)) / sqrt(2 * M_PI * sigma2);
}

int main() {
  std::cout << "probability is: " << calc_gaussian_probability(10.0, 9.8, 6.0);
  return 0;
}