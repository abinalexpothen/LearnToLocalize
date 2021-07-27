#include <math.h>

#include <iostream>
#include <tuple>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/LU"

using namespace Eigen;

float measurements[3] = {1, 2, 3};

std::tuple<MatrixXf, MatrixXf> kalman_filter(MatrixXf x, MatrixXf P, MatrixXf u,
                                             MatrixXf F, MatrixXf H, MatrixXf R,
                                             MatrixXf I) {
  for (int i = 0; i < sizeof(measurements) / sizeof(measurements[0]); i++) {
    MatrixXf z(1, 1);
    z << measurements[i];

    // measurement update
    MatrixXf y(1, 1);
    y << z - H * x;

    MatrixXf S(1, 1);
    S << H * P * H.transpose() + R;

    // Kalman gain
    MatrixXf K(2, 1);
    K << P * H.transpose() * S.inverse();

    x << x + K * y;
    P << (I - K * H) * P;

    // state prediction
    x << F * x + u;
    P << F * P * F.transpose();
  }

  return std::make_tuple(x, P);
}

int main() {
  MatrixXf x(2, 1);  // initial position and velocity
  x << 0, 0;
  MatrixXf P(2, 2);  // initial uncertainty
  P << 100, 0, 0, 100;
  MatrixXf u(2, 1);  // external motion
  u << 0, 0;
  MatrixXf F(2, 2);  // next state function
  F << 1, 1, 0, 1;
  MatrixXf H(1, 2);  // measurement function
  H << 1, 0;
  MatrixXf R(1, 1);  // measurement uncertainty
  R << 1;
  MatrixXf I(2, 2);  // identity matrix
  I << 1, 0, 0, 1;

  std::tie(x, P) = kalman_filter(x, P, u, F, H, R, I);

  std::cout << "x = " << x << std::endl;
  std::cout << "P = " << P << std::endl;

  return 0;
}