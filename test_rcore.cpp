#include "rCore/rxInterpolator.h"
#include <iostream>

int main() {
  rCore::Interpolator::TYPE type = rCore::Interpolator::TYPE::LINEAR;
  double t0 = 0.0;
  double tf = 10.0;
  rMath::VectorXd vi;
  vi.resize(2);
  vi << 0.0, 0.0;
  rMath::VectorXd vf;
  vf.resize(2);
  vf << 10.0, 100.0;
  rlab::rxcontrolsdk::rxInterpolator linIntp(type, t0, tf, vi, vf);
  double p, v, a;
  linIntp.interpolate(5.0, p, v, a);
  std::cout << p << std::endl;
  return 0;
}