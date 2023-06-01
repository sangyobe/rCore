#include "rCore/rxInterpolator.h"
#include <chrono>
#include <iostream>

void TestInterpolator(rCore::Interpolator::TYPE type, int sps, int loop) {
  double t0 = 0.0;
  double tf = 10.0;
  rMath::Vector3d vi;
  vi << 0.0, 0.0, 0.0;
  rMath::Vector3d vf;
  vf << 5.0, 0.0, 0.0;
  for (int j = 0; j < loop; j++) {
    std::chrono::system_clock::time_point start =
        std::chrono::system_clock::now();
    for (int i = 0; i < sps; i++) {
      rlab::rxcontrolsdk::rxInterpolator linIntp(type, t0, tf, vi, vf);
      double p, v, a;
      double t = t0 + (tf - t0) / sps * i;
      linIntp.interpolate(t, p, v, a);
      // std::cout << "(" << t << ", " << p << ")" << std::endl;
    }
    std::chrono::system_clock::time_point end =
        std::chrono::system_clock::now();
    std::chrono::duration<double> sec_elapsed = end - start;
    std::cout << "elapsed time(" << j << " th)=" << sec_elapsed.count()
              << " (sec)" << std::endl;
  }
}

int main() {
  TestInterpolator(rCore::Interpolator::TYPE::CUBIC, 1000000, 10);
  return 0;
}