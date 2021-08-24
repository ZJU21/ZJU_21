/*
 *  (https://github.com/cgxeiji/CGx-InverseK).
 * Inverse Kinematic Library for Arduino for a three link-arm system with a
 * rotating base.
 */
#include <Arduino.h>
#ifndef IKARM_H
#define IKARM_H

#define DOUBLE_PI 6.28318530718

//机械臂的“零构形”（初始构形），水平
class Link {
 public:
  Link();

  void init(float length, float angle_low_limit, float angle_high_limit);

  bool inRange(float angle);
  float getLength();
  float getAngle();
  void setAngle(float angle);

 private:
  float _length;
  float _angleLow;
  float _angleHigh;
  float _angle;
};

class IKArm {
 public:
  IKArm();

  void attach(Link shoulder, Link upperarm, Link forearm, Link hand);

  bool solve(float x, float y, float z, float phi, float& base, float& shoulder,
             float& elbow, float& wrist);

 private:
  Link _L0;  // Link 0: Shoulder
  Link _L1;  // Link 1: Upperarm
  Link _L2;  // Link 2: Forearm
  Link _L3;  // Link 3: Hand

  float _currentPhi;

  // Get polar coords from cartesian ones 卡坐标转极坐标
  void cart2polar(float a, float b, float& r, float& theta);

  bool _cosrule(float opposite, float adjacent1, float adjacent2, float& angle);
};
#endif  // IKARM_H