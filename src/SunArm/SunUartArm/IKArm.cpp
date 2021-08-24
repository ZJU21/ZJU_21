/*
 * This file is part of the CGx-InverseK distribution
 * (https://github.com/cgxeiji/CGx-InverseK).
 * 类的组合
 */

#include "IKArm.h"
#include "math.h"

//杆件构造函数
Link::Link() {
  _length = 0.0;     //杆长
  _angleLow = 0.0;   //关节最小角度
  _angleHigh = 0.0;  //关节最大角度
  _angle = 0.0;      //关节角度
}
//初始化
void Link::init(float length, float angle_low_limit, float angle_high_limit) {
  _length = length;
  _angleLow = angle_low_limit;
  _angleHigh = angle_high_limit;
}
//角度是否在范围内
bool Link::inRange(float angle) {
  return (angle >= _angleLow) && (angle <= _angleHigh);
}
//获取杆长
float Link::getLength() { return _length; }
//获取角度
float Link::getAngle() { return _angle; }
//设置角度
void Link::setAngle(float angle) { _angle = angle; }

// Initialize Inverse object 逆运动学构造函数
IKArm::IKArm() {
  _currentPhi = -DOUBLE_PI;  //初始角度
}

// Load the links of the mechanism to be solved 加载杆长 单位mm
void IKArm::attach(Link shoulder, Link upperarm, Link forearm, Link hand) {
  _L0 = shoulder;
  _L1 = upperarm;
  _L2 = forearm;
  _L3 = hand;
}

// Cosine rule function 余弦定理
// bool _Inverse::_cosrule(float opposite, float adjacent1, float adjacent2,
// float& angle) bool _Inverse::_cosrule( 对边, 邻边1, 邻边2, 角度)
// a^2=b^2+c^2-2bccA
bool IKArm::_cosrule(float opposite, float adjacent1, float adjacent2,
                     float& angle) {
  float delta = 2 * adjacent1 * adjacent2;

  if (delta == 0) return false;

  float cos =
      (adjacent1 * adjacent1 + adjacent2 * adjacent2 - opposite * opposite) /
      delta;

  if ((cos > 1) || (cos < -1)) return false;

  angle = acos(cos);  // acos 返回弧度值 0~pi

  return true;
}

// Get polar coords from cartesian ones 笛卡尔坐标 到极坐标
// void IKArm::cart2polar(float x, float y, float& r, float& theta)
void IKArm::cart2polar(float a, float b, float& r, float& theta) {
  // Determine magnitude of cartesian coords
  //平面坐标系转为基座标系
  r = sqrt(a * a + b * b);

  // Don't try to calculate zero-magnitude vectors' angles
  if (r == 0) return;

  //使用atan2代替 b=y a=x
  theta = atan2(b, a);
}

// phi是末端和地平面的夹角
// Solve the angles for XY with a fixed attack angle 指定末端姿态
// phi末端对于水平面的夹角
// atan2 返回 y/x 的反正切值，以弧度表示，取值范围为(-π,π]
// atan 说明：值域为(-π/2，+π/2)
bool IKArm::solve(float x, float y, float z, float phi, float& base,
                  float& shoulder, float& elbow, float& wrist) {
  // Adjust coordinate system for base as ground plane
  /// Solve top-down view 解出基座旋转角度
  float _r, _base;
  cart2polar(x, y, _r, _base);
 // Serial.println(_r);
  //Serial.println(_base);
  // Check the range of the base 检查角度范围
  if (!_L0.inRange(_base)) {
    // If not in range, flip the angle 如果超出范围，跳出
    return false;
  } else  base = _base;
  //Serial.println(base);



  //手臂平面RZ
  //得到腕部坐标相对于肩关节坐标
  float _z = z - _L0.getLength();
  float rb = _r - _L3.getLength() * cos(phi);
  float zb = _z - _L3.getLength() * sin(phi);
    //Serial.println( _z);
	  //Serial.println(rw);
	   // Serial.println(zw);
  // Get polar system 极坐标
  float _LR, _alpha;
  cart2polar(rb, zb, _LR, _alpha);

  // Calculate the inner angle of the shoulder 肩关节
  // bool _Inverse::_cosrule(float opposite, float adjacent1, float adjacent2,
  // float& angle) bool _Inverse::_cosrule( 对边, 邻边1, 邻边2, 角度)
  float beta;
  if (!_cosrule(_L2.getLength(), _LR, _L1.getLength(), beta)) return false;

  // Calcula the inner angle of the elbow 肘关节
  float gamma;
  if (!_cosrule(_LR, _L1.getLength(), _L2.getLength(), gamma)) return false;

  // Solve the angles of the arm
  float _shoulder, _elbow, _wrist;
  _shoulder = _alpha - beta;
  _elbow = PI - gamma;                //补角
  _wrist = phi - _shoulder - _elbow;  // phi=_wrist+ _shoulder + _elbow

  // Check the range of each hinge
  if (!_L1.inRange(_shoulder) || !_L2.inRange(_elbow) || !_L3.inRange(_wrist)) {
    // If not in range, solve for the second solution
    _shoulder += 2 * beta;  // alpha + beta; 上轴位
    _elbow *= -1;
    _wrist = phi - _shoulder - _elbow;

    // Check the range for the second solution
    if (!_L1.inRange(_shoulder) || !_L2.inRange(_elbow) || !_L3.inRange(_wrist))
      return false;
  }

  // Return the solution

  shoulder = _shoulder;
  elbow = _elbow;
  wrist = _wrist;

  return true;
}
