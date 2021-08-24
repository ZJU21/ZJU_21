/* SunArmIK
 * Inverse Kinematics solver for three degrees of freedom
 * Inverse Kinematics solver for four degrees of freedom
 */
#ifndef IK_H_INCLUDED
#define IK_H_INCLUDED

extern const float L_UPPERARM, L_FOREARM, L_OFFSET;

// Get polar coords from cartesian ones 卡坐标转极坐标
void cart2polar(float a, float b, float& r, float& theta);

// Get angle from a triangle using cosine rule  余弦定理获得角度
bool cosangle(float opp, float adj1, float adj2, float& theta);

// Solve angles!  结算角度
bool solve(float x, float y, float z, float& a0, float& a1, float& a2);

#endif // IK_H_INCLUDED
