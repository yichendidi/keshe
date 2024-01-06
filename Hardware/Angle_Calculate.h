#ifndef __ANGLE_CALCULATE_H
#define __ANGLE_CALCULATE_H



// 变量定义
 
#define Kp 100.0f                        // 比例增益支配率收敛到加速度计/磁强计
#define Ki 0.002f                // 积分增益支配率的陀螺仪偏见的衔接
#define halfT 0.001f                // 采样周期的一半
 

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
#endif
