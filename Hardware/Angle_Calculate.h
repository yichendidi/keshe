#ifndef __ANGLE_CALCULATE_H
#define __ANGLE_CALCULATE_H



// ��������
 
#define Kp 100.0f                        // ��������֧�������������ٶȼ�/��ǿ��
#define Ki 0.002f                // ��������֧���ʵ�������ƫ�����ν�
#define halfT 0.001f                // �������ڵ�һ��
 

void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
#endif
