#ifndef CIRCLERUN_PAD_CONTROLLER_H
#define CIRCLERUN_PAD_CONTROLLER_H


#define RIGHT_ARM 1
#define LEFT_ARM  2
#define pi 3.14159265359

double right_arm_presets[1][7] = {{-0.90, 0.0,  0.04,  0.06, -0.35,  0.3, -pi}
                                  };
double left_arm_presets[1][7]  = {{0.90, 0.0, -0.04,  0.2, -2.7, -0.3,  pi}
                                 };

                                 //RIGHT ARM JOINTS: -0.905,  0.00255,  0.0276,  0.0578, -0.354,  0.292, -3.11,
                                 //LEFT ARM JOINTS:   0.917, -0.0404, -0.0393,  0.201, -2.68, -0.288,  2.98,

double joint_velcmd_limit[7] = {0.05,0.05,0.05,0.05,0.05,0.05,0.05};
               //RIGHT ARM JOINTS: -0.986,  0.247,  0.0915, -0.113, -0.15,  0.379, -3.44,
               //LEFT ARM JOINTS:   0.984,  0.24, -0.131,  0.00226, -2.88, -0.195,  3.25,

#endif
