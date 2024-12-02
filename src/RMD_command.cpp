#include "RMD_COMMAND.h"

void RMD_COMMAND::RMD_COMMAND_setup()
{
    output_degree = 0.;
    degree = 0.;
    torque = 0.;
    speed = 0.;
    pulse = 0.;
    pulse_to_degree = 0.00001525878 * 360 / Ratio;
}

// ============================ RMD command ============================
void RMD_COMMAND::ENCODER_ZERO_POSITON(int s, int ID, struct can_frame frame)
{
    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0x19;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::TORQUE_CLOSED_LOOP_CONTROL(int s, int ID, int torque, struct can_frame frame)
{ /// torque = tau = (float)(~~)
    int16_t torque_low_byte = 0;
    int16_t torque_high_byte = 0;

    /*if (s == 16)
    {
        if (ID == 1)
        {
            torque = torque;
        }
        else if (ID == 2)
        {
            torque = -1 * torque; // base joint
        }
    }

    else if (s == 17) // can11
    {
        if (ID == 1)
        {
            torque = torque;
        }
        else if (ID == 2)
        {
            torque = -1 * torque;
        }
    }
    else if (s == 18) // can12
    {
        if (ID == 1)
        {
            torque = -1 * torque;
        }
        else if (ID == 2)
        {
            torque = torque;
        }
    }
    else if (s == 19) // can13
    {
        if (ID == 1)
        {
            torque = -1 * torque;
        }
        else if (ID == 2)
        {
            torque = -1 * torque;
        }
    }
    */

    torque_low_byte = (torque >> 0) & (0x00FF);
    torque_high_byte = (torque >> 8) & (0x00FF);

    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0xA1;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = torque_low_byte;
    frame.data[5] = torque_high_byte;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    try
    {
        int ret = write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame);
        if (ret < 0)
        {
            if (ret == EAGAIN || ret == EWOULDBLOCK)
            {
                printf("No data available.\n");
                return;
            }
            else
            {
                perror("Write");
                return;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
}

void RMD_COMMAND::RPM_CLOSED_LOOP_CONTROL(int s, int ID, int rpm, struct can_frame frame)
{
    int32_t rpm_low_byte = 0;
    int32_t rpm_2nd_byte = 0;
    int32_t rpm_3rd_byte = 0;
    int32_t rpm_high_byte = 0;

    rpm_low_byte = (rpm >> 0) & (0x00FF);
    rpm_2nd_byte = (rpm >> 8) & (0x00FF);
    rpm_3rd_byte = (rpm >> 16) & (0x00FF);
    rpm_high_byte = (rpm >> 24) & (0x00FF);

    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0xA2;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = rpm_low_byte;
    frame.data[5] = rpm_2nd_byte;
    frame.data[6] = rpm_3rd_byte;
    frame.data[7] = rpm_high_byte;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::POSITION_CONTROL_A4(int s, int ID, float f_angle, int maxspeed, struct can_frame frame)
{
    // 각도를 0.01도 단위로 변환 (0.01도 단위로 표현하기 위해 100을 곱함)
    int32_t angle = static_cast<int32_t>(f_angle * 100);

    uint16_t speed = static_cast<uint16_t>(maxspeed);

    uint8_t angle_low_byte = (angle >> 0) & 0xFF;
    uint8_t angle_mid_low_byte = (angle >> 8) & 0xFF;
    uint8_t angle_mid_high_byte = (angle >> 16) & 0xFF;
    uint8_t angle_high_byte = (angle >> 24) & 0xFF;

    // 속도를 2바이트로 분리
    uint8_t speed_low_byte = (speed >> 0) & 0xFF;
    uint8_t speed_high_byte = (speed >> 8) & 0xFF;

    // CAN 프레임 설정
    frame.can_id = 0x140 + ID; // CAN ID: 0x140 + 모터 ID
    frame.can_dlc = 8;         // 데이터 길이: 8바이트

    frame.data[0] = 0xA4;
    frame.data[1] = 0x00;
    frame.data[2] = speed_low_byte;      // 최대 속도 낮은 바이트
    frame.data[3] = speed_high_byte;     // 최대 속도 높은 바이트
    frame.data[4] = angle_low_byte;      // 각도 낮은 바이트
    frame.data[5] = angle_mid_low_byte;  // 각도 중간 낮은 바이트
    frame.data[6] = angle_mid_high_byte; // 각도 중간 높은 바이트
    frame.data[7] = angle_high_byte;     // 각도 높은 바이트

    // CAN 프레임 전송
    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::REQUEST_MOTOR_STATUS(int s, int ID, struct can_frame frame)
{
    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0x9C;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::REQUEST_DATA(int command, int s, int ID, struct can_frame frame)
{
    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = command;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}

void RMD_COMMAND::WRITE_MOTOR_OFF(int s, int ID, struct can_frame frame)
{
    frame.can_id = 0x140 + ID;
    frame.can_dlc = 8;

    frame.data[0] = 0x80;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x00;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    if (write(s, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame))
    {
        perror("Write");
        return;
    }
}



// void *MainActiveNode::pthread_main()
// { // 현재 pthread
//   struct period_info pinfo;
//   //* Set Main Thread Period 주기 설정
//   pinfo.period_ns = ns_TwoMilSec; // 2ms ns_TwoMilSec ns_FiveMilSec
//   usleep(500);
//   clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
//   auto thread_start = std::chrono::high_resolution_clock::now();
//   {
//     std::lock_guard<std::mutex> lock(_mtx);
//     planar_flag = global_planar_flag;
//   }
//   //* -------------------------------------------------------------------------Thread Loop-------------------------------------------------------
//   while ((!main_thread_exit))
//   {
//     pthread_testcancel();                                  // make thread cancel point to saftey exit thread
//     clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_1); // time1 load
//     // input loop function
//     // Read buffer - motor state data : torque, speed, angle
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         if (i < 2)
//           READ_TORQUE_SPEED_ANGLE(cansock[i], i, frame[i]); // Read for first two CAN channels
//         else
//           READ_TORQUE_SPEED_ANGLE_X4(cansock[i], i, frame[i]); // Read for remaining CAN channels
//       }
//     }
//     GetState();
//     // Lock the mutex only when accessing shared resources
//     {
//       std::lock_guard<std::mutex> lock(_mtx);
//       // Update joint positions and velocities
//       for (int n = 0; n < 4; n++)
//       {
//         joint[2 * n].CurrentPos = motor_state[n].theta[0];
//         joint[2 * n].CurrentVel = motor_state[n].speed[0];
//         joint[2 * n + 1].CurrentPos = motor_state[n].theta[1];
//         joint[2 * n + 1].CurrentVel = motor_state[n].speed[1];
//       }
//       if (std::abs(x_pixel) <= 0.005f && !planar_flag)
//       {
//         planar_flag = true;
//         runtime = 0.0; // Reset runtime for planar motion
//       }

//       // 이 코드에서 base_flag 부분을 빼고 planar 만 남기기(x_pixel==0으로 가정). 활성화 시 rpm = 0.;
//       // 이후 joint[2] joint[3](shoulder) joint[4](elbow) joint[6](wrist=90*D2R) joint[5](wrist) IK
//       // runtime=0.; 으로 초기화, 모터 구동
//       // j8(gripper) grip motion->이후 다시 복귀하는 알고리즘
//       // 이야기를 듣고 마음을 알아보는 것 -- 경청

//       if (planar_flag) // Planar motion control
//       {
//         if (runtime == 0.0)
//         {
//           float th_y = std::asin(y_pixel / z_depth);
//           float th_z = std::acos((L1 * L1 + z_depth * z_depth - L2 * L2) / (2 * L1 * z_depth));
//           float th_shoulder = PI - (th_z - th_y);
//           float th_elbow = std::acos((L1 * L1 - z_depth * z_depth + L2 * L2) / (2 * L1 * L2));
//           // j3 <---> th3 //

//           // joint[0].RefTargetPos = ; // pitch
//           joint[0].InitPos = motor_state[0].theta[0]; // can10 motor 1
//           joint[0].InitVel = motor_state[0].speed[0];
//           joint[0].RefTargetPos = joint[0].InitPos; // motor is not in use.

//           joint[1].RefTargetPos = -th_elbow; // can10 motor 2
//           joint[1].InitPos = joint[1].CurrentPos;
//           joint[1].InitVel = joint[1].CurrentVel;

//           joint[2].RefTargetPos = -th_shoulder; // can11 motor 1 2
//           joint[2].InitPos = joint[2].CurrentPos;
//           joint[2].InitVel = joint[2].CurrentVel;
//           joint[3].RefTargetPos = th_shoulder;
//           joint[3].InitPos = motor_state[1].theta[1];
//           joint[3].InitVel = motor_state[1].speed[1];

//           joint[4].RefTargetPos = 90 * D2R; // for yaw to pitch of joint[5]
//           joint[4].InitPos = motor_state[2].theta[0];
//           joint[4].InitVel = motor_state[2].speed[0];

//           joint[5].RefTargetPos = -30 * D2R; // pitch
//           joint[5].InitPos = motor_state[2].theta[1];
//           joint[5].InitVel = motor_state[2].speed[1];

//           joint[6].InitPos = motor_state[3].theta[0];
//           joint[6].InitVel = motor_state[3].speed[0];
//           joint[6].RefTargetPos = joint[6].InitPos; // motor of Mr. ChanWoo is not in use.

//           joint[7].RefTargetPos = -300 * D2R; // opening
//           joint[7].InitPos = motor_state[3].theta[1];
//           joint[7].InitVel = motor_state[3].speed[1];
//         }

//         if (runtime < 3000.0)
//         {
//           joint[0].RefPos = joint[0].InitPos;
//           joint[0].RefVel = joint[0].InitVel;

//           joint[1].RefPos = func_1_cos(runtime, joint[1].InitPos, joint[1].RefTargetPos, 3000.0);
//           joint[1].RefVel = dt_func_1_cos(runtime, joint[1].InitPos, joint[1].RefTargetPos, 3000.0);

//           joint[2].RefPos = func_1_cos(runtime, joint[2].InitPos, joint[2].RefTargetPos, 3000.0);
//           joint[2].RefVel = dt_func_1_cos(runtime, joint[2].InitPos, joint[2].RefTargetPos, 3000.0);

//           joint[3].RefPos = func_1_cos(runtime, joint[3].InitPos, joint[3].RefTargetPos, 3000.0);
//           joint[3].RefVel = dt_func_1_cos(runtime, joint[3].InitPos, joint[3].RefTargetPos, 3000.0);

//           joint[4].RefPos = func_1_cos(runtime, joint[4].InitPos, joint[4].RefTargetPos, 3000.0);
//           joint[4].RefVel = dt_func_1_cos(runtime, joint[4].InitPos, joint[4].RefTargetPos, 3000.0);

//           joint[6].RefPos = joint[6].InitPos;
//           joint[6].RefVel = joint[6].InitVel;

//           joint[5].RefPos = func_1_cos(runtime, joint[5].InitPos, joint[5].RefTargetPos, 3000.0);
//           joint[5].RefVel = dt_func_1_cos(runtime, joint[5].InitPos, joint[5].RefTargetPos, 3000.0);

//           joint[7].RefPos = func_1_cos(runtime, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
//           joint[7].RefVel = dt_func_1_cos(runtime, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
//         }
//         else if (runtime == 3000.0)
//         {
//           for (int i = 0; i < 7; i++)
//           {
//             joint[i].RefPos = joint[i].RefPos;
//             joint[i].RefVel = joint[i].RefVel;
//           }
//           joint[7].RefTargetPos = 300 * D2R; // gripping motion
//           joint[7].InitPos = motor_state[3].theta[1];
//           joint[7].InitVel = motor_state[3].speed[1];
//         }
//         else if (3000.0 < runtime && runtime < 6000.0)
//         {
//           joint[7].RefPos = func_1_cos(runtime - 3000, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
//           joint[7].RefVel = dt_func_1_cos(runtime - 3000, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
//         }
//         else if (runtime == 6000.0)
//         {
//           joint[1].RefTargetPos = th_elbow;
//           joint[1].InitPos = joint[1].CurrentPos;
//           joint[1].InitVel = joint[1].CurrentVel;

//           joint[2].RefTargetPos = th_shoulder;
//           joint[2].InitPos = joint[2].CurrentPos;
//           joint[2].InitVel = joint[2].CurrentVel;
//           joint[3].RefTargetPos = -th_shoulder;
//           joint[3].InitPos = motor_state[1].theta[1];
//           joint[3].InitVel = motor_state[1].speed[1];

//           joint[4].RefTargetPos = -90 * D2R;
//           joint[4].InitPos = motor_state[2].theta[0];
//           joint[4].InitVel = motor_state[2].speed[0];

//           joint[5].RefTargetPos = 30 * D2R;
//           joint[5].InitPos = motor_state[2].theta[1];
//           joint[5].InitVel = motor_state[2].speed[1];

//           // joint[6].RefTargetPos = -90 * D2R;
//           // joint[6].InitPos = motor_state[3].theta[0];
//           // joint[6].InitVel = motor_state[3].speed[0];

//           // joint[7].RefTargetPos = 300 * D2R; // gripping motion
//           // joint[7].InitPos = motor_state[3].theta[1];
//           // joint[7].InitVel = motor_state[3].speed[1];
//         }
//         else if (6000.0 < runtime && runtime < 9000.0)
//         {
//           for (int i = 1; i < 8; i++)
//           {
//             joint[i].RefPos = func_1_cos(runtime - 6000, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
//             joint[i].RefVel = dt_func_1_cos(runtime - 6000, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
//           }
//         }
//         else
//         {
//           for (int i = 0; i < 8; i++)
//           {
//             joint[i].RefPos = joint[i].RefPos;
//             joint[i].RefVel = joint[i].RefVel;
//           }
//         }
//         // else
//         // {
//         //   planar_flag = false;
//         // }
//       }

//       std::cout << "Joint_Ref_POS" << joint[4].RefPos * R2D << std::endl;
//       std::cout << "Joint_Cur_POS" << joint[4].CurrentPos * R2D << std::endl;
//       std::cout << "Joint_RefTarget_POS" << joint[4].RefTargetPos * R2D << std::endl;
//       std::cout << "Joint_Init_POS" << joint[4].InitPos * R2D << std::endl;
//       std::cout << "RUNTIME" << runtime << std::endl;
//     }
//     // calc torque(real range: -30~30A) Mapping to -2000~2000
//     ComputeTorque();
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[i], j + 1, tau[i][j], frame[i]);
//         // Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[i], j+1, tau[i][j],frame[i]);
//       }
//     }

//     runtime = (runtime + (pinfo.period_ns / 1000000.)); //-> runtime += thread period
//     pinfo.next_period.tv_nsec += pinfo.period_ns;
//     while (pinfo.next_period.tv_nsec >= ns_OneSec) // 1000000000 nsec = 1sec
//     {
//       /* timespec nsec overflow */
//       pinfo.next_period.tv_sec++;
//       pinfo.next_period.tv_nsec -= ns_OneSec; // clear nanosec
//     }
//     clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo.next_period, NULL);
//     clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_2); // tim2 load

//     // std::cout << " ============================= " << std::endl;
//     // std::cout << "Pthread_time: "<< (double)(pinfo.current_time_2.tv_nsec - pinfo.current_time_1.tv_nsec) / ns_OneMilSec << " ms " << std::endl;
//     // std::cout << " ============================= " << std::endl;
//     auto thread_end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double, std::milli> thread_time = thread_end - thread_start;
//     thread_timer = thread_time.count();
//     // std::cout << "end-Thread Time: " << thread_time.count() << " ms" << std::endl;
//     // printf("end-Thread Time: %.5f ms\n", thread_time.count());
//     thread_start = std::chrono::high_resolution_clock::now();
//     //-----------logging part-----------//
//     fprintf(Minirok_data, "%f \t", runtime);

//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", motor_state[i].theta[j]);
//       }
//     }
//     // std::cout<<"mt done"<<std::endl;
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", motor_state[i].speed[j]);
//       }
//     }
//     // std::cout<<"ms done"<<std::endl;

//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", (float)(tau[i][j]));
//       }
//     }
//     // std::cout<<"tq done"<<std::endl;
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", motor_state[i].torque[j]);
//       }
//     }

//     fprintf(Minirok_data, "%f \t", thread_timer);

//     // fprintf temperature
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", motor_state[i].temp[j]);
//       }
//     }

//     fprintf(Minirok_data, "\n");
//     //-----------logging part-----------//
//   } // end of while loop
//   return NULL;
// }


// void *MainActiveNode::pthread_main()
// {
//   struct period_info pinfo;
//   //* Set Main Thread Period 주기 설정
//   pinfo.period_ns = ns_TwoMilSec; // 2ms ns_TwoMilSec ns_FiveMilSec
//   usleep(500);
//   clock_gettime(CLOCK_MONOTONIC, &(pinfo.next_period));
//   auto thread_start = std::chrono::high_resolution_clock::now();

//   {
//     std::lock_guard<std::mutex> lock(_mtx);
//     planar_flag = global_planar_flag;
//   }

//   //* -------------------------------------------------------------------------Thread Loop-------------------------------------------------------
//   while ((!main_thread_exit))
//   {
//     pthread_testcancel();                                  // make thread cancel point to saftey exit thread
//     clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_1); // time1 load
//     // input loop function
//     // Read buffer - motor state data : torque, speed, angle
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         if (i < 2)
//           READ_TORQUE_SPEED_ANGLE(cansock[i], i, frame[i]); // Read for first two CAN channels
//         else
//           READ_TORQUE_SPEED_ANGLE_X4(cansock[i], i, frame[i]); // Read for remaining CAN channels
//       }
//     }
//     GetState();
//     // Lock the mutex only when accessing shared resources
//     {
//       std::lock_guard<std::mutex> lock(_mtx);
//       // Update joint positions and velocities
//       for (int n = 0; n < 4; n++)
//       {
//         joint[2 * n].CurrentPos = motor_state[n].theta[0];
//         joint[2 * n].CurrentVel = motor_state[n].speed[0];
//         joint[2 * n + 1].CurrentPos = motor_state[n].theta[1];
//         joint[2 * n + 1].CurrentVel = motor_state[n].speed[1];
//       }

//       if (std::abs(x_pixel) <= 0.005f && !planar_flag)
//       {
//         planar_flag = true;
//         runtime = 0.0; // Reset runtime for planar motion
//       }

//       // 이 코드에서 base_flag 부분을 빼고 planar 만 남기기(x_pixel==0으로 가정). 활성화 시 rpm = 0.;
//       // 이후 joint[2] joint[3](shoulder) joint[4](elbow) joint[6](wrist=90*D2R) joint[5](wrist) IK
//       // runtime=0.; 으로 초기화, 모터 구동
//       // j8(gripper) grip motion->이후 다시 복귀하는 알고리즘
//       // 이야기를 듣고 마음을 알아보는 것 -- 경청

//       if (planar_flag) // Planar motion control
//       {
//         if (runtime == 0.0)
//         {
//           float th_y = std::asin(y_pixel / z_depth);
//           float th_z = std::acos((L1 * L1 + z_depth * z_depth - L2 * L2) / (2 * L1 * z_depth));
//           float th_shoulder = PI - (th_z - th_y);
//           float th_elbow = std::acos((L1 * L1 - z_depth * z_depth + L2 * L2) / (2 * L1 * L2));
//           // j3 <---> th3 //

//           // joint[0].RefTargetPos = ; // pitch
//           joint[0].InitPos = motor_state[0].theta[0]; // can10 motor 1
//           joint[0].InitVel = motor_state[0].speed[0];
//         //   joint[0].RefTargetPos = joint[0].InitPos; // motor is not in use.

//           joint[1].RefTargetPos = -th_elbow; // can10 motor 2
//           joint[1].InitPos = joint[1].CurrentPos;
//           joint[1].InitVel = joint[1].CurrentVel;

//           joint[2].RefTargetPos = -th_shoulder; // can11 motor 1 2
//           joint[2].InitPos = joint[2].CurrentPos;
//           joint[2].InitVel = joint[2].CurrentVel;
//           joint[3].RefTargetPos = th_shoulder;
//           joint[3].InitPos = motor_state[1].theta[1];
//           joint[3].InitVel = motor_state[1].speed[1];

//           joint[4].RefTargetPos = 90 * D2R; // for yaw to pitch of joint[5]
//           joint[4].InitPos = motor_state[2].theta[0];
//           joint[4].InitVel = motor_state[2].speed[0];

//           joint[5].RefTargetPos = -30 * D2R; // pitch
//           joint[5].InitPos = motor_state[2].theta[1];
//           joint[5].InitVel = motor_state[2].speed[1];

//           joint[6].InitPos = motor_state[3].theta[0];
//           joint[6].InitVel = motor_state[3].speed[0];
//           joint[6].RefTargetPos = joint[6].InitPos; // motor of Mr. ChanWoo is not in use.

//           joint[7].RefTargetPos = -300 * D2R; // opening
//           joint[7].InitPos = motor_state[3].theta[1];
//           joint[7].InitVel = motor_state[3].speed[1];
//         }

//         if (runtime < 3000.0)
//         {
//           joint[0].RefPos = joint[0].InitPos;
//           joint[0].RefVel = joint[0].InitVel;

//           joint[1].RefPos = func_1_cos(runtime, joint[1].InitPos, joint[1].RefTargetPos, 3000.0);
//           joint[1].RefVel = dt_func_1_cos(runtime, joint[1].InitPos, joint[1].RefTargetPos, 3000.0);

//           joint[2].RefPos = func_1_cos(runtime, joint[2].InitPos, joint[2].RefTargetPos, 3000.0);
//           joint[2].RefVel = dt_func_1_cos(runtime, joint[2].InitPos, joint[2].RefTargetPos, 3000.0);

//           joint[3].RefPos = func_1_cos(runtime, joint[3].InitPos, joint[3].RefTargetPos, 3000.0);
//           joint[3].RefVel = dt_func_1_cos(runtime, joint[3].InitPos, joint[3].RefTargetPos, 3000.0);

//           joint[4].RefPos = func_1_cos(runtime, joint[4].InitPos, joint[4].RefTargetPos, 3000.0);
//           joint[4].RefVel = dt_func_1_cos(runtime, joint[4].InitPos, joint[4].RefTargetPos, 3000.0);

//           joint[6].RefPos = joint[6].InitPos;
//           joint[6].RefVel = joint[6].InitVel;

//           joint[5].RefPos = func_1_cos(runtime, joint[5].InitPos, joint[5].RefTargetPos, 3000.0);
//           joint[5].RefVel = dt_func_1_cos(runtime, joint[5].InitPos, joint[5].RefTargetPos, 3000.0);

//           joint[7].RefPos = func_1_cos(runtime, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
//           joint[7].RefVel = dt_func_1_cos(runtime, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
//         }
//         else if (runtime == 3000.0)
//         {
//           for (int i = 0; i < 7; i++)
//           {
//             joint[i].RefPos = joint[i].RefPos;
//             joint[i].RefVel = joint[i].RefVel;
//           }
//           joint[7].RefTargetPos = 300 * D2R; // gripping motion
//           joint[7].InitPos = motor_state[3].theta[1];
//           joint[7].InitVel = motor_state[3].speed[1];
//         }
//         else if (3000.0 < runtime && runtime < 6000.0)
//         {
//           joint[7].RefPos = func_1_cos(runtime - 3000, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
//           joint[7].RefVel = dt_func_1_cos(runtime - 3000, joint[7].InitPos, joint[7].RefTargetPos, 3000.0);
//         }
//         else if (runtime == 6000.0)
//         {
//           joint[1].RefTargetPos = th_elbow;
//           joint[1].InitPos = joint[1].CurrentPos;
//           joint[1].InitVel = joint[1].CurrentVel;

//           joint[2].RefTargetPos = th_shoulder;
//           joint[2].InitPos = joint[2].CurrentPos;
//           joint[2].InitVel = joint[2].CurrentVel;
//           joint[3].RefTargetPos = -th_shoulder;
//           joint[3].InitPos = motor_state[1].theta[1];
//           joint[3].InitVel = motor_state[1].speed[1];

//           joint[4].RefTargetPos = -90 * D2R;
//           joint[4].InitPos = motor_state[2].theta[0];
//           joint[4].InitVel = motor_state[2].speed[0];

//           joint[5].RefTargetPos = 30 * D2R;
//           joint[5].InitPos = motor_state[2].theta[1];
//           joint[5].InitVel = motor_state[2].speed[1];

//           // joint[6].RefTargetPos = -90 * D2R;
//           // joint[6].InitPos = motor_state[3].theta[0];
//           // joint[6].InitVel = motor_state[3].speed[0];

//           // joint[7].RefTargetPos = 300 * D2R; // gripping motion
//           // joint[7].InitPos = motor_state[3].theta[1];
//           // joint[7].InitVel = motor_state[3].speed[1];
//         }
//         else if (6000.0 < runtime && runtime < 9000.0)
//         {
//           for (int i = 1; i < 8; i++)
//           {
//             joint[i].RefPos = func_1_cos(runtime - 6000, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
//             joint[i].RefVel = dt_func_1_cos(runtime - 6000, joint[i].InitPos, joint[i].RefTargetPos, 3000.0);
//           }
//         }
//         else
//         {
//           for (int i = 0; i < 8; i++)
//           {
//             joint[i].RefPos = joint[i].RefPos;
//             joint[i].RefVel = joint[i].RefVel;
//           }
//         }
//         // else
//         // {
//         //   planar_flag = false;
//         // }
//       }

//       std::cout << "Joint_Ref_POS" << joint[4].RefPos * R2D << std::endl;
//       std::cout << "Joint_Cur_POS" << joint[4].CurrentPos * R2D << std::endl;
//       std::cout << "Joint_RefTarget_POS" << joint[4].RefTargetPos * R2D << std::endl;
//       std::cout << "Joint_Init_POS" << joint[4].InitPos * R2D << std::endl;
//       std::cout << "RUNTIME" << runtime << std::endl;
//     }
//     // calc torque(real range: -30~30A) Mapping to -2000~2000
//     ComputeTorque();
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[i], j + 1, tau[i][j], frame[i]);
//         // Rmd.TORQUE_CLOSED_LOOP_CONTROL(cansock[i], j+1, tau[i][j],frame[i]);
//       }
//     }

//     runtime = (runtime + (pinfo.period_ns / 1000000.)); //-> runtime += thread period
//     pinfo.next_period.tv_nsec += pinfo.period_ns;
//     while (pinfo.next_period.tv_nsec >= ns_OneSec) // 1000000000 nsec = 1sec
//     {
//       /* timespec nsec overflow */
//       pinfo.next_period.tv_sec++;
//       pinfo.next_period.tv_nsec -= ns_OneSec; // clear nanosec
//     }
//     clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &pinfo.next_period, NULL);
//     clock_gettime(CLOCK_MONOTONIC, &pinfo.current_time_2); // tim2 load

//     // std::cout << " ============================= " << std::endl;
//     // std::cout << "Pthread_time: "<< (double)(pinfo.current_time_2.tv_nsec - pinfo.current_time_1.tv_nsec) / ns_OneMilSec << " ms " << std::endl;
//     // std::cout << " ============================= " << std::endl;
//     auto thread_end = std::chrono::high_resolution_clock::now();
//     std::chrono::duration<double, std::milli> thread_time = thread_end - thread_start;
//     thread_timer = thread_time.count();
//     // std::cout << "end-Thread Time: " << thread_time.count() << " ms" << std::endl;
//     // printf("end-Thread Time: %.5f ms\n", thread_time.count());
//     thread_start = std::chrono::high_resolution_clock::now();
//     //-----------logging part-----------//
//     fprintf(Minirok_data, "%f \t", runtime);

//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", motor_state[i].theta[j]);
//       }
//     }
//     // std::cout<<"mt done"<<std::endl;
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", motor_state[i].speed[j]);
//       }
//     }
//     // std::cout<<"ms done"<<std::endl;

//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", (float)(tau[i][j]));
//       }
//     }
//     // std::cout<<"tq done"<<std::endl;
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", motor_state[i].torque[j]);
//       }
//     }

//     fprintf(Minirok_data, "%f \t", thread_timer);

//     // fprintf temperature
//     for (int i = 0; i < NUM_OF_CANABLE; i++)
//     {
//       for (int j = 0; j < NUM_OF_MOTOR; j++)
//       {
//         fprintf(Minirok_data, "%f \t", motor_state[i].temp[j]);
//       }
//     }

//     fprintf(Minirok_data, "\n");
//     //-----------logging part-----------//
//   } // end of while loop
//   return NULL;
// }

