


struct pid_param
{
    int kp;
    int ki;
    int kd;
};

struct imu_data
{
    float angle_x;
    float angle_y;
    float angle_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float q0;
    float q1;
    float q2;
    float q3;
};