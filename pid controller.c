#define WINDOW_MODE

#ifdef WINDOW_MODE
#include <unistd.h>  // sleep을 사용하기 위한 헤더파일
#include <windows.h> // pause를 사용하기 위한 헤더파일
#define _USE_MATH_DEFINES
#include <stdio.h> // printf를 사용하기 위한 헤더파일
#include <time.h>  // 랜덤시드로 time을 사용하기 위한 헤더파일
#endif

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h> // log를 사용하기 위한 헤더파일

#ifndef WINDOW_MODE
#define M_PI 3.14159265358979323846    // pi
#define M_PI_2 1.57079632679489661923  // pi/2
#define M_PI_4 0.785398163397448309616 // pi/4
#endif

// ! ||--------------------------------------------------------------------------------||
// ! ||                              Rocket Specifications                             ||
// ! ||--------------------------------------------------------------------------------||

const double gravity = 9.80665;

const int specific_impulse_1 = 263;
const int specific_impulse_2 = 421;
const int specific_impulse_3 = 421;

const int propellent_mass_1 = 2077000;
const int propellent_mass_2 = 456100;
const int propellent_mass_3 = 39136; // 3단은 두번에 나눠 점화한다 => state 3, 4로 나눔.
const int propellent_mass_4 = 83864;

const int burntime_1 = 168;
const int burntime_2 = 360;
const int burntime_3 = 165;
const int burntime_4 = 335;

const int stage_mass_1 = 137000;
const int stage_mass_2 = 40100;
const int stage_mass_3 = 15200;
const int lm = 15103;
const int cmsm = 11900; // command module and service module

const double clock_period = 1;

// ! ||--------------------------------------------------------------------------------||
// ! ||                              initialize parameters                             ||
// ! ||--------------------------------------------------------------------------------||

int total_mass_1, total_mass_2, total_mass_3, total_mass_4;
double initial_velocity, used_propellent_mass, specific_impulse, propellent_mass, burntime, initial_mass, consume_ratio;
double relative_velocity;
double final_mass, final_mass_ratio, final_speed;

void initialize_parameters(void)
{
    total_mass_1 = propellent_mass_1 + stage_mass_1 + propellent_mass_2 + stage_mass_2 + propellent_mass_3 + stage_mass_3 + propellent_mass_4 + lm + cmsm;
    total_mass_2 = propellent_mass_2 + stage_mass_2 + propellent_mass_3 + stage_mass_3 + propellent_mass_4 + lm + cmsm;
    total_mass_3 = propellent_mass_3 + stage_mass_3 + propellent_mass_4 + lm + cmsm;
    total_mass_4 = stage_mass_3 + propellent_mass_4 + lm + cmsm;

    // at the beginning of the program
    initial_velocity = 0;
    used_propellent_mass = 0;
    specific_impulse = specific_impulse_1;
    propellent_mass = propellent_mass_1;
    burntime = burntime_1;
    initial_mass = total_mass_1;
    consume_ratio = propellent_mass / burntime;

    // Tsiolkovsky Rocket Equation
    relative_velocity = specific_impulse * gravity;

    // at the end of the program
    final_mass = total_mass_2;
    final_mass_ratio = (initial_mass - propellent_mass) / initial_mass;
    final_speed = initial_velocity + (-1) * relative_velocity * log(final_mass_ratio);
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                               Velocity Calculator                              ||
// ! ||--------------------------------------------------------------------------------||

// Tsiolkovsky Rocket Equation
double ln_mass_ratio = 0;
double velocity_equation(void)
{
    double mass_ratio = (initial_mass - used_propellent_mass) / initial_mass;
    ln_mass_ratio = log(mass_ratio);
    return initial_velocity - relative_velocity * ln_mass_ratio;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                   Integrater                                   ||
// ! ||--------------------------------------------------------------------------------||
double integrate(double value, double buffer, double period)
{
    double integral = (value + buffer) / 2.0;
    return integral * period;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                Parameter Update                                ||
// ! ||--------------------------------------------------------------------------------||
void propellent_update(void)
{
    used_propellent_mass += clock_period * consume_ratio;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                               Velocity Calulator                               ||
// ! ||--------------------------------------------------------------------------------||
double velocity = 0;
double travel_distance = 0;
void velocity_calculator(void)
{
    double _vel;
    _vel = velocity;
    velocity = velocity_equation();
    travel_distance += integrate(velocity, _vel, clock_period);
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                       R2D                                      ||
// ! ||--------------------------------------------------------------------------------||
double R2D(double radian)
{
    return radian * 180 / M_PI;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                              Trajectory Generator                              ||
// ! ||--------------------------------------------------------------------------------||
double angular_velocity = 0;
double accumulated_angle = 0;
double desired_pitch = 0;
void trajectory_generator(double velocity, int which_path)
{
    double _av, av;
    switch (which_path)
    {
    case 1:
        /* code */
        _av = R2D(angular_velocity);
        angular_velocity = velocity / 400000;
        av = R2D(angular_velocity);
        desired_pitch = (180 - av) * 0.5 - accumulated_angle;
        accumulated_angle += integrate(av, _av, clock_period);
        desired_pitch = 90 - desired_pitch;
        break;

    default:
        desired_pitch = 0;
        break;
    }
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                   Differnece                                   ||
// ! ||--------------------------------------------------------------------------------||
double difference(double a, double b)
{
    return (a - b) / clock_period;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                 PID Controller                                 ||
// ! ||--------------------------------------------------------------------------------||
double error = 0;
double kp = 5.6;
double kd = -3;
double printpid = 0;
double pid_controller(double desired, double current)
{
    double _error, p, d, pid;
    _error = error;
    error = desired - current;
    kp = 5.6;
    kd = -3;
    p = kp * error;
    d = kd * difference(error, _error);
    pid = p + d;

    // printf("error: %lf, p: %lf, d: %lf, pid: %lf \n", error, p, d, pid);
    return pid;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                           Control Signal Convsersion                           ||
// ! ||--------------------------------------------------------------------------------||
double spacecraft_width = 10.1;
double spacecraft_height = 110.6;
double thrust = 0;
double mass_moment_of_inertia()
{
    double mmoi;
    mmoi = 0.5 * (initial_mass - used_propellent_mass) * (0.5 * spacecraft_width) * (0.5 * spacecraft_width);
    // printf("mmoi: %lf \n", mmoi);
    return mmoi;
}
double cal_torque(double pid, double mass_moment_of_inertia)
{
    double torque;
    torque = pid * mass_moment_of_inertia;
    // printf("torque: %lf \n", torque);
    return torque;
}
double cal_gimbal_angle(double torque)
{
    double moment_arm, _gimbal_angle, gimbal_angle;
    thrust = velocity * consume_ratio;
    moment_arm = 0.5 * spacecraft_height;
    _gimbal_angle = torque / thrust / moment_arm;
    if (_gimbal_angle > 1.0)
    {
        _gimbal_angle = 1.0;
    }
    if (_gimbal_angle < -1.0)
    {
        _gimbal_angle = -1.0;
    }
    gimbal_angle = asin(_gimbal_angle);

    // printf("thrust: %lf, _gimbal_angle: %lf, gimbal_angle: %lf \n", thrust, _gimbal_angle, gimbal_angle);
    return gimbal_angle;
}
double saturation(double gimbal_angle)
{
    if (gimbal_angle > 10)
    {
        gimbal_angle = 10;
    }
    else if (gimbal_angle < -10)
    {
        gimbal_angle = -10;
    }
    return gimbal_angle;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                      Plant                                     ||
// ! ||--------------------------------------------------------------------------------||
double incremental_angle = 0;
double pitch = 0;
double plant(double gimbal_angle)
{
    double a, b, _dff;
    // moment
    a = (1.0 / 12.0) * (initial_mass - used_propellent_mass) * (spacecraft_height * spacecraft_height);
    b = (1.0 / 4.0) * (initial_mass - used_propellent_mass) * (spacecraft_width * spacecraft_width);
    _dff = incremental_angle;
    // incremented angle
    incremental_angle = sin(gimbal_angle) * (thrust) / (a + b);
    // pitch position
    pitch += integrate(incremental_angle, _dff, clock_period);

    // printf("a: %lf, b: %lf, incremental_angle: %lf, pitch: %lf \n", a, b, incremental_angle, pitch);
    return pitch;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                              White Noise Generator                             ||
// ! ||--------------------------------------------------------------------------------||
double whiteNoise(double amplitude)
{
    double noise;
    noise = (double)rand() / RAND_MAX;
    noise = 3.0 * noise * amplitude;
    // printf("noise: %lf \n", noise);
    return (double)noise;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                      main                                      ||
// ! ||--------------------------------------------------------------------------------||
int main(void)
{
    double tt, current_pitch, noise, pid, mmoi, torque, gimbal_angle;
    tt = 0;
    current_pitch = 0;
#ifdef WINDOW_MODE
    srand((unsigned int)time(NULL)); // seed configuration
#endif
    initialize_parameters();

    while (!((propellent_mass - used_propellent_mass) < clock_period * consume_ratio))
    {
        propellent_update();
        velocity_calculator();
        trajectory_generator(velocity, 1);

        noise = whiteNoise(1.0);

        // pid = pid_controller(desired_pitch, current_pitch);
        pid = pid_controller(desired_pitch, current_pitch + noise);
        mmoi = mass_moment_of_inertia();
        torque = cal_torque(pid, mmoi);
        gimbal_angle = cal_gimbal_angle(torque);
        gimbal_angle = saturation(gimbal_angle);
        current_pitch = plant(gimbal_angle);
        current_pitch = R2D(current_pitch);

#ifdef WINDOW_MODE
        // printf("current_velocity: %lf m/s, travel_distance: %lf km \n", velocity, travel_distance / 1000);
        // printf("desired_pitch: %lf, accumulated_angle: %lf \n", desired_pitch, accumulated_angle);
        printf("current_pitch: %lf, gimbal_angle: %lf, error: %lf \n", current_pitch, gimbal_angle, error);
        usleep(50000);
#endif
        tt += clock_period;
    }

#ifdef WINDOW_MODE
    printf("time: %lf s, left propellent: %lf kg \n", tt, propellent_mass - used_propellent_mass);
    system("pause");
    return 0;
#endif
    while (1)
    {
        /* code */
    }
}