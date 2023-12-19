#include <stdio.h> // printf를 사용하기 위한 헤더파일
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>   // log를 사용하기 위한 헤더파일
#include <unistd.h> // sleep을 사용하기 위한 헤더파일

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

const double clock_period = 0.5;

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
double velocity_equation(double used_propellent_mass, double initial_mass, double initial_velocity, double relative_velocity)
{
    double mass_ratio = (initial_mass - used_propellent_mass) / initial_mass;
    double ln_mass_ratio = log(mass_ratio);
    return initial_velocity - relative_velocity * ln_mass_ratio;
}

// Integrater
double integrate(double value, double buffer, double period)
{
    double integral = (value + buffer) / 2.0;
    return integral * period;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                      main                                      ||
// ! ||--------------------------------------------------------------------------------||
int main(void)
{
    initialize_parameters();

    double current_velocity = 0;
    double velocity_buffer = 0;
    double travel_distance = 0;

    while (!((propellent_mass - used_propellent_mass) < clock_period * consume_ratio))
    {
        used_propellent_mass += clock_period * consume_ratio;
        velocity_buffer = current_velocity;
        current_velocity = velocity_equation(used_propellent_mass, initial_mass, initial_velocity, relative_velocity);
        travel_distance += integrate(current_velocity, velocity_buffer, clock_period);
        printf("current_velocity: %f m/s, travel_distance: %f\n", current_velocity, travel_distance / 1000);
        usleep(500000);
    }

    printf("Awaiting 10s.\n");
    sleep(10); // 10초간 프로그램 일시 중지

    return 0;
}

// ! ||--------------------------------------------------------------------------------||
// ! ||                                      TO DO                                     ||
// ! ||--------------------------------------------------------------------------------||
// ! ||                       3. 궤적 출력하는 함수 만들기                                 ||
// ! ||                       4. 노이즈 추가하는 함수 만들기                               ||
// ! ||                       5. 전체적으로 조합하기                                      ||
// ! ||--------------------------------------------------------------------------------||