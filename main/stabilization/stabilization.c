#include "stabilization.h"
#include <stdio.h>

// int sign(double val)
// {
//     return (0.0 < val) - (val < 0.0);
// }

// double abs_d(double x)
// {
//     return x > 0 ? x : -x;
// }

// int closeEnough(double a, double b)
// {
//     double diff = a - b;
//     return abs_d(diff) < 1 ? 1 : 0;
// }

static float max(float a, float b)
{
    return a > b ? a : b;
}

static float min(float a, float b)
{
    return a < b ? a : b;
}

static float minmax(float n, float n_min, float n_max)
{
    return min(max(n, n_min), n_max);
}

// v [deg / s]
// x [deg]
double calcAcc(double v, double angle, double angleIntegral)
{
    //printf("Calculate acc. x: %.2f, v: %.2f \n", x, v);
    // double a = 0;
    // const double expectedA = abs_d(v * v / 2 / x);
    //const double aMax = 90; // [deg / s^2]

    // // printf("Expected a: %.2f \n", expectedA);

    // if (closeEnough(x, 0) && closeEnough(v, 0))
    // {
    //     // printf("1");
    //     a = 0;
    // }
    // else
    // {
    //     if (expectedA < aMax)
    //     {
    //         if (expectedA > 0.3 * aMax)
    //         {
    //             a = expectedA * -sign(v);
    //         }
    //         else
    //         {
    //             a = aMax * -sign(x);
    //         }
    //     }
    //     else
    //     {
    //         a = aMax * -sign(v);
    //     }
    // }

    const double P = -0.001;
    const double I = P * 0.3;
    const double D = -0.003;

    //const double boundedIntegral = minmax(angleIntegral, -5, 5);

    const double PID = P * angle + I * angleIntegral + D * v;

    return PID > 1 ? 1 : PID < -1 ? -1 : PID;
}