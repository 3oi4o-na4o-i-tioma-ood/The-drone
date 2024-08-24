#include "stabilization.h"
#include <stdio.h>

int sign(double val)
{
    return (0.0 < val) - (val < 0.0);
}

double abs_d(double x)
{
    return x > 0 ? x : -x;
}

int closeEnough(double a, double b)
{
    double diff = a - b;
    return abs_d(diff) < 1 ? 1 : 0;
}

// v [deg / s]
// x [deg]
double calcAcc(double v, double x)
{
    //printf("Calculate acc. x: %.2f, v: %.2f \n", x, v);
    // double a = 0;
    // const double expectedA = abs_d(v * v / 2 / x);
    const double aMax = 90; // [deg / s^2]

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

    const double P = 0;//-7;
    const double D = -30;

    const double PD = (P * x + D * v) / aMax;

    return PD > 1 ? 1 : PD < -1 ? -1 : PD;
}