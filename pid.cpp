#include "pid.h"

pid::pid()
{
    x.setZero(2);
    u.setZero(2);
    A.setZero(2,2);
    B.setZero(2);
    kusi = 0;
    nature_freq = 0;
    input = 0;
    dt = 0.01;
}
void pid::set(double a){
    kusi = a;
    A << 0, 1, -nature_freq*nature_freq, -2*kusi*nature_freq;
    B << 0, nature_freq*nature_freq;
}
double pid::run(){
    x += (A * x + B * input) * dt;
    return x(0);
}
void pid::reset(){
   x.setZero(2);
}
