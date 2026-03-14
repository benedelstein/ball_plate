#pragma once
#include <cmath>
#include <cstdlib>

// Analog pin constants
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Arduino map() — using double for float compatibility
inline double map(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline unsigned long millis() { return 0; }

class _SerialClass {
public:
    void begin(long) {}
    template<typename T> _SerialClass& print(T)    { return *this; }
    template<typename T> _SerialClass& println(T)  { return *this; }
};

inline _SerialClass Serial;
