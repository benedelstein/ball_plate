#pragma once

struct TSPoint {
    int x, y, z;
    TSPoint()              : x(0), y(0), z(0) {}
    TSPoint(int x, int y, int z) : x(x), y(y), z(z) {}
};

class TouchScreen {
public:
    TouchScreen(int xp, int yp, int xm, int ym, int rx) {}
    TSPoint getPoint() { return TSPoint(0, 0, 0); }
};
