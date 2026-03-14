#pragma once

class Servo {
public:
    void attach(int pin) { _attached = true; _pin = pin; }
    void detach()        { _attached = false; }
    void write(int angle){ _angle = angle; }
    int  read() const    { return _angle; }
    bool attached() const{ return _attached; }
private:
    int  _pin      = 0;
    int  _angle    = 90;
    bool _attached = false;
};
