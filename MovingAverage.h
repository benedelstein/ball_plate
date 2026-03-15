#ifndef MOVING_AVERAGE_H
#define MOVING_AVERAGE_H

class MovingAverage {
public:
    MovingAverage() : size_(0), sum_(0), readings_(nullptr) {}

    void init(int windowSize) {
        size_ = windowSize;
        sum_ = 0;
        readings_ = new float[size_];
        for (int i = 0; i < size_; i++) {
            readings_[i] = 0;
        }
    }

    ~MovingAverage() {
        delete[] readings_;
    }

    float update(float newValue) {
        sum_ -= readings_[0];

        // Shift readings left
        for (int i = 0; i < size_ - 1; i++) {
            readings_[i] = readings_[i + 1];
        }

        readings_[size_ - 1] = newValue;
        sum_ += newValue;

        return sum_ / size_;
    }

    float getValue() const {
        return (size_ > 0) ? sum_ / size_ : 0;
    }

    void reset() {
        sum_ = 0;
        for (int i = 0; i < size_; i++) {
            readings_[i] = 0;
        }
    }

private:
    int size_;
    float sum_;
    float* readings_;

    // Prevent copying (no copy constructor/assignment on Arduino)
    MovingAverage(const MovingAverage&);
    MovingAverage& operator=(const MovingAverage&);
};

#endif
