#ifndef KALMAN1D_HPP
#define KALMAN1D_HPP

class Kalman1D {
public:
    Kalman1D();
    void predict();
    void update(float measurement);
    float getEstimate() const;

private:
    float _x; // 状态估计
    float _P; // 估计误差协方差
};

#endif // KALMAN1D_HPP
