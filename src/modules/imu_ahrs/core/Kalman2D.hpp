#ifndef KALMAN2D_HPP
#define KALMAN2D_HPP

class Kalman2D {
public:
    Kalman2D();
    void predict();
    void update(float measurement);
    float getEstimate() const;

private:
    float _x; // 状态估计
    float _P; // 估计误差协方差
};

#endif // KALMAN2D_HPP
