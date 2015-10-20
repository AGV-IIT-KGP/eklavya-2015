#ifndef __TP_RRT_VELOCITY2D_HPP__
#define __TP_RRT_VELOCITY2D_HPP__

class Velocity2D {
public:
    Velocity2D() {
        x_ = 0.0;
        y_ = 0.0;
        phi_ = 0.0;
    }
    Velocity2D(const double& xp, const double& yp, const double& phip);
    const double& x() const;
    const double& y() const;
    const double& phi() const;
protected:
    double x_, y_;
    double phi_;
};

#endif //ifndef __TP_RRT_VELOCITY2D_HPP__