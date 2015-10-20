#include <iostream>

#include <tp_rrt_planner/velocity2d.hpp>

Velocity2D::Velocity2D(const double& xp, const double& yp, const double& phip) : x_(xp), y_(yp), phi_(phip) {
    //std::cout<<"Cmd_Vel x:"<< x_<<" phi : "<<phi_<<"\n";
}
const double& Velocity2D::x() const {
    return x_;
}  
const double& Velocity2D::y() const {
    return y_;
}
const double& Velocity2D::phi() const {
    return phi_;
}