#include "DaRe/Utilities/Vector.h"

#ifndef QUATERNION_H_
#define QUATERNION_H_
namespace caster{
class Quaternion : public dare::utils::Vector<4, double>{
public:
    using Vector = dare::utils::Vector<4, double>;
    Quaternion() : Vector(0., 0., 0., 0.){}
    Quaternion(double a, double b, double c, double d) : Vector(a, b, c, d){}

    Quaternion operator*(const Quaternion& q) const{
        double pr = q.r() * r() - q.x() * x() - q.y() * y() - q.z() * z();
        double px = q.x() * r() + q.r() * x() - q.z() * y() + q.y() * z();
        double py = q.y() * r() + q.z() * x() + q.r() * y() - q.x() * z();
        double pz = q.z() * r() - q.y() * x() + q.x() * y() + q.r() * z();
        return Quaternion(pr, px, py, pz);
    }

    Quaternion GetInverse() const{
            return Quaternion(this->data()[0], -this->data()[1], -this->data()[2], -this->data()[3]);
    }

    double r() const { return this->data()[0]; }
    double x() const { return this->data()[1]; }
    double y() const { return this->data()[2]; }
    double z() const { return this->data()[3]; }
    double& r() { return this->data()[0]; }
    double& x() { return this->data()[1]; }
    double& y() { return this->data()[2]; }
    double& z() { return this->data()[3]; }
};
} // namespace caster

#endif // QUATERNION_H_
