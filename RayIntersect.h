#ifndef RAYINTERSECT_H_
#define RAYINTERSECT_H_

#include "Geometry.h"
namespace caster{
class RayIntersect{
public:
    RayIntersect() : point(0., 0., 0.), trajectory(0., 0., 0.) {}
    RayIntersect(const Vector3& p, const Vector3& t) : point(p), trajectory(t) {}
    RayIntersect(const RayIntersect& r) : RayIntersect(r.point, r.trajectory) {}
    ~RayIntersect() {}

    Vector3& GetPoint() { return point; }
    Vector3& GetTrajectory() { return trajectory; }

    const Vector3& GetPoint() const { return point; }
    const Vector3& GetTrajectory() const { return trajectory; }

    friend std::ostream& operator<<(std::ostream& os, const RayIntersect& r) {
        os << r.GetPoint() << "  " << r.GetTrajectory();
        return os;
    }

private:
    Vector3 point;
    Vector3 trajectory;
};
} // namespace caster

#endif // RAYINTERSECT_H_
