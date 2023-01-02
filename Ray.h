#ifndef RAY_H_
#define RAY_H_

#include <iostream>
#include <vector>
#include "RayIntersect.h"
namespace caster{
class Ray{
public:
    Ray() {}
    ~Ray() {}

    void SetSource(const RayIntersect& s) {
        intersects.clear();
        AppendIntersect(s);
    }

    void SetSource(const Vector3& point, const Vector3& trajectory) {
        SetSource({point, trajectory});
    }

    void AppendIntersect(const RayIntersect& s) {
        intersects.push_back(s);
    }

    void AppendIntersect(const Vector3& point, const Vector3& trajectory) {
        AppendIntersect({point, trajectory});
    }

    const RayIntersect& GetIntersect() const {
#ifndef N_DEBUG
        if (intersects.size() == 0) {
            std::cerr << "number of intersects is 0, segmentation fault incoming" << std::endl;
        }
#endif
        return *(--intersects.end());
    }

    const std::vector<RayIntersect>& GetAllIntersects() const {
        return intersects;
    }

    std::size_t GetNumIntersects() const{
        return intersects.size();
    }

private:
    std::vector<RayIntersect> intersects;
};
} // namespace caster
#endif  // RAY_H_
