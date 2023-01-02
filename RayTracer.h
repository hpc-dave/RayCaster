#ifndef RAYTRACER_H_
#define RAYTRACER_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <Ray.h>
#include <FacetManager.h>
namespace caster{
class RayTracer {
public:
    RayTracer();
    explicit RayTracer(const FacetManager<>& f_man);

    void SetSource(const std::vector<RayIntersect>& sources);

    void AdvanceRays();

    void WriteToFile(const std::string& file_name);

    const std::vector<Ray>& GetRays() const { return rays; }

private:
    std::vector<Ray> rays;
    const FacetManager<>* facet_manager;
    const int max_iterations = 100;
};

} // namespace caster
#endif // RAYTRACER_H_
