#include <iostream>

#include <Geometry.h>
#include <Quaternion.h>
#include <RayTracer.h>
int main(int argc, char* argv[]){
    using caster::RayIntersect;
    using caster::Vector3;
    caster::RayTracer tracer;
    std::vector<RayIntersect> source = {RayIntersect(Vector3(0., 0.2, 0.2), Vector3(1., 0., 0.))};
    source.push_back(RayIntersect(Vector3(1., 0.2, 0.2), Vector3(-1., 0., 0.)));
    source.push_back(RayIntersect(Vector3(1., 1., 1.), Vector3(0., -0.5, -0.5)));

    tracer.SetSource(source);

    tracer.AdvanceRays();

    std::size_t n_rays = tracer.GetRays().size();

    for (std::size_t n = 0; n < n_rays; n++) {
        std::cout << "Ray " << n << ":\n";
        for (const auto& i : tracer.GetRays()[n].GetAllIntersects()) {
            std::cout << '\t' << i << std::endl;
        }
    }

    Vector3 v1(1., 0., 0.);
    Vector3 v2(-0.5, 1., 0.);
    Vector3 v3(0.5, 0.5, 0.);
    std::cout << "v1.v2: " << v1.dot(v2) << " v1.v3: " << v1.dot(v3) << std::endl;

    // Vector v3 = Rotate(v1, v2, M_PI * 0.5);
    /*for(int i = 0; i < 9; i++)
        std::cout << Rotate(v1, v2, M_PI * (0.25 * i)) << std::endl;

    Vector A(0., 0., 0.);
    Vector B(1., 0., 0.);
    Vector C(0., 0., 1.);

    Vector O(1., 1., 1.);
    Vector D(0., 0., 1.);

    _uvt uvt = GetBarycentricCoordinates(A, B, C, O, D);
    std::cout << "u: " << uvt.u << " v: " << uvt.v << " t: " << uvt.t << std::endl;*/

    std::cout << "Hello World!\n";
}
