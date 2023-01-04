#include <iostream>

#include <DaRe/FileFormats/STL.h>

#include <Geometry.h>
#include <Quaternion.h>
#include <RayTracer.h>
#include <FacetManager.h>
#include <ConvertSTLtoFacetList.h>


int main(int argc, char* argv[]){
    using caster::RayIntersect;
    using caster::Vector3;

    dare::ff::STL<double> stl("test.stl");

    caster::FacetManager fman(caster::ConvertSTLtoFacetList(stl), 0.8);
    caster::RayTracer tracer(fman);
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

    tracer.WriteToFile("output.xml");

    std::cout << "Hello World!\n";
}
