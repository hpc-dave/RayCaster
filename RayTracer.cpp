
#include <cmath>
#include <string>
#include <sstream>
#include <DaRe/FileFormats/XML.h>
#include "RayTracer.h"

namespace caster{

RayTracer::RayTracer()
    : facet_manager(nullptr) {
}

RayTracer::RayTracer(const FacetManager<>& f_man)
    : facet_manager(&f_man) {
}

void RayTracer::SetSource(const std::vector<RayIntersect>& sources) {
    rays.resize(sources.size());
    for (std::size_t i = 0; i < sources.size(); i++)
        rays[i].SetSource(sources[i]);
}

void RayTracer::AdvanceRays() {
    const double tol{DBL_EPSILON};

    const int n_rays{static_cast<int>(rays.size())};
    for (int n = 0; n < n_rays; n++) {
        RayIntersect ray(rays[n].GetIntersect());  // intersect, which will be advanced stepwise

        for (int step = 0; step < max_iterations; ++step) {
            // Here we have to determine, where the ray from the specified source
            // does intersect with the object and how it's refracted
            // start of a dummy implementation
            _ray_facet_intersect itsc_close;
            std::size_t n_close{0};
            for (std::size_t nf = 0; nf < facet_manager->GetNumFacets(); ++nf) {
                _ray_facet_intersect itsc_temp
                    = facet_manager->GetFacet(nf).ComputeIntersect(ray.GetPoint(), ray.GetTrajectory());
                if (itsc_temp.is_on_facet && (itsc_temp.uvt.t < itsc_close.uvt.t)) {
                    itsc_close = itsc_temp;
                    n_close = nf;
                }
            }
            if (itsc_close.is_on_facet) {
                Vector3 p(ray.GetPoint() + ray.GetTrajectory() * itsc_close.uvt.t);
                Vector3 t(ray.GetTrajectory());
                Vector3 normal(facet_manager->GetFacet(n_close).GetNormal());
                double dotp = normal.dot(t);
                bool out_to_in{dotp < 0};
                normal *= out_to_in - !out_to_in;
                Vector3 rot_axis(t.cross(normal));

                if ((fabs(rot_axis.x()) + fabs(rot_axis.y()) + fabs(rot_axis.z())) < tol) {
                    ray = RayIntersect(p, t);
                } else {
                    double theta_1 = std::acos(normal.dot(t) / (normal.length() * t.length())) - 0.5 * M_PI;
                    double refr_ind = facet_manager->GetProperty(n_close);
                    if (!out_to_in) {
                        refr_ind = 1. / refr_ind;
                    }
                    double theta_manip = std::sin(theta_1) / refr_ind;
                    if(fabs(theta_manip) >= 1.){
                        std::cerr << "manipulated theta leads to total mirroring, functionality not implemented\n";
                    }
                    double theta_2 = std::asin(theta_manip);
                    double theta_tot = theta_1 - theta_2;
                    t = Rotate(t, rot_axis, theta_tot);
                    ray = RayIntersect(p, t);
                }
            } else {
                ray = RayIntersect(ray.GetPoint() + ray.GetTrajectory(), ray.GetTrajectory());
            }
            // itsc = ObjIntersect(itsc);
            rays[n].AppendIntersect(ray);

            // end of the dummy implementation
            bool stop = ray.GetPoint().x() < 0. || ray.GetPoint().x() > 1.;
            stop |= ray.GetPoint().y() < 0. || ray.GetPoint().y() > 1.;
            stop |= ray.GetPoint().z() < 0. || ray.GetPoint().z() > 1.;

            if (stop)
                break;
        }
    }
}

void RayTracer::WriteToFile(const std::string& file_name){

    dare::ff::XMLNode main_node("Rays",
                                { dare::ff::_XMLAttribute("encoding", "ASCII"),
                                  dare::ff::_XMLAttribute("type", "custom")    });

    for (std::size_t n = 0; n < rays.size(); n++) {
        dare::ff::XMLNode ray_node("ray");
        ray_node << dare::ff::XMLNode("ID", std::to_string(n));
        ray_node << dare::ff::XMLNode("num_intersects", std::to_string(rays[n].GetNumIntersects()));
        std::ostringstream ray_data;
        for(const auto& itsc : rays[n].GetAllIntersects()){
            ray_data << itsc.GetPoint() << ',' << itsc.GetTrajectory() << ';';
        }
        ray_node << dare::ff::XMLNode("data", ray_data.str());
        main_node << ray_node;
    }
    dare::ff::XML xml_file(main_node);

    xml_file.WriteToFile(file_name, std::ios::out);
}
} // namespace caster
