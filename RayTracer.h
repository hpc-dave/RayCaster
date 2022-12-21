#ifndef RAYTRACER_H_
#define RAYTRACER_H_

#include <vector>
#include <Ray.h>
#include <FacetManager.h>
namespace caster{
class RayTracer {
public:
    RayTracer() : facet_manager(nullptr){}
    explicit RayTracer(const FacetManager<>& f_man) : facet_manager(&f_man){}

    void SetSource(const std::vector<RayIntersect>& sources) {
        rays.resize(sources.size());
        for (std::size_t i = 0; i < sources.size(); i++)
            rays[i].SetSource(sources[i]);
    }

    void AdvanceRays() {
        const double tol{DBL_EPSILON};
/*         auto ObjIntersect = [](RayIntersect& ray) {
            const double tol{DBL_EPSILON};
            // Triangle
            Vector3 A(1., 0., 0.);
            Vector3 B(0., 1., 0.);
            Vector3 C(1., 0., 1.);
            const double refr_ind{2.};

            Facet facet(A, B, C);
            _ray_facet_intersect isct = facet.ComputeIntersect(ray.GetPoint(), ray.GetTrajectory());

            if (isct.is_on_facet) {
                Vector3 p(ray.GetPoint() + ray.GetTrajectory() * isct.uvt.t);
                Vector3 t(ray.GetTrajectory());
                Vector3 normal(facet.GetNormal());
                double dotp = normal.dot(t);
                normal *= (dotp < 0) - (dotp > 0);
                Vector3 rot_axis(t.cross(normal));

                if ((abs(rot_axis.x()) + abs(rot_axis.y()) + abs(rot_axis.z())) < tol)
                    return RayIntersect(p, t);

                double theta_1 = acos(normal.dot(t) / (normal.length() * t.length())) - 0.5 * M_PI;
                double theta_2 = asin(sin(theta_1) / refr_ind);
                double theta_tot = theta_1 - theta_2;
                t = Rotate(t, rot_axis, theta_tot);
                return RayIntersect(p, t);
            } else {
                return RayIntersect(ray.GetPoint() + ray.GetTrajectory(), ray.GetTrajectory());
            }
        }; */

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
                    if(itsc_temp.is_on_facet && (itsc_temp.uvt.t < itsc_close.uvt.t)){
                        itsc_close = itsc_temp;
                        n_close = nf;
                    }
                }
                if(itsc_close.is_on_facet){
                    Vector3 p(ray.GetPoint() + ray.GetTrajectory() * itsc_close.uvt.t);
                    Vector3 t(ray.GetTrajectory());
                    Vector3 normal(facet_manager->GetFacet(n_close).GetNormal());
                    double dotp = normal.dot(t);
                    normal *= (dotp < 0) - (dotp > 0);
                    Vector3 rot_axis(t.cross(normal));

                    if ((abs(rot_axis.x()) + abs(rot_axis.y()) + abs(rot_axis.z())) < tol) {
                        ray = RayIntersect(p, t);
                    } else {
                        double theta_1 = acos(normal.dot(t) / (normal.length() * t.length())) - 0.5 * M_PI;
                        double theta_2 = asin(sin(theta_1) / facet_manager->GetProperty(n_close));
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

    const std::vector<Ray>& GetRays() const { return rays; }

private:
    std::vector<Ray> rays;
    const FacetManager<>* facet_manager;
    const int max_iterations = 100;
};

} // namespace caster
#endif // RAYTRACER_H_
