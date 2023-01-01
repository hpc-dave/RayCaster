#ifndef FACETMANAGER_H_
#define FACETMANAGER_H_

#include <vector>
#include <DaRe/Utilities/Vector.h>
#include "Geometry.h"
namespace caster{

struct _ray_facet_intersect{
    _uvt uvt ={DBL_MAX, DBL_MAX, DBL_MAX};
    bool is_on_facet{false};
};
class Facet {
public:
    const double tolerance{DBL_EPSILON};

    Facet(){}
    Facet(const Vector3& A, const Vector3& B, const Vector3& C)
        : Facet(A, B, C, ComputeNormal(A, B, C)){}
    Facet(const Vector3& A, const Vector3& B, const Vector3& C, const Vector3& N)
        : points{A, B, C}, normal(N) {}
    Facet(const Facet& other)
        : points{other.GetA(), other.GetB(), other.GetC()}, normal(other.normal){
    }

    Facet& operator=(const Facet& other){
        if(this == &other)
            return *this;

        for(uint8_t i = 0; i < 3; i++)
            points[i] = other.points[i];

        normal = other.normal;

        return *this;
    }

    _ray_facet_intersect ComputeIntersect(const Vector3& source, const Vector3& traject) const {
        _ray_facet_intersect isct;
        isct.uvt = GetBarycentricCoordinates(GetA(), GetB(), GetC(), source, traject);

        isct.is_on_facet  = isct.uvt.t > tolerance && isct.uvt.t <= 1.;
        isct.is_on_facet &= isct.uvt.u > tolerance && isct.uvt.u <= 1.;
        isct.is_on_facet &= isct.uvt.v > tolerance && isct.uvt.v <= 1.;
        isct.is_on_facet &= isct.uvt.u + isct.uvt.v <= 1.;
        return isct;
    }

    const Vector3& GetNormal() const { return normal; }
    const Vector3* GetPoints() const { return points; }
    const Vector3& GetA() const { return points[0]; }
    const Vector3& GetB() const { return points[1]; }
    const Vector3& GetC() const { return points[2]; }

private :
    Vector3 ComputeNormal(const Vector3& A, const Vector3& B, const Vector3& C) const {
        Vector3 norm = (B - A).cross(C - A);
        return norm / norm.length();
    }
    Vector3 points[3];
    Vector3 normal;
};

template<typename FPROP = double>
class FacetManager{
public:
    FacetManager() {}
    FacetManager(const std::vector<Facet>& _facets, const FPROP& prop)
        : FacetManager(_facets, std::vector<FPROP>(_facets.size(), prop)){
            default_property = prop;
            use_default = true;
    }

    FacetManager(const std::vector<Facet>& _facets, const std::vector<FPROP>& props)
        : facets(_facets), properties(props), use_default(false){}

    void SetFacet(std::size_t n, const Facet& f) {
        if(use_default)
            SetFacet(n, f, default_property);
        else
            facets[n] = f;
    }
    void SetFacet(std::size_t n, const Facet& f, const FPROP& prop) {
        facets[n] = f;
        properties[n] = prop;
    }

    Facet* AccessFacet(std::size_t n) { return &facets[n]; }
    const Facet& GetFacet(std::size_t n) const { return facets[n]; }
    const FPROP& GetProperty(std::size_t n) const {return properties[n]; }

    const std::size_t GetNumFacets() const {return facets.size(); }

    void Reserve(std::size_t n) {
        facets.reserve(n);
        properties.reserve(n);
    };

private:
    std::vector<Facet> facets;
    std::vector<FPROP> properties;
    FPROP default_property;
    bool use_default;
};

} // namespace caster

#endif // FACETMANAGER_H_
