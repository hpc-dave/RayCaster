#ifndef CONVERTSTLTOFACETLIST_H_
#define CONVERTSTLTOFACETLIST_H_

#include <vector>
#include <DaRe/FileFormats/STL.h>
#include <FacetManager.h>
namespace caster{

std::vector<Facet> ConvertSTLtoFacetList(const dare::ff::STL<double>& stl){
    std::vector<Facet> facets(stl.GetFacets().size());
    for(std::size_t n = 0; n < stl.GetFacets().size(); n++){
        facets[n] = Facet(stl.GetFacets()[n].GetPoints()[0],
                          stl.GetFacets()[n].GetPoints()[1],
                           stl.GetFacets()[n].GetPoints()[2]);
    }
    return facets;
}

} // namespace caster

#endif // CONVERTSTLTOFACETLIST_H_
