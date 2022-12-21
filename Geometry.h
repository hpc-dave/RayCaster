#ifndef GEOMETRY_H_
#define GEOMETRY_H_
#include <cmath>
#include <limits>
#include <algorithm>
#include <cfloat>
#include <cmath>
#include <DaRe/Utilities/Vector.h>
#include "Quaternion.h"

namespace caster{

using Vector3 = dare::utils::Vector<3, double>;

template<typename T>
struct conditional{
    bool is_set{false};
    T value;
    void Set(const T& t){
        value = t;
        is_set = true;
    }
};

void SolveMatrixSystem3x3(const double A[3][3], const double b[3], double x[3]) {
    /*
     * Uses Cramer's rule for solving the system directly
     * could be optimized by using column major matrices
     */
    auto GetDeterminant3x3 = [](const double* A) {
        enum {
            a11,
            a12,
            a13,
            a21,
            a22,
            a23,
            a31,
            a32,
            a33
        };
        // first row of minors
        const double MinA{A[a22] * A[a33] - A[a32] * A[a23]};
        const double MinB{A[a21] * A[a33] - A[a31] * A[a23]};
        const double MinC{A[a21] * A[a32] - A[a31] * A[a22]};
        return A[a11] * MinA - A[a12] * MinB + A[a13] * MinC;
    };

    const double detA{GetDeterminant3x3(A[0])};

    // test if matrix can be solved
    if (fabs(detA) < DBL_EPSILON) {
        std::fill(&x[0], &x[3], std::numeric_limits<double>::max());
        return;
    }

    double A_m[3][3];
    std::copy(A[0], A[0] + 9, A_m[0]);

    A_m[0][0] = b[0];
    A_m[1][0] = b[1];
    A_m[2][0] = b[2];

    x[0] = GetDeterminant3x3(A_m[0]) / detA;

    A_m[0][0] = A[0][0];
    A_m[1][0] = A[1][0];
    A_m[2][0] = A[2][0];

    A_m[0][1] = b[0];
    A_m[1][1] = b[1];
    A_m[2][1] = b[2];

    x[1] = GetDeterminant3x3(A_m[0]) / detA;

    A_m[0][1] = A[0][1];
    A_m[1][1] = A[1][1];
    A_m[2][1] = A[2][1];

    A_m[0][2] = b[0];
    A_m[1][2] = b[1];
    A_m[2][2] = b[2];

    x[2] = GetDeterminant3x3(A_m[0]) / detA;
}

struct _uvt{
    _uvt() : u(0.), v(0.), t(0.){}
    _uvt(double a, double b, double c): u(a), v(b), t(c){}
    double u, v, t;
};

_uvt GetBarycentricCoordinates(const Vector3& vA,
                               const Vector3& vB,
                               const Vector3& vC,
                               const Vector3& vO,
                               const Vector3& vD) {
    Vector3 AB(vB - vA);
    Vector3 AC(vC - vA);
    Vector3 AO(vO - vA);

    double A[3][3] = {AB.x(), AC.x(), vD.x(),
                      AB.y(), AC.y(), vD.y(),
                      AB.z(), AC.z(), vD.z()};

    double x[3] = {DBL_MAX, DBL_MAX, DBL_MAX};

    SolveMatrixSystem3x3(A, AO.data(), x);

    _uvt uvt;
    uvt.u = x[0];
    uvt.v = x[1];
    uvt.t = -x[2];

    return uvt;
}

Vector3 Rotate(const Vector3& p, const Vector3& axis, double theta) {
    Quaternion q_p(0., p.x(), p.y(), p.z());

    const double f{sin(theta / 2.)};
    Quaternion q_rot(cos(theta/2.), axis.x() * f, axis.y() * f, axis.z() * f);

    Quaternion q_inv(q_rot.GetInverse());

    Quaternion q_res = q_inv * q_p * q_rot;

    Vector3 res(q_res.x(), q_res.y(), q_res.z());
    return res;
}

} // namespace caster

#endif  // GEOMETRY_H_
