#ifndef PLANNING_UTIL_H
#define PLANNING_UTIL_H

#include <vector>

namespace planning_util {

// pathlocal representation
struct pathstruct{
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> psi;
    std::vector<double> kappa_c;
    std::vector<double> s;
};

}; // END NAMESPACE

#endif /* PLANNING_UTIL_H */
