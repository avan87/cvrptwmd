//
// Created by veter on 09.06.16.
//

#ifndef CVRPTW_UTILS_H
#define CVRPTW_UTILS_H

#include "constraint_solver/routing.h"

class Utils {

public:
    std::vector<int64> castList(const std::vector<int64_t> & demands);

    std::vector<std::vector<int64> > cast2dList (const std::vector<std::vector<int64_t> > & vec);

    std::vector<std::pair<int64, int64>> castToListOfPairs(const std::vector<std::vector<int64_t> > & timeWindows);

};


#endif //CVRPTW_UTILS_H
