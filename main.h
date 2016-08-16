//
// Created by veter on 07.06.16.
//

#ifndef CVRP_MAIN_H
#define CVRP_MAIN_H

#include "constraint_solver/routing.h"
#include "constraint_solver/routing_parameters.pb.h"
#include "matr.h"

namespace operations_research {

    class CVRPTWSolver{

    public:

      static  std::vector<std::vector<int64>> SolveCVRP (Matrix& matrix, int64 num_v);

      static  std::vector<std::vector<int64>>  SolveCVRPTW (Matrix& matrix, int64 num_v, std::string meta_euristic, int64 time);

        static  std::vector<std::vector<int64>>  SolveCVRPTW  (Matrix& matrix, int64 num_v, std::vector<std::vector<int64>> initialSolution ,  std::string meta_euristic, int64 time);

       // static  std::vector<std::vector<int64>>  SolveCVRPTW (Matrix& matrix, int64 num_v, int64 lns);

        static std::pair<int64, std::vector<std::vector<int64>>> SolveCVRPTW (Matrix& matrix, int64 num_v, FirstSolutionStrategy_Value fss, LocalSearchMetaheuristic_Value);

        static std::vector<std::vector<int64>> SolveCVRPTW_Best_Solution(Matrix& matrix, int64 num_v);

        static   std::vector<std::vector<int64>> SolveCVRPTWMD (Matrix& matrix, int64 num_v, std::string meta_euristic, int64 time);

        static std::vector<std::vector<int64>> SolveCVRPTWPnD (Matrix& matrix, int64 num_v, std::string meta_euristic, int64 time);

        static std::vector<std::vector<int64>> SolveCVRPTWBackHauls(Matrix& matrix, int64 num_v, std::string meta_euristic, int64 time);

        vector <vector<int64>> SolveCVRPTW_VP(Matrix &matrix, int64 num_v);
    };


}



#endif //CVRP_MAIN_H
