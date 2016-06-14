#include <iostream>
#include "main.h"


using namespace std;


namespace operations_research {


    std::vector<std::vector<int64>>  CVRPTWSolver::SolveCVRP (Matrix& matrix, int64 num_v) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode),0 ,NewPermanentCallback(&matrix, &Matrix::getVehicleCapicity), true, "Capacity" );


        RoutingSearchParameters parameters  = routing.DefaultSearchParameters();


        parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
        parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);
        parameters.set_lns_time_limit_ms(10 * 1000);
        parameters.set_time_limit_ms(30 * 1000);

        // parameters.set_time_limit_ms(240 * 1000);
        parameters.set_guided_local_search_lambda_coefficient(0.1);


        std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;

        // parameters.local_search_metaheuristic();


        // Setting depot
        //CHECK_GT(FLAGS_depot, 0) << " Because we use the" << " TSPLIB convention, the depot id must be > 0";
        //RoutingModel::NodeIndex depot(FLAGS_depot -1);
        //routing.SetDepot(depot);

        routing.CloseModelWithParameters(parameters);

        // Forbidding empty routes (optional)
//         for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
//           IntVar* const start_var = routing.NextVar(routing.Start(vehicle_nbr));
//           for (int64 node_index = routing.Size(); node_index < routing.Size() + routing.vehicles(); ++node_index) {
//             start_var->RemoveValue(node_index);
//           }
//         }

        // SOLVE          we can use different solving strategies
        const Assignment* solution = routing.SolveWithParameters(parameters);

        //const Assignment* solution = routing.Solve(routing.solver()->MakeAssignment());

        // INSPECT SOLUTION
        if (solution != NULL) {

            // Solution cost.
            std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
            // Inspect solution.
            std::string route;
            for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                route = "";
                result.clear();
                for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                    route = StrCat(route, StrCat(routing.IndexToNode(node).value()  , " -> "));
                    result.push_back(routing.IndexToNode(node).value());
                }
                route = StrCat(route,  routing.IndexToNode(routing.End(vehicle_nbr)).value()  );
                std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                cvrp_result.push_back(result);
            }


            return cvrp_result;

        } else {
            LG << "No solution found.";
        }

    }  //  void  VRPSolver (CVRPData & data)


    std::vector<std::vector<int64>>  CVRPTWSolver::SolveCVRPTW(Matrix &matrix, int64 num_v) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                NewPermanentCallback(&matrix, &Matrix::getVehicleCapicity), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), true, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();



        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); ++i) {
            int64 index = routing.NodeToIndex(i);
            IntVar* const cumul_var = routing.CumulVar(index, "time");
            cumul_var->SetMin(matrix.getTimeWindow(i.value()).first);
            cumul_var->SetMax(matrix.getTimeWindow(i.value()).second);
        }




        parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
        parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);
        parameters.set_lns_time_limit_ms(10 * 1000);
        parameters.set_time_limit_ms(30 * 1000);
        parameters.set_guided_local_search_lambda_coefficient(0.1);

        std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;


        routing.CloseModelWithParameters(parameters);

        // Forbidding empty routes (optional)
//         for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
//           IntVar* const start_var = routing.NextVar(routing.Start(vehicle_nbr));
//           for (int64 node_index = routing.Size(); node_index < routing.Size() + routing.vehicles(); ++node_index) {
//             start_var->RemoveValue(node_index);
//           }
//         }

        // SOLVE          we can use different solving strategies
        const Assignment *solution = routing.SolveWithParameters(parameters);

        //const Assignment* solution = routing.Solve(routing.solver()->MakeAssignment());

        // INSPECT SOLUTION
        if (solution != NULL) {

            // Solution cost.
            std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
            // Inspect solution.
            std::string route;
            for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                route = "";
                result.clear();
                for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                    route = StrCat(route, StrCat(routing.IndexToNode(node).value(), " -> "));
                    result.push_back(routing.IndexToNode(node).value());
                }
                route = StrCat(route, routing.IndexToNode(routing.End(vehicle_nbr)).value());
                std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                cvrp_result.push_back(result);
            }


            return cvrp_result;

        } else {
            LG << "No solution found.";
        }



        //  void  VRPSolver (CVRPData & data)
    }

    std::vector<std::vector<int64>> CVRPTWSolver::SolveCVRPTWMD(Matrix &matrix, int64 num_v,  std::vector<std::pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex >> &depots){

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        int size = matrix.getSize();

        RoutingModel routing(size, num_v, depots);


       // routing.SetStartEnd()

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                NewPermanentCallback(&matrix, &Matrix::getVehicleCapicity), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), true, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();



        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); ++i) {
            int64 index = routing.NodeToIndex(i);
            IntVar* const cumul_var = routing.CumulVar(index, "time");
            cumul_var->SetMin(matrix.getTimeWindow(i.value()).first);
            cumul_var->SetMax(matrix.getTimeWindow(i.value()).second);
        }




        parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
        parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);
        parameters.set_lns_time_limit_ms(60 * 1000);
        parameters.set_time_limit_ms(120 * 1000);
        parameters.set_guided_local_search_lambda_coefficient(0.1);

        std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;


        routing.CloseModelWithParameters(parameters);

        // Forbidding empty routes (optional)
//         for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
//           IntVar* const start_var = routing.NextVar(routing.Start(vehicle_nbr));
//           for (int64 node_index = routing.Size(); node_index < routing.Size() + routing.vehicles(); ++node_index) {
//             start_var->RemoveValue(node_index);
//           }
//         }

        // SOLVE          we can use different solving strategies
        const Assignment *solution = routing.SolveWithParameters(parameters);

        //const Assignment* solution = routing.Solve(routing.solver()->MakeAssignment());

        // INSPECT SOLUTION
        if (solution != NULL) {

            // Solution cost.
            std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
            // Inspect solution.
            std::string route;
            for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                route = "";
                result.clear();
                for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                    route = StrCat(route, StrCat(routing.IndexToNode(node).value(), " -> "));
                    result.push_back(routing.IndexToNode(node).value());
                }
                route = StrCat(route, routing.IndexToNode(routing.End(vehicle_nbr)).value());
                std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                cvrp_result.push_back(result);
            }


            return cvrp_result;

        } else {
            LG << "No solution found.";
        }



    }






}



bool diagonalTest(std::vector <std::vector<int64>> vec){

    for (int i = 0; i < vec.size(); i ++){
        for (int j = 0; j < vec.size(); j ++){
            if (i == j && vec[i][j] != 0){
                return false;
            }
            else continue;
        }
    }return true;
}

//int main(){
//
//    std::vector <std::vector<int64>> vec = {
//            {0, 19, 17, 34, 7, 20, 10, 17, 28, 15, 23, 29, 23, 29, 21, 20, 9, 16, 21, 13, 12},
//            {19, 0 ,10, 41, 26, 3, 27, 25, 15, 17, 17, 14, 18, 48, 17, 6, 21, 14, 17, 13, 31},
//            {17, 10, 0, 47, 23, 13, 26, 15, 25, 22, 26, 24, 27, 44, 7, 5, 23, 21, 25, 18, 29},
//            {34, 41, 47, 0, 36, 39, 25, 51, 36, 24, 27, 38, 25, 44, 54, 45, 25, 28, 26, 28, 27},
//            {7, 26, 23, 36, 0, 27, 11, 17, 35, 22, 30, 36, 30, 22, 25, 26, 14, 23, 28, 20, 10},
//            {20, 3, 13, 39, 27, 0, 26, 27, 12, 15, 14, 11, 15, 49, 20, 9, 20, 11, 14, 11, 30},
//            {10, 27 ,26, 25, 11, 26, 0, 26, 31, 14, 23 ,32, 22, 25, 31, 28, 6, 17, 21, 15, 4},
//            {17, 25, 15, 51, 17, 27, 26, 0, 39, 31, 38, 38 ,38, 34, 13, 20, 26, 31, 36, 28, 27},
//            {28, 15, 25, 36, 35, 12, 31, 39, 0, 17, 9, 2, 11, 56, 32, 21, 24, 13, 11, 15 ,35},
//            {15, 17, 22, 24, 22 ,15, 14, 31, 17, 0, 9, 18, 8, 39, 29, 21, 8, 4, 7 ,4, 18},
//            { 23, 17, 26, 27, 30, 14, 23, 38, 9, 9 ,0, 11, 2, 48, 33, 23, 17, 7, 2, 10, 27},
//            { 29, 14, 24, 38, 36, 11, 32 ,38,2 ,18, 11 ,0, 13, 57 ,31, 20 ,25, 14 ,13 ,17, 36},
//            {23, 18, 27, 25 ,30, 15, 22, 38, 11, 8, 2 ,13 ,0, 47, 34, 24, 16, 7, 2, 10, 26},
//            {29, 48, 44 ,44, 22, 49 ,25, 34, 56, 39, 48, 57, 47, 0 ,46, 48, 31, 42 ,46 ,40 ,21},
//            {21, 17, 7, 54, 25 ,20 ,31, 13 ,32 ,29, 33 ,31 ,34, 46 ,0, 11, 29, 28 ,32, 25, 33},
//            { 20, 6, 5, 45 ,26, 9, 28 ,20 ,21, 21, 23 ,20 ,24 ,48, 11 ,0 ,23 ,19, 22, 17 ,32},
//            { 9, 21 ,23, 25 ,14 ,20, 6, 26, 24, 8, 17, 25 ,16, 31, 29 ,23, 0, 11, 15, 9, 10},
//            {16, 14, 21, 28, 23 ,11, 17 ,31, 13 ,4, 7, 14 ,7, 42, 28 ,19, 11 ,0, 5, 3, 21},
//            { 21, 17, 25, 26, 28 ,14 ,21, 36 ,11, 7 ,2 ,13, 2, 46, 32, 22, 15 ,5, 0 ,8, 25},
//            {13, 13, 18, 28, 20, 11 ,15 ,28, 15 ,4 ,10 ,17, 10 ,40 ,25, 17, 9 ,3 ,8, 0 ,19},
//            {12, 31, 29, 27, 10, 30, 4, 27, 35, 18 ,27, 36, 26, 21, 33, 32 ,10 ,21, 25, 19 ,0}
//
//    };
//    std::vector<std::pair<int64, int64>> timeWindows = {
//            {0, 408},
//            { 62, 68 },
//            { 181, 205 },
//            { 306, 324 },
//            { 214, 217 },
//            { 51, 61 },
//            { 102, 129 },
//            { 175, 186 },
//            { 250, 263 },
//            { 3, 23 },
//            { 21, 49 },
//            { 79, 90 },
//            { 78, 96 },
//            { 140, 154 },
//            { 354, 386 },
//            { 42, 63 },
//            { 2, 13 },
//            { 4, 42 },
//            { 20, 33 },
//            { 9, 21 },
//            { 275, 300 }
//
//    };
//
//    std::vector<int64> serviceTime = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//
//    std::vector<int64> demands = {1, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//    std::vector<int64> capacities = {22, 23, 23, 23};
//    std::vector<std::pair<operations_research::RoutingModel::NodeIndex, operations_research::RoutingModel::NodeIndex>> depots(4);
//
//    depots[0] = std::make_pair(0,0);
//    depots[1] = std::make_pair(0, 14);
//    depots[2] = std::make_pair(4, 10);
//    depots[3] = std::make_pair(2, 111);
//
//
//    operations_research::Matrix matrix(vec, demands, capacities, timeWindows, serviceTime);
//
//
//        operations_research::CVRPTWSolver::SolveCVRPTWMD(matrix, capacities.size(), depots);
//
//
//
//    return 0;
//}
