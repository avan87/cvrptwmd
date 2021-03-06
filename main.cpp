#include <iostream>
#include "main.h"
#include <map>                
#include <vector>
#include "limits.h"
#include <future>

using namespace std;


namespace operations_research {


    std::vector<std::vector<int64>>  CVRPTWSolver::SolveCVRP (Matrix& matrix, int64 num_v) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));



        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode),0 ,matrix.getVcaps(), true, "Capacity" );


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


    std::vector<std::vector<int64>>  CVRPTWSolver::SolveCVRPTW(Matrix &matrix, int64 num_v, std::string meta_euristic, int64 time) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector




        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                matrix.getVcaps(), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), false, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();



        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); ++i) {
            int64 index = routing.NodeToIndex(i);
            IntVar* const cumul_var = routing.CumulVar(index, "time");

            cumul_var->SetMin(matrix.getTimeWindow(i.value()).first);
            cumul_var->SetMax(matrix.getTimeWindow(i.value()).second);
        }

        for(int i = 0; i < num_v; i++){

            int64 startNode  = routing.Start(i);
            IntVar* timeStartDim = routing.CumulVar(startNode, "time");
            timeStartDim->SetMin(matrix.getVehTimeWindows().at(i).first);
            int64 endNode = routing.End(i);
            IntVar* timeEndDim = routing.CumulVar(endNode, "time");
            timeEndDim->SetMax(matrix.getVehTimeWindows().at(i).second);
        }

     //   FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION;



        parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);


        if ("CVRPTW_TABU       " == meta_euristic      ) parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_TABU_SEARCH);

        if ("CVRPTW_SA       " == meta_euristic    ) parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_SIMULATED_ANNEALING);


        else parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);




         SearchMonitor* const no_improvement_limit =  MakeNoImprovementLimit(routing.solver(),  routing.solver()->MakeIntVar(0,5000000), 1000, true );
         routing.AddSearchMonitor(no_improvement_limit);




        //set callback to custom limit


       //set time limit

        parameters.set_time_limit_ms(time);
        //parameters.set_solution_limit(1);

       //std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;




        const int64 kPenalty = 10000000;
        const RoutingModel::NodeIndex kFirstNodeAfterDepot(1);
        for (RoutingModel::NodeIndex order = kFirstNodeAfterDepot;
             order < routing.nodes(); ++order) {
            std::vector<RoutingModel::NodeIndex> orders(1, order);
            routing.AddDisjunction(orders, kPenalty);
        }


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


            //LG <<  solution->DebugString();
            // Solution cost.
            std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
            // Inspect solution.
            std::string route;
            for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                route = "";
                result.clear();
                for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                   // IntVar* load = routing.GetDimensionOrDie("Capacity").CumulVar(node);
                    //IntVar* time = routing.GetDimensionOrDie("time").CumulVar(node);
                    route = StrCat(route, StrCat(routing.IndexToNode(node).value(), " -> "));
                    result.push_back(routing.IndexToNode(node).value());
                }
                route = StrCat(route, routing.IndexToNode(routing.End(vehicle_nbr)).value());
                std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                cvrp_result.push_back(result);
            }

            std::vector<int64> unass_orders;
            for(int i = 0; i < cvrp_result.size(); ++i){

             unass_orders.insert(end(unass_orders), begin(cvrp_result.at(i)), end(cvrp_result.at(i)));
            }

            std::sort(begin(unass_orders), end(unass_orders));


            std::vector<int64>::iterator it;
            it = std::unique (begin(unass_orders), end(unass_orders));   // 10 20 30 20 10 ?  ?  ?  ?

            unass_orders.resize( std::distance(begin(unass_orders),it) );


            std::cout << "Initial number of nodes: " << matrix.getDemands().size() << " number of nodes in solution: " << unass_orders.size() << std::endl;
            std::cout << "Difference: " << matrix.getDemands().size() - unass_orders.size() << std::endl;

            return cvrp_result;

        } else {
            LG << "No solution found.";
            LG <<" Trying to solve with more time";

            return  CVRPTWSolver::SolveCVRPTW(matrix, num_v, meta_euristic, time  * 2);

           // return CVRPTWSolver::SolveCVRPTW_Best_Solution(matrix, num_v);

        }



        //  void  VRPSolver (CVRPData & data)
    }

    std::vector<std::vector<int64>> CVRPTWSolver::SolveCVRPTWMD(Matrix &matrix, int64 num_v, std::string meta_euristic, int64 time){

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        int size = matrix.getSize();

        RoutingModel routing(size, num_v, matrix.getDepots());


       // routing.SetStartEnd()

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                matrix.getVcaps(), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), false, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();



        for (int64 i = 0; i < matrix.getSize(); ++i) {
            int64 index = i;
            IntVar* const cumul_var = routing.CumulVar(index, "time");
            cumul_var->SetMin(matrix.getTimeWindow(i).first);
            cumul_var->SetMax(matrix.getTimeWindow(i).second);
        }


        for(int i = 0; i < num_v; i++){

            int64 startNode  = routing.Start(i);
            IntVar* timeStartDim = routing.CumulVar(startNode, "time");
            timeStartDim->SetMin(matrix.getVehTimeWindows().at(i).first);
            int64 endNode = routing.End(i);
            IntVar* timeEndDim = routing.CumulVar(endNode, "time");
            timeEndDim->SetMax(matrix.getVehTimeWindows().at(i).second);

        }


//        for (int64 i = 0; i < matrix.getSize(); ++i) {
//            int64 index = i;
//            IntVar* const cumul_var = routing.CumulVar(index, "time");
//            cumul_var->SetMin(matrix.getTimeWindow(i).first);
//            cumul_var->SetMax(matrix.getTimeWindow(i).second);
//        }



        parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
        parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);

        parameters.set_time_limit_ms(time);
        //parameters.set_solution_limit(1000);



        std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;

        const int64 kPenalty = 10000000;
        const RoutingModel::NodeIndex kFirstNodeAfterDepot(1);
        for (RoutingModel::NodeIndex order = kFirstNodeAfterDepot;
             order < routing.nodes(); ++order) {
            std::vector<RoutingModel::NodeIndex> orders(1, order);
            routing.AddDisjunction(orders, kPenalty);
        }


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
                result.push_back(routing.IndexToNode(routing.End(vehicle_nbr)).value());
                cvrp_result.push_back(result);
            }


            return cvrp_result;

        } else {
            LG << "No solution found.";
        }



    }


    std::vector<std::vector<int64>>  CVRPTWSolver::SolveCVRPTW_VP(Matrix &matrix, int64 num_v) {

        //cvrptw backup

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                matrix.getVcaps(), true,
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


    std::vector<std::vector<int64>> CVRPTWSolver::SolveCVRPTW_Best_Solution(Matrix &matrix, int64 num_v) {
        std::map<int64, std::vector<std::vector<int64 >>> solutions;
        std::vector<future<std::pair<int64, std::vector<std::vector<int64>>>>> futures;


        std::vector<FirstSolutionStrategy_Value > first_solution_strategies = {
                //FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_ALL_UNPERFORMED,
              //  FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_AUTOMATIC,
                FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_BEST_INSERTION,
                //FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_SAVINGS,
                //FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_FIRST_UNBOUND_MIN_VALUE,
                FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_GLOBAL_CHEAPEST_ARC,
                FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_LOCAL_CHEAPEST_ARC,
                FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_LOCAL_CHEAPEST_INSERTION,
                FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_PARALLEL_CHEAPEST_INSERTION,
                FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC,
                FirstSolutionStrategy_Value ::FirstSolutionStrategy_Value_PATH_MOST_CONSTRAINED_ARC
        };

        std::vector<LocalSearchMetaheuristic_Value > local_search_meta_euristic_values = {
                LocalSearchMetaheuristic_Value ::LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH,
               // LocalSearchMetaheuristic_Value ::LocalSearchMetaheuristic_Value_SIMULATED_ANNEALING,
                LocalSearchMetaheuristic_Value ::LocalSearchMetaheuristic_Value_TABU_SEARCH};





        for(FirstSolutionStrategy_Value fss : first_solution_strategies){

            for(LocalSearchMetaheuristic_Value lsmev : local_search_meta_euristic_values){


                auto f = std::async(std::launch::async, [&] { return CVRPTWSolver::SolveCVRPTW(matrix, num_v, fss, lsmev);});
                futures.push_back(std::move(f));
               //solutions.insert(CVRPTWSolver::SolveCVRPTW(matrix, num_v, fss, lsmev));

             }




        }

        for(int i = 0; i < futures.size(); i++){
            solutions.insert(futures.at(i).get());
        }

       if(solutions.find(0) != solutions.end() ){

           solutions.erase(0);
       }

        //for(future f: futures){ f.get();};

        return  solutions.begin()->second;



    }

    std::pair<int64, std::vector<std::vector<int64>>> CVRPTWSolver::SolveCVRPTW(Matrix &matrix, int64 num_v,
                                                                                FirstSolutionStrategy_Value firstSolutionStrategy_value,
                                                                                LocalSearchMetaheuristic_Value localSearchMetaheuristic_value) {

        std::pair<int64, std::vector<std::vector<int64>>> solution_with_path_and_obj_value;

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetDepot(RoutingModel::NodeIndex(0));

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector




        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                matrix.getVcaps(), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), false, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();



        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); ++i) {
            int64 index = routing.NodeToIndex(i);
            IntVar* const cumul_var = routing.CumulVar(index, "time");

            cumul_var->SetMin(matrix.getTimeWindow(i.value()).first);
            cumul_var->SetMax(matrix.getTimeWindow(i.value()).second);
        }

        for(int i = 0; i < num_v; i++){

            int64 startNode  = routing.Start(i);
            IntVar* timeStartDim = routing.CumulVar(startNode, "time");
            timeStartDim->SetMin(matrix.getVehTimeWindows().at(i).first);
            int64 endNode = routing.End(i);
            IntVar* timeEndDim = routing.CumulVar(endNode, "time");
            timeEndDim->SetMax(matrix.getVehTimeWindows().at(i).second);

        }





        parameters.set_first_solution_strategy(firstSolutionStrategy_value);

        parameters.set_local_search_metaheuristic(localSearchMetaheuristic_value);

        //parameters.set_lns_time_limit_ms(2 * 1000);
        parameters.set_time_limit_ms(60 * 5  * 1000);

        //parameters.set_solution_limit(500);

        std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;

        // routing.CumulVar(routing.Start(0), "time")


        const int64 kPenalty = 10000000;
        const RoutingModel::NodeIndex kFirstNodeAfterDepot(1);
        for (RoutingModel::NodeIndex order = kFirstNodeAfterDepot;
             order < routing.nodes(); ++order) {
            std::vector<RoutingModel::NodeIndex> orders(1, order);
            routing.AddDisjunction(orders, kPenalty);
        }


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
              //  std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                cvrp_result.push_back(result);
            }

            LG << "First Solution Strategy: " << firstSolutionStrategy_value;
            LG << "Meta Euristic: " << localSearchMetaheuristic_value;
            solution_with_path_and_obj_value.first = solution->ObjectiveValue();
            solution_with_path_and_obj_value.second = cvrp_result;
            LG << "number of disjunctions: " <<  routing.GetNumberOfDisjunctions();

            return solution_with_path_and_obj_value;

        } else {
            LG << "No solution found.";
        }



    }

    vector<vector<int64>> CVRPTWSolver::SolveCVRPTWPnD(Matrix &matrix, int64 num_v, string meta_euristic, int64 time) {


        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector



        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                matrix.getVcaps(), true,
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

        for(int i = 0; i < num_v; i++){

            int64 startNode  = routing.Start(i);
            IntVar* timeStartDim = routing.CumulVar(startNode, "time");
            timeStartDim->SetMin(matrix.getVehTimeWindows().at(i).first);
            int64 endNode = routing.End(i);
            IntVar* timeEndDim = routing.CumulVar(endNode, "time");
            timeEndDim->SetMax(matrix.getVehTimeWindows().at(i).second);

        }



        const RoutingDimension &time_dimension = routing.GetDimensionOrDie("time");
        Solver *const solver = routing.solver();

        //setting PnD constraint
        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); i++) {

            const int64 index = routing.NodeToIndex(i);
            if (matrix.getPickups()[i.value()] == 0) {
                if (matrix.getDeliveries()[i.value()] == 0) {
                    routing.SetDepot(i);
                } else {

                    const int64 delivery_index = matrix.getDeliveries()[i.value()];
                    solver->AddConstraint(solver->MakeEquality(
                            routing.VehicleVar(index), routing.VehicleVar(delivery_index)));
                    solver->AddConstraint(
                            solver->MakeLessOrEqual(time_dimension.CumulVar(index),
                                                    time_dimension.CumulVar(delivery_index)));
                    routing.AddPickupAndDelivery(i, routing.IndexToNode(matrix.getDeliveries()[i.value()]));

                }
            }
        }

        parameters.set_first_solution_strategy(FirstSolutionStrategy_Value_PATH_CHEAPEST_ARC);
        parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);

        SearchMonitor* const no_improvement_limit =  MakeNoImprovementLimit(routing.solver(),  routing.solver()->MakeIntVar(0,5000000), 1000, true );
        routing.AddSearchMonitor(no_improvement_limit);

        parameters.set_time_limit_ms(time);



        const int64 kPenalty = 10000000;
        const RoutingModel::NodeIndex kFirstNodeAfterDepot(1);
        for (RoutingModel::NodeIndex order = kFirstNodeAfterDepot;
             order < routing.nodes(); ++order) {
            std::vector<RoutingModel::NodeIndex> orders(1, order);
            routing.AddDisjunction(orders, kPenalty);
        }


        const Assignment *solution = routing.SolveWithParameters(parameters);

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

    std::vector<std::vector<int64>> CVRPTWSolver::SolveCVRPTWBackHauls(Matrix &matrix, int64 num_v,
                                                                       std::string meta_euristic, int64 time) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector




        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                matrix.getVcaps(), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), false, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();



        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); ++i) {
            int64 index = routing.NodeToIndex(i);
            IntVar* const cumul_var = routing.CumulVar(index, "time");

            cumul_var->SetMin(matrix.getTimeWindow(i.value()).first);
            cumul_var->SetMax(matrix.getTimeWindow(i.value()).second);
        }

        for(int i = 0; i < num_v; i++){

            int64 startNode  = routing.Start(i);
            IntVar* timeStartDim = routing.CumulVar(startNode, "time");
            timeStartDim->SetMin(matrix.getVehTimeWindows().at(i).first);
            int64 endNode = routing.End(i);
            IntVar* timeEndDim = routing.CumulVar(endNode, "time");
            timeEndDim->SetMax(matrix.getVehTimeWindows().at(i).second);

        }





        parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);


        if ("CVRPTW_TABU       " == meta_euristic      ) parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_TABU_SEARCH);

        if ("CVRPTW_SA       " == meta_euristic    ) parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_SIMULATED_ANNEALING);


        else parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);



//         SearchMonitor* const no_improvement_limit =  MakeNoImprovementLimit(routing.solver(),  routing.CostVar(), 10, true );
//         routing.AddSearchMonitor(no_improvement_limit);




        //set callback to custom limit


        //set time limit

        parameters.set_time_limit_ms(time);
        // parameters.set_solution_limit(50);

        //parameters.set_solution_limit(2000);

        // std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;

        // routing.CumulVar(routing.Start(0), "time")


        const int64 kPenalty = 10000000;
        const RoutingModel::NodeIndex kFirstNodeAfterDepot(1);
        for (RoutingModel::NodeIndex order = kFirstNodeAfterDepot;
             order < routing.nodes(); ++order) {
            std::vector<RoutingModel::NodeIndex> orders(1, order);
            routing.AddDisjunction(orders, kPenalty);
        }


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

            std::vector<int64> unass_orders;
            for(int i = 0; i < cvrp_result.size(); ++i){

                unass_orders.insert(end(unass_orders), begin(cvrp_result.at(i)), end(cvrp_result.at(i)));
            }

            std::sort(begin(unass_orders), end(unass_orders));


            std::vector<int64>::iterator it;
            it = std::unique (begin(unass_orders), end(unass_orders));   // 10 20 30 20 10 ?  ?  ?  ?

            unass_orders.resize( std::distance(begin(unass_orders),it) );


            std::cout << "Initial number of nodes: " << matrix.getDemands().size() << " number of nodes in solution: " << unass_orders.size() << std::endl;
            std::cout << "Difference: " << matrix.getDemands().size() - unass_orders.size();

            return cvrp_result;

        } else {
            LG << "No solution found.";
            LG <<" Trying to solve with more time";

            return  CVRPTWSolver::SolveCVRPTW(matrix, num_v, meta_euristic, time  * 2);

            // return CVRPTWSolver::SolveCVRPTW_Best_Solution(matrix, num_v);

        }
    }

    std::vector<std::vector<int64>> CVRPTWSolver::SolveCVRPTW(Matrix &matrix, int64 num_v,
                                                              std::vector<std::vector<int64>> initialSolution,
                                                              std::string meta_euristic, int64 time) {

        std::vector<int64> result;
        std::vector<std::vector<int64>> cvrp_result;

        const int size = matrix.getSize();

        RoutingModel routing(size, num_v);

        routing.SetCost(NewPermanentCallback(&matrix, &Matrix::Distance));

        //routing.AddVectorDimension(&demands[0], capacity, true, "Demand"); // demands for each vector




        routing.AddDimensionWithVehicleCapacity(NewPermanentCallback(&matrix, &Matrix::demandForANode), 0,
                                                matrix.getVcaps(), true,
                                                "Capacity");

        routing.AddDimension(NewPermanentCallback(&matrix, &Matrix::distancePlusServiceTime), matrix.getHorizon(),
                             matrix.getHorizon(), false, "time");

        RoutingSearchParameters parameters = routing.DefaultSearchParameters();



        for (RoutingModel::NodeIndex i(0); i < matrix.getSize(); ++i) {
            int64 index = routing.NodeToIndex(i);
            IntVar* const cumul_var = routing.CumulVar(index, "time");

            cumul_var->SetMin(matrix.getTimeWindow(i.value()).first);
            cumul_var->SetMax(matrix.getTimeWindow(i.value()).second);
        }

        for(int i = 0; i < num_v; i++){

            int64 startNode  = routing.Start(i);
            IntVar* timeStartDim = routing.CumulVar(startNode, "time");
            timeStartDim->SetMin(matrix.getVehTimeWindows().at(i).first);
            int64 endNode = routing.End(i);
            IntVar* timeEndDim = routing.CumulVar(endNode, "time");
            timeEndDim->SetMax(matrix.getVehTimeWindows().at(i).second);

        }

        //   FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION;



        parameters.set_first_solution_strategy(FirstSolutionStrategy::PARALLEL_CHEAPEST_INSERTION);


        if ("CVRPTW_TABU       " == meta_euristic      ) parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_TABU_SEARCH);

        if ("CVRPTW_SA       " == meta_euristic    ) parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_SIMULATED_ANNEALING);


        else parameters.set_local_search_metaheuristic(LocalSearchMetaheuristic_Value_GUIDED_LOCAL_SEARCH);




//        SearchMonitor* const no_improvement_limit =  MakeNoImprovementLimit(routing.solver(),  routing.solver()->MakeIntVar(0,5000000), 4000, true );
//        routing.AddSearchMonitor(no_improvement_limit);




        //set callback to custom limit


        //set time limit

        parameters.set_time_limit_ms(time);
        parameters.set_solution_limit(10);

        //std::cout << "Local search metaeuristics " << parameters.local_search_metaheuristic() << std::endl;




        const int64 kPenalty = 10000000;
        const RoutingModel::NodeIndex kFirstNodeAfterDepot(1);
        for (RoutingModel::NodeIndex order = kFirstNodeAfterDepot;
             order < routing.nodes(); ++order) {
            std::vector<RoutingModel::NodeIndex> orders(1, order);
            routing.AddDisjunction(orders, kPenalty);
        }


        routing.CloseModelWithParameters(parameters);

        // Forbidding empty routes (optional)
//         for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
//           IntVar* const start_var = routing.NextVar(routing.Start(vehicle_nbr));
//           for (int64 node_index = routing.Size(); node_index < routing.Size() + routing.vehicles(); ++node_index) {
//             start_var->RemoveValue(node_index);
//           }
//         }
        std::vector<std::vector<RoutingModel::NodeIndex>> initialSol;

        for(int i = 0; i < initialSolution.size(); ++i){
            initialSol.push_back(std::vector<RoutingModel::NodeIndex>(initialSolution.at(i).size()));
            for(int j = 0; j < initialSolution.at(i).size(); ++j){
                initialSol[i][j] = RoutingModel::NodeIndex(initialSolution[i][j]);
            }
        }


        Assignment*  initSolution = routing.solver()->MakeAssignment();
        routing.RoutesToAssignment(initialSol, true, true, initSolution);
        routing.solver()->CheckAssignment(initSolution);


        // SOLVE          we can use different solving strategies
        const Assignment *solution = routing.SolveFromAssignmentWithParameters(initSolution, parameters);

        //const Assignment* solution = routing.Solve(routing.solver()->MakeAssignment());

        // INSPECT SOLUTION
        if (solution != NULL) {


          //  LG <<  solution->DebugString();
            // Solution cost.
            std::cout << "Obj value: " << solution->ObjectiveValue() << std::endl;
            // Inspect solution.
            std::string route;
            for (int vehicle_nbr = 0; vehicle_nbr < num_v; ++vehicle_nbr) {
                route = "";
                result.clear();
                for (int64 node = routing.Start(vehicle_nbr); !routing.IsEnd(node);
                     node = solution->Value(routing.NextVar(node))) {
                  //  IntVar* load = routing.GetDimensionOrDie("Capacity").CumulVar(node);
                    //IntVar* time = routing.GetDimensionOrDie("time").CumulVar(node);
                    route = StrCat(route, StrCat(routing.IndexToNode(node).value(), " -> "));
                    result.push_back(routing.IndexToNode(node).value());
                }
                route = StrCat(route, routing.IndexToNode(routing.End(vehicle_nbr)).value());
                std::cout << "Route #" << vehicle_nbr + 1 << std::endl << route << std::endl;
                cvrp_result.push_back(result);
            }

            std::vector<int64> unass_orders;
            for(int i = 0; i < cvrp_result.size(); ++i){

                unass_orders.insert(end(unass_orders), begin(cvrp_result.at(i)), end(cvrp_result.at(i)));
            }

            std::sort(begin(unass_orders), end(unass_orders));



            std::vector<int64>::iterator it;
            it = std::unique (begin(unass_orders), end(unass_orders));   // 10 20 30 20 10 ?  ?  ?  ?

            unass_orders.resize(std::distance(begin(unass_orders),it));


            std::cout << "Initial number of nodes: " << matrix.getDemands().size() << " number of nodes in solution: " << unass_orders.size() << std::endl;
            std::cout << "Difference: " << matrix.getDemands().size() - unass_orders.size() << std::endl;

            return cvrp_result;

        } else {
            LG << "No solution found.";
            LG <<" Trying to solve with more time";

            return  CVRPTWSolver::SolveCVRPTW(matrix, num_v, meta_euristic, time  * 2);

            // return CVRPTWSolver::SolveCVRPTW_Best_Solution(matrix, num_v);

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
//    int constant = 300;
//
//    for (int i = 0; i < timeWindows.size(); i++){
//        int64 f = timeWindows.at(i).first;
//        int64 s = timeWindows.at(i).second;
//        timeWindows.at(i).first = f + constant;
//        timeWindows.at(i).second = s + constant;
//    }
//
//
//   // std::vector<int64> serviceTime = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//    std::vector<int64> serviceTime =   {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//
//    std::vector<int64> demands = {0, 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1};
//    assert(demands.size() == 21);
//    std::vector<int64> capacities = {100, 100};
//    std::vector<std::pair<operations_research::RoutingModel::NodeIndex, operations_research::RoutingModel::NodeIndex>> depots(1);
//
//    depots[0] = std::make_pair(0, 0);
//    //depots[1] = std::make_pair(10, 10);
////    depots[2] = std::make_pair(4, 10);
////    depots[3] = std::make_pair(2, 5);
//
//    std::vector<std::vector<int64>> initialsolution = {
//
//            {16, 9, 19, 17, 18, 10, 12, 6, 13, 7, 2, 4, 20, 3},
//            {15, 5, 1, 11, 8, 14},
//    };
//
//    std::vector<std::pair<int64, int64 >> vehWindows = {{0, timeWindows.at(0).second},{0, timeWindows.at(0).second}};
//
//    operations_research::Matrix matrix(vec, demands, capacities, timeWindows, serviceTime, vehWindows);
//
//
//
//    operations_research::CVRPTWSolver::SolveCVRPTW(matrix, capacities.size(), initialsolution,"CVTPTW_GLS", 30 * 1000);
//
//
//
//    return 0;
//}
