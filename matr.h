//
// Created by veter on 07.06.16.
//

#ifndef CVRP_MATR_H
#define CVRP_MATR_H


#include "constraint_solver/routing.h"


namespace operations_research  {



    class Matrix
    {
    public:

        Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities);


        Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities, std::vector<std::pair<int64, int64>> timeWindows,
               std::vector<int64> serviceTime, std::vector<std::pair<int64, int64>> vehWindows );


        Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities,
               std::vector<std::pair<int64, int64>> timeWindows,
               std::vector<int64> serviceTime, std::vector<std::pair<int64, int64>> vehWindows , std::vector<std::pair<int64,int64>> depots);

       Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities,
              std::vector<std::pair<int64, int64>> timeWindows,
              std::vector<int64> serviceTime, std::vector<std::pair<int64, int64>> vehWindows,
              std::vector<int64> deliveries, std::vector<int64> pickups
       );

        int64 Distance(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const;

        int getSize()const;

        std::vector<int64>getDemands();

        int64 getVehicleCapicity(int64 num);

        int64 demandForANode(RoutingModel::NodeIndex from , RoutingModel::NodeIndex to);

        std::pair<int64, int64> getTimeWindow(int64 i);

        int64 getServiceTime(int64 i);

        int64 distancePlusServiceTime(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to);


        const vector<pair<RoutingModel::NodeIndex, RoutingModel::NodeIndex>> &getDepots() const {
            return depots;
        }

        int64 getHorizon() const ;

        const vector<int64> &getDeliveries() const {
            return deliveries;
        }

        const vector<int64> &getPickups() const {
            return pickups;
        }

        std::vector<std::pair<int64, int64>> getVehTimeWindows();


        const vector<int64> &getVcaps() const {
            return vcaps;
        }

    private:

        int64 MatrixIndex(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const;

        std::unique_ptr<int64[]> matrix_;
        int size;

        //demands for each node
        std::vector<int64> dems;

        //capacities for each vehicle

        std::vector<int64> vcaps;

        std::vector<std::pair<int64, int64>> timeWindows;

        std::vector<std::pair<int64, int64>> vehWindows;

        std::vector<int64> serviceTime;

        int64 horizon;

        std::vector<pair<RoutingModel::NodeIndex , RoutingModel::NodeIndex>> depots;

        std::vector<int64> deliveries;

        std::vector<int64> pickups;



    };


}

#endif //CVRP_MATR_H
