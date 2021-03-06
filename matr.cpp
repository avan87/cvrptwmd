
#include "matr.h"
#include <cmath>


namespace operations_research{


    Matrix::Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities)

    :size(vec.size()), dems(demands), vcaps(v_capacities)


    {


        this->matrix_.reset(new int64[size * size]);




        for (RoutingModel::NodeIndex from = RoutingModel::kFirstNode; from < size;
             ++from) {
            for (RoutingModel::NodeIndex to = RoutingModel::kFirstNode; to < size;
                 ++to) {
                if (to != from) {
                    matrix_[MatrixIndex(from, to)] = vec[from.value()][to.value()];
                } else {
                    matrix_[MatrixIndex(from, to)] = 0LL;
                }
            }
        }

    }




    Matrix::Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities,
                   std::vector<std::pair<int64, int64>> timeWindows, std::vector<int64> serviceTime,
                   std::vector<std::pair<int64, int64>> vehWindows)
    : size(vec.size()), vehWindows(vehWindows), dems(demands), vcaps(v_capacities), timeWindows(timeWindows), serviceTime(serviceTime), horizon(timeWindows.at(0).second)
    {
        this->size = vec.size();

        this->matrix_.reset(new int64[size * size]);
        //this->vehWindows = vehWindows;
       // this->dems = demands;
       // this->vcaps = v_capacities;
       // this->timeWindows = timeWindows;
       // this->serviceTime = serviceTime;
        //this->horizon = this->timeWindows.at(0).second;


        for (RoutingModel::NodeIndex from = RoutingModel::kFirstNode; from < size;
             ++from) {
            for (RoutingModel::NodeIndex to = RoutingModel::kFirstNode; to < size;
                 ++to) {
                if (to != from) {
                    matrix_[MatrixIndex(from, to)] = vec[from.value()][to.value()];
                } else {
                    matrix_[MatrixIndex(from, to)] = 0LL;
                }
            }
        }

    }



    Matrix::Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities, std::vector<std::pair<int64, int64>> timeWindows,
            std::vector<int64> serviceTime, std::vector<std::pair<int64, int64>> vehWindows , std::vector<std::pair<int64, int64>> depots)

            : size(vec.size()), vehWindows(vehWindows),
              dems(demands), vcaps(v_capacities), timeWindows(timeWindows),
              serviceTime(serviceTime), horizon(timeWindows.at(0).second)

    {

        this->size = vec.size();

        this->matrix_.reset(new int64[size * size]);
        //this->vehWindows = vehWindows;
       // this->dems = demands;
       // this->vcaps = v_capacities;
        //this->timeWindows = timeWindows;
        //this->serviceTime = serviceTime;

        std::vector<std::pair<int64, int64>>::iterator first = timeWindows.begin() + 0;
        std::vector<std::pair<int64, int64>>::iterator last = timeWindows.begin() + 10;


        std::vector<std::pair<int64, int64>> wharehouseTimewindows(first, last);
        std::vector<int64 > horizons;
        std::for_each(wharehouseTimewindows.begin(), wharehouseTimewindows.end(), [&horizons] (std::pair<int64, int64 > x) {horizons.push_back(x.second);});
        int maxHorizon  = std::accumulate(horizons.begin(), horizons.end(), 1 , [] (int64 a, int64 b) { return std::max(a,b );});

        this->horizon = maxHorizon;


        for (RoutingModel::NodeIndex from = RoutingModel::kFirstNode; from < size;
             ++from) {
            for (RoutingModel::NodeIndex to = RoutingModel::kFirstNode; to < size;
                 ++to) {
                if (to != from) {
                    matrix_[MatrixIndex(from, to)] = vec[from.value()][to.value()];
                } else {
                    matrix_[MatrixIndex(from, to)] = 0LL;
                }
            }
        }


       for(int  i = 0; i < depots.size(); ++i){
           RoutingModel::NodeIndex start(depots.at(i).first);
           RoutingModel::NodeIndex end(depots.at(i).second);

           this->depots.push_back(std::make_pair(start, end));
       }


    }


    int64 Matrix::Distance(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) const{
        return matrix_[MatrixIndex(from, to)];
    }

    int Matrix::getSize()const{
        return this->size;
    }


    std::vector<int64> Matrix::getDemands() {return this->dems;}



    int64 Matrix:: demandForANode(RoutingModel::NodeIndex from , RoutingModel::NodeIndex to){
        return this->dems.at(from.value());

    }


    int64 Matrix::getVehicleCapicity(int64 num){

            return this->vcaps.at(num);
    }



    int64 Matrix::MatrixIndex(RoutingModel::NodeIndex from,
                              RoutingModel::NodeIndex to) const {
        return (from * size + to).value();
    }

    std::pair<int64, int64> Matrix::getTimeWindow(int64 i) {
        return this->timeWindows.at(i);
    }

    int64 Matrix::getServiceTime(int64 i) {
        return this->serviceTime.at(i);
    }

    int64 Matrix::distancePlusServiceTime(RoutingModel::NodeIndex from, RoutingModel::NodeIndex to) {
        return Distance(from, to) + getServiceTime(from.value());
    }

    int64 Matrix::getHorizon() const {
        return this->horizon;
    }

    std::vector<std::pair<int64, int64>> Matrix::getVehTimeWindows() {
        return this->vehWindows;
    }

    Matrix::Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64> v_capacities,
                   std::vector<std::pair<int64, int64>> timeWindows, std::vector<int64> serviceTime,
                   std::vector<std::pair<int64, int64>> vehWindows, std::vector<int64> deliveries,
                   std::vector<int64> pickups)
    : vehWindows(vehWindows), dems(demands), vcaps(v_capacities), timeWindows(timeWindows),
      serviceTime(serviceTime),
      deliveries(deliveries), pickups(pickups), horizon(timeWindows.at(0).second)
    {

        this->size = vec.size();

        this->matrix_.reset(new int64[size * size]);
        //this->vehWindows = vehWindows;
       // this->dems = demands;
       // this->vcaps = v_capacities;
       // this->timeWindows = timeWindows;
       // this->serviceTime = serviceTime;
       // this->deliveries = deliveries;
       // this->pickups = pickups;
       // this->horizon = timeWindows.at(0).second;


        for (RoutingModel::NodeIndex from = RoutingModel::kFirstNode; from < size;
             ++from) {
            for (RoutingModel::NodeIndex to = RoutingModel::kFirstNode; to < size;
                 ++to) {
                if (to != from) {
                    matrix_[MatrixIndex(from, to)] = vec[from.value()][to.value()];
                } else {
                    matrix_[MatrixIndex(from, to)] = 0LL;
                }
            }
        }

    }


}

