
#include "matr.h"



namespace operations_research{


    Matrix::Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities)
    {
        this->size = vec.size();

        this->matrix_.reset(new int64[size * size]);

        this->dems = demands;
        this->vcaps = v_capacities;


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




    Matrix::Matrix(std::vector<std::vector<int64>> &vec, std::vector<int64> demands, std::vector<int64 > v_capacities, std::vector<std::pair<int64, int64>> timeWindows, std::vector<int64> serviceTime )
    {
        this->size = vec.size();

        this->matrix_.reset(new int64[size * size]);

        this->dems = demands;
        this->vcaps = v_capacities;
        this->timeWindows = timeWindows;
        this->serviceTime = serviceTime;
        this->horizon = this->timeWindows.at(0).second;


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


}

