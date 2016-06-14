//
// Created by veter on 09.06.16.
//

#include "utils.h"

    std::vector<std::vector<int64> > Utils::cast2dList(const std::vector<std::vector<int64_t> > &vec) {

       int size = vec.size();
       std::vector<long long int> v2 (size, 0);
       std::vector<std::vector<long long int>> tempVec(size, v2);

       for(int i=0; i< tempVec.size(); i++) {
           for (int j=0;j<tempVec[i].size(); j++){

               tempVec[i][j] = (long long int) vec[i][j];
               // std::cout << tempVec[i][j] << " ";
           }
           // std::cout << std::endl;
       }

       return tempVec;

}

    std::vector<int64> Utils::castList(const std::vector<int64_t> &list) {

       int listSize = list.size();

       std::vector<long long int> newList (listSize, 0);

       for(int i=0; i< listSize; i++) {

           newList[i] = (long long int)list[i];
       }


   }

std::vector<std::pair<int64, int64>> Utils::castToListOfPairs(const std::vector<std::vector<int64_t> > &timeWindows) {

    int size = timeWindows.size();

    std::vector<std::pair<int64, int64>> tempVec;



    for(int i=0; i < timeWindows.size(); i++) {

        for(int j=0; j < timeWindows.at(i).size() - 1; j++) {

            std::pair<int64, int64> p;

            p.first = timeWindows.at(i).at(j);
            p.second = timeWindows.at(i).at(j + 1);
            //
            // std::cout << p.first << " : " << p.second << std::endl;

            tempVec.push_back(p);
            // std::cout << tempVec[i][j] << " ";
        }
        // std::cout << std::endl;
    }




    return tempVec;

}


