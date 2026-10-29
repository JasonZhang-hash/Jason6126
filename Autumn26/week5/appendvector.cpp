#include <iostream>
#include <vector>

void append_vectors(const std::vector<int>& vin1, const std::vector<int>& vin2, std::vector<int>& vout){
    for (int i=0;i<vin1.size();i++){
        vout.push_back(vin1.at(i));
    }
    for (int i=0;i<vin2.size();i++){
        vout.push_back(vin2.at(i));
    }
}

