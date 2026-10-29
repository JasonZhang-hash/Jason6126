#include <iostream>
#include <vector>

int min_vector_element(const std::vector<double>& vin){
    double min;
    int index = 0;
    min = vin.at(0);
    for (int i=1;i<vin.size();i++){
        if (vin.at(i) < min){
            min = vin.at(i);
            index = i;
        }
    }
    return index;
}

int main(){
    std::vector<double> v{3, 0.5, -1};
    std::cout<< min_vector_element(v) <<std::endl;
}