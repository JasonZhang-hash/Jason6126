#include <iostream>
#include <vector>

void merge_vectors_int(const std::vector<int>& v1, const std::vector<int>& v2, std::vector<int>& merged){
    int cnt1 = 0, cnt2 = 0;
    while (cnt1 < v1.size() && cnt2 < v2.size()){
        if (v1[cnt1]<v2[cnt2]){
            merged.push_back(v1[cnt1]);
            cnt1 += 1;
        }
        else{
            merged.push_back(v2[cnt2]);
            cnt2 += 1;
        }
    }
    for (;cnt2<v2.size();cnt2++){
        merged.push_back(v2[cnt2]);
    }
    for (;cnt1<v1.size();cnt1++){
        merged.push_back(v1[cnt1]);
    }
}


int main(){

    std::vector<int> v1;
    std::vector<int> v2;
    std::vector<int> out;

    std::vector<int> expected_output;

    v1 = {1, 3, 5, 6};
    v2 = {4, 6, 10};
    expected_output = {1, 3, 4, 5, 6, 6, 10};

    merge_vectors_int(v1, v2, out);
    if(out == expected_output){ // we can compare the content of vectors (at least in some cases)
        std::cout << "passed" << std::endl;
    }
    else{
        std::cout << "not passed" << std::endl;
    }
}