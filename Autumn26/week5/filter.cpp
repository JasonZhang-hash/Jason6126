#include <iostream>
#include <vector>

int main(){
    int n,num,t;
    std::vector<int> v;
    std::cin>>n;
    for (int i=0;i<n;i++){
        std::cin>>num;
        v.push_back(num);
    }
    std::cin>>t;
    for (int i=0;i<n;i++){
        if (v.at(i) < t){
            std::cout << v.at(i) << std::endl;
        }
    }
}