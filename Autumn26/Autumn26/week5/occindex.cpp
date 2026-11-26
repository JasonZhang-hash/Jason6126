#include <iostream>
#include <vector>

int main(){
    int n,num,t,cnt=0;
    std::vector<int> v;
    std::vector<int> index;
    std::cout << "please enter n: " << std::endl;
    std::cin>>n;
    for (int i=0;i<n;i++){
        std::cout << "please enter a number: " << std::endl;
        std::cin>>num;
        v.push_back(num);
    }
    std::cout << "please enter s: " << std::endl;
    std::cin>>t;
    for (int i=0;i<n;i++){
        if (v.at(i) == t){
            cnt += 1;
            index.push_back(i);
        }
    }
    if (cnt == 0){
        std::cout << "there are no occurrences of 3" << std::endl;
    }
    else{
        std::cout << "there are "<< cnt << " occurrences of 3 at indices: " << std::endl;
        for (int i=0;i<cnt;i++){
            std::cout<<index.at(i)<<std::endl;
        }
    }
}