#include <iostream>
#include <cmath>
int main(){
    int x, n, sum=0;
    std::cin>>x>>n;
    for (int i=0;i<n;i++){
        sum += pow(x,i+1);
    }
    std::cout<<sum;
}