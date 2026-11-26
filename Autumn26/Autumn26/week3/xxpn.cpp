#include <iostream>

int main(){
    int x,n, product = 1;
    std::cin >> x >> n;
    for(int i = 0; i<n; i++){
        product = product * x;
    }
    std::cout<<product<<std::endl;
}