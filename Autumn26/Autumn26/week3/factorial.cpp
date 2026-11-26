#include <iostream>

int main(){
    int x, product = 1;
    std::cin >> x;
    for(int i = 1; i<x+1; i++){
        product = product * i;
    }
    std::cout<<product<<std::endl;
}