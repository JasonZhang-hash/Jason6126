#include <iostream>

int main(){
    int n = -1, sum = 0;
    while(n != 0){
        std::cin >> n;
        sum += n;
        std::cout << "sum so far: " << sum << std::endl;
    }
}