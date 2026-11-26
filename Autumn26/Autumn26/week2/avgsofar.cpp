#include <iostream>

int main(){
    double n = -1, sum = 0, counter = 0;
    while(n != 0){
        std::cin >> n;
        sum += n;
        counter += 1;
        std::cout << "average so far: " << sum / counter << std::endl;
    }
}