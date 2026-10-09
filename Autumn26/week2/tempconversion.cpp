#include <iostream>
int main(){
    double value;
    char unit;
    std::cin >> value >> unit;
    if (unit == 'c' || unit == 'C'){
        value = (value * 9 / 5) + 32;
        std::cout << value << " F" << std::endl;
    }   
    else if (unit == 'f' || unit == 'F'){
        value = (value - 32) * 5 / 9;
        std::cout << value << " C" << std::endl;
    }
    else{
        std::cout << "invalid unit" << std::endl;
    }
}