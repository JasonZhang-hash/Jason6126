#include <iostream>
int main(){
    double a, b, c;

    a = 1;
    b = 2;
    c = a + b;

    std::cout << c << std::endl;

    a = 2;
    //expected output 3
    std::cout << c << std::endl;
    c = a + b;
    //expected output 4
    std::cout << c << std::endl;
    std::cout << 5 / 2.0 << std::endl;

    //rectangles
    std::cout << "enter length and width of rectangle" << std::endl;
    double length, width, area, perimeter;
    std::cin >> length >> width;
    area = length * width;
    perimeter = 2 * (length + width);
    std::cout << "area: " << area << std::endl;
    std::cout << "perimeter: " << perimeter << std::endl;

    //temperature conversion
    std::cout << "enter temperature in Celsius" << std::endl;
    double celsius, fahrenheit;
    std::cin >> celsius;
    fahrenheit = (celsius * 9 / 5) + 32;
    std::cout << "temperature in Fahrenheit: " << fahrenheit << std::endl;
}