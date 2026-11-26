#include <iostream>
#include <cmath>

int main(){
    int a,b,c;
    std::cin>>a>>b>>c;
    if(a+b>c && a+c>b && b+c>a){
        std::cout<<"valid triangle"<<std::endl;
        if (a==b || b==c || a==c){
            std::cout<<"isosceles triangle"<<std::endl;
        }
        else if(a==b && b==c){
            std::cout<<"equilateral triangle"<<std::endl;
        }
        else{
            std::cout<<"scalene triangle"<<std::endl;
        }
        std::cout << "perimeter: " << a+b+c << std::endl;
        std::cout << "area " << std::sqrt((a+b+c)/2 * ((a+b+c)/2 - a) * ((a+b+c)/2 - b) * ((a+b+c)/2 - c)) << std::endl;
    }
        
         

    else{
        std::cout<<"invalid triangle"<<std::endl;
    }
}