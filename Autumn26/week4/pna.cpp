#include <iostream>

void pna(double a, double b, double& Perimeter, double& Area){
    Perimeter = 2*(a+b);
    Area = a*b;
}

int main(){
    double a, b, pe, ar;
    std::cin>>a>>b;
    pna(a,b,pe,ar);
    std::cout<<pe<<" "<<ar<<std::endl;
}