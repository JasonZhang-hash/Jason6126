#include <iostream>

void intdiv(int a, int b, int& quo, int& rem){
    quo = 0;
    while (a>b){
        a-=b;
        quo+=1;
    }
    rem = a;
}

int main(){
    int a, b, quo, rem;
    std::cin>>a>>b;
    intdiv(a,b,quo,rem);
    std::cout<<quo<<" "<<rem<<std::endl;
}