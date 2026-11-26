#include <iostream>

double mypow(double base, int exp){
    double result=1;
    if (exp>=0){
        for (int i=0;i<exp;i++){
        result *= base;
    }
    return result;
    }
    for (int i=0;i>exp;i--){
        result /= base;
    }
    return result;
    
}

int main(){
    std::cout << "2^3: "<< mypow(2,3)<<std::endl;
    std::cout << "0.5^2: "<< mypow(0.5,2)<<std::endl;
    std::cout << "1^0: "<< mypow(1,0)<<std::endl;
    std::cout << "4^-2: "<< mypow(4,-2)<<std::endl;
    int x, n, sum=0;
    std::cin>>x>>n;
    for (int i=0;i<n;i++){
        sum += mypow(x,i+1);
    }
    std::cout<<sum;
}
