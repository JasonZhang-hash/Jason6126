#include <iostream>

int main(){
    int n, curmax, count = 0;

    std::cout << "please enter number:" << std::endl;
    std::cin >> n;

    curmax = n;

    while(n != 1){
        if(n % 2 == 0){
            n = n/2;
        }
        else if(n % 2 == 1){
            n = 3*n + 1;
        }
        if(n > curmax){
            curmax = n;
        }

        std::cout << n << " ";
        count ++;
    }
    std::cout<<std::endl;
    std::cout<<"max is: "<<curmax<<" count is :"<<count;

}