#include <iostream>

int main(){
    int n,num;
    std::cin>>n;
    for (int i=1;i<n+1;i++){
        for(int j=1;j<n+1;j++){
            num = i*j;
            if (num<10){
                std::cout<<num<<"\t";
            }
            else {
                std::cout<<num<<"\t";
            }
        }
        std::cout<<std::endl;

    }
                
}