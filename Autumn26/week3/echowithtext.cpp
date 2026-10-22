#include <iostream>
#include <string>
using namespace std;

int main(){
    string text;
    std::cin>>text;

    while(text != "STOP"){
        std::cout<<text<<std::endl;
        std::cin>>text;
    }
}