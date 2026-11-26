#include <iostream>
#include <fstream>

int main(int argc, char* argv[]){
    std::ifstream myfile;
    std::string tmp;
    myfile.open(argv[1]);
    while(myfile>>tmp){
        std::cout<<tmp<<std::endl;
    }
}
