#include <iostream>
#include <fstream>
#include <string>
#include <vector>

int main(){
    std::ifstream file1;
    std::ifstream file2;
    std::vector<std::string> tocheck;
    std::vector<int> numcheck;
    file2.open("words.txt");
    std::string tmp;
    int cnt = 0;
    while(file2>>tmp){
        tocheck.push_back(tmp);
        numcheck.push_back(0);
        cnt+=1;
    }
    file1.open("text.txt");
    while(file1>>tmp){
        for (int i=0;i<cnt;i++){
            if (tmp == tocheck[i]){
                numcheck[i] += 1;
            }
        }
    }
    for (int i=0;i<cnt;i++){
        std::cout<<tocheck[i]<<": "<<numcheck[i]<<std::endl;
    }

}