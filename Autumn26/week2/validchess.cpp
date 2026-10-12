#include <iostream>

int main(){
    int x1,y1,x2,y2;
    std::cout<<"give the coordinate of initial position";
    std::cin >>x1>>y1;
    std::cout<<"give the coordinate of final position";
    std::cin >>x2>>y2;
    bool ifvalid = false;
    if (x1<8 && x1>0 && x2<8 && x2>0 && y1<8 && y1>0 && y2<8 && y2>0){
        if (x2 == x1+1 || x2 == x1-1){
            if (y2 == y1+2 || y2 == y1-2){
                ifvalid = true;
            }
        }
        if (y2 == y1+1 || y2 == y1-1){
            if (x2 == x1+2 || x1 == y1-2){
                ifvalid = true;
            }
        }
    }
    if (ifvalid){
        std::cout<<"The move is a valid knight move";
    }
    else{
        std::cout<<"The move is an invalid knight move";
    }
}