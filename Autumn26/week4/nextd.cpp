#include <iostream>

bool leapyear(int year){
    bool ly;
    if ((year % 4)== 0){
        if ((year % 100)== 0){
            if ((year % 400)== 0){
                ly = true;
            }
            else{
                ly = false;
            }
        }
        else{
            ly = true;
        }
    }
    else{
        ly = false;
    }
    return ly;
}

void next_date(int d1, int m1, int y1, int& d2, int& m2, int& y2){
    d2=d1+1;
    m2= m1;
    y2=y1;

    if (m1 == 2){
        if (leapyear(y1) && d2 == 30){
            m2+=1;
            d2=1;
        }
        if ((!leapyear(y1)) && d2 == 29){
            m2+=1;
            d2=1;
        }
    }
    if ((m1 == 4)||(m1 == 6)||(m1 == 9)||(m1 == 11)){
        if (d2 == 31){
            m2+=1;
            d2=1;
        }
    }
    if(m1 == 1||m1 == 3||m1 == 5||m1 == 7||m1 == 8||m1 == 10||m1 == 12){
        if (d2 == 32){
            m2+=1;
            d2=1;
            if (m2==13){
                y2+=1;
                m2=1;
            }
        }
    }
}
int main(){
    int d,m,y,da,ma,ya;
    std::cin>>d>>m>>y;
    next_date(d,m,y,da,ma,ya);
    std::cout<<da<<" "<<ma<<" "<<ya<<" "<<std::endl;

}