#include <iostream>
#include <vector>
#include <string>

bool findword(std::string& line, std::string word){
    bool found = false;
    std::string myline;
    for (int i=0;i<line.size();i++){
        myline.push_back('#');
    }
    for (int i=0;i<line.size();i++){
        if (line[i]==word[0]){
            found = true;
            int n=1;
            while (found && n<word.size()){ //&& n+i<line.size()
                if (line[i+n]!=word[n]){
                    found = false;
                }
                n+=1;
            }
            if (found){
                for (int j=0;j<n;j++){
                    myline[i+j]=word[j];
                }
                line =  myline;
                return found;
            }
        }
    }
    line = myline;
    return found;
}

int main(){
    std::string line = "xappleg";
    std::string word = "apple";

    if (findword(line,word)){
        std::cout<<"true "<<line<<std::endl;
    }
    else{
        std::cout<<"wrong "<<line<<std::endl;
    }
}
