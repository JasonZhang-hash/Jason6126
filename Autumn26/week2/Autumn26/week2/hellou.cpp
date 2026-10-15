#include <iostream>

int main(){
    std::string user_firstname;
    std::string user_surname;
    std::cout << "hello, what is your name?: " << std::endl;
    std::cin >> user_firstname;
    std::cout << "hello, what is your surname?: " << std::endl;
    std::cin >> user_surname;
    std::cout << "hello, " << user_firstname << " " << user_surname<< std::endl;
}