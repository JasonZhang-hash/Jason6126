#include <iostream>
#include <vector>
#include <string>

void merge_grid(std::vector<std::string>& grid1,std::vector<std::string>& grid2){
    for (int i=0;i<grid2.size();i++){
        for (int j=0;j<grid2[0].size();j++){
            if (grid2[i][j] != '#'){
                grid1[i][j] = grid2[i][j];
            }
        }
    }
}

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
                std:: cout << "found"<<word<<"\n";
                return found;
            }
        }
    }
    line = myline;
    return found;
}

std::vector<std::string> invert(std::vector<std::string> grid){
    std::vector<std::string> newgrid;
    std::string col;
    for (int j=0;j<grid[0].size();j++){
        col = "";
        for (int i=0;i<grid.size();i++){
            col.push_back(grid[i][j]);
        }
        newgrid.push_back(col);
    }
    return newgrid;
}

void find_words(
    const std::vector<std::string>& grid, 
    const std::vector<std::string>& words,
    std::vector<std::string>& solution,
    std::vector<std::string>& not_found
){
    bool iffound = false;
    std::string line;
    std::vector<std::string> gridinv;
    gridinv = invert(grid);

    for (int j=0;j<grid.size();j++){
        std::string col = "";
        for (int i=0;i<grid[0].size();i++){
            col.push_back('#');
        }
        solution.push_back(col);
    }
   


    for (int i=0; i<words.size(); i++){
        std::vector<std::string> mysol;
        std::vector<std::string> mysolinv;
        iffound = false;
        bool a = false;
        for (int j=0;j<grid.size();j++){
            line = grid[j];
            iffound = findword(line,words[i]);
            if (iffound){
                a = true;
            }
            mysol.push_back(line);
        }

        for (int j=0;j<gridinv.size();j++){
            line = gridinv[j];
            iffound = findword(line,words[i]);
            if (iffound){
                a = true;
            }
            mysolinv.push_back(line);
        }

        if (!a){
            not_found.push_back(words[i]);
        }
        mysolinv = invert(mysolinv);
        merge_grid(mysol,mysolinv);
        merge_grid(solution,mysol);
    }
}
int main(){

    std::vector<std::string> grid = {
        "xappleg",
        "dzeazkg",
        "zwazzbx",
        "dxnqzoy",
        "qyueytp",
        "pythtyg"
    };

    std::vector<std::string> words = {"peanut", "bot", "mango", "apple", "cinnamon"};
    std::vector<std::string> solution;
    std::vector<std::string> not_found;

    find_words(grid, words, solution, not_found); 

    std::vector<std::string> expected_solution = {
        "#apple#",
        "##e####",
        "##a##b#",
        "##n##o#",
        "##u##t#",
        "##t####"
    };

    std::vector<std::string> expected_not_found = {"mango", "cinnamon"};

    if( (solution == expected_solution) && (not_found == expected_not_found) ){
        std::cout << "test passed" << std::endl;
    }
    else{
        std::cout << "test failed" << std::endl;
    }

    std::cout << "solution:" << std::endl;
    for(int i = 0; i < solution.size(); i++){
        std::cout << solution[i] << std::endl;
    }

    std::cout << "not found:" << std::endl;
    for(int i = 0; i < not_found.size(); i++){
        std::cout << not_found[i] << std::endl;
    }

}