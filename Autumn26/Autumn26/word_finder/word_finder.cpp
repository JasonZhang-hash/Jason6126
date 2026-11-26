#include <iostream>
#include <vector>
#include <string>

void find_words(
    const std::vector<std::string>& grid, 
    const std::vector<std::string>& words,
    std::vector<std::string>& solution,
    std::vector<std::string>& not_found
){
    bool iffound = false;
    solution = grid
    for (int i=0;i<words.size();i++){
        iffound = false;
        for (int j=0;j<grid.size();j++){
            iffound = findrow(grid[j],words[i]);
        }
        for (int j=0;j<words[0].size();j++){
            iffound = findcol(grid,words[i]);
        }
        if (!iffound){
            not_found.push_back(words[i]);
        }
    }
}
void merge_grid(std::vector<std::string>& grid1,std::vector<std::string>& grid2){
    for (int i=0;i<grid2.size();i++){
        for (int j=0;j<grid2[0].size();j++){
            if (grid2[i][j] != "#"){
                grid1[i][j] = grid2[i][j];
            }
        }
    }
}
void findword(std::string line, std::string word){
    
}


