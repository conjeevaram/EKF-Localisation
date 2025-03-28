#include "map.h"
#include <random>   // for std::mt19937 and std::uniform_int_distribution

Map::Map() {
    // define map dimensions
    const int mapWidth = 120;
    const int mapHeight = 120;
    const int numLandmarks = 30;

    // define grid dimensions
    const int gridCols = 6;
    const int gridRows = 5;
    const int cellWidth = mapWidth / gridCols;
    const int cellHeight = mapHeight / gridRows;

    std::random_device rd;
    std::mt19937 gen(rd());

    // place one landmark in each cell
    int id = 1;
    for (int row = 0; row < gridRows; ++row) {
        for (int col = 0; col < gridCols; ++col) {
            if (id > numLandmarks) break;

            // define cell boundaries
            int cellXStart = col * cellWidth;
            int cellYStart = row * cellHeight;

            // create distributions for random x and y within the cell
            std::uniform_int_distribution<> disX(cellXStart, cellXStart + cellWidth - 1);
            std::uniform_int_distribution<> disY(cellYStart, cellYStart + cellHeight - 1);

            // generate random position within the cell
            int x = disX(gen);
            int y = disY(gen);


            landmarks[id++] = {x, y};
        }
    }
}

std::map<int, std::pair<int, int>> Map::getLandmarks() {
    return landmarks;
}
