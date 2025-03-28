#ifndef MAP_H
#define MAP_H

#include <map>

class Map {
    public:
        Map();

        std::map<int, std::pair<int, int>> getLandmarks();

    private:
        std::map<int, std::pair<int, int>> landmarks;
};

#endif