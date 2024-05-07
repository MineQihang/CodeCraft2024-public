#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"

class Good {
   public:
    int id{-1};
    Point p;
    int val;
    int fade_time;
    int booked = 0;
    Good() {}
    Good(int x, int y, int val, int fade_time) : p(x, y), val(val), fade_time(fade_time) {
        static int good_id = 0;
        id = good_id++;
    }
};

class GoodList {
   public:
    std::vector<Good> goods;
    int add(int x, int y, int val, int fade_time) {
        goods.push_back(Good(x, y, val, fade_time));
        return goods.size() - 1;
    }
    void remove(int id) {
        goods[id] = goods.back();
        goods.pop_back();
    }
    Good get(int id) { return goods[id]; }
};

class GoodMap {
   public:
    std::unordered_map<Point, Good> goods;
    void add(int x, int y, int val, int fade_time) { goods.emplace(Point(x, y), Good(x, y, val, fade_time)); }
    void delete_good(Point p) { goods.erase(p); }
};