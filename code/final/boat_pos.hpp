#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"


using ppd = std::pair<Point, Direction>;
struct ppd_hash {
    std::size_t operator()(const ppd& p) const {
        std::size_t h1 = std::hash<Point>{}(p.first);
        std::size_t h2 = p.second;
        return h1 * 5 + h2;  // 注意Direction最大值是5（也可以是4）
    }
};

const ppd default_ppd = {{-1, -1}, STAY};

using pip = std::pair<int, ppd>;
using pii = std::pair<int, int>;

// 想要不疯狂旋转需要将比较函数改为std::greater<pip>
struct ComparePip {
    bool operator()(const pip& a, const pip& b) const { return a.first > b.first; }
};

ppd next_ppd(ppd now, BoatOp op) {
    if (op == FORWARD) {
        now.first += now.second;
    } else if (op == CLOCKWISE) {
        now.first = now.first + now.second + now.second;
        now.second = clockwise_dir(now.second);
    } else if (op == ANTICLOCKWISE) {
        now.first = now.first + now.second + clockwise_dir(now.second);
        now.second = anticlockwise_dir(now.second);
    }
    return now;
}

ppd next_ppd_rev(ppd now, BoatOp op) {
    if (op == FORWARD) {
        now.first += reverse_dir(now.second);
    } else if (op == CLOCKWISE) {
        now.second = anticlockwise_dir(now.second);
        now.first = now.first + reverse_dir(now.second) + reverse_dir(now.second);
    } else if (op == ANTICLOCKWISE) {
        now.second = clockwise_dir(now.second);
        now.first = now.first + reverse_dir(now.second) + anticlockwise_dir(now.second);
    }
    return now;
}