#pragma once
#pragma GCC optimize("O2")
#include "map.hpp"
#include "utils.hpp"

enum RobotExtraStatus { GOTOBERTH = 0, GOTOGOOD = 1, GOTOPATCH = 2, WAITING = 3, DISABLED = 4, PENDING = 5 };

class BaseRobot {
   public:
    virtual ~BaseRobot() {}
    // 机器人的id
    int public_id{-1};
    // 机器人的位置
    Point p;
    // 当前机器人携带货物的数量
    int good_num{0};
    // 机器人的容量
    int capacity{1};
    // 机器人携带的货物id
    std::stack<int> good_ids{};
    // 机器人是否是我们的
    bool is_ours{false};
    // 停留时间
    int stay_time{0};

    BaseRobot() {}
    BaseRobot(int public_id, Point p, int good_num, int capacity, bool is_ours = false)
        : public_id(public_id), p(p), good_num(good_num), capacity(capacity), is_ours(is_ours) {}
    BaseRobot(const BaseRobot&) = delete;
    BaseRobot& operator=(const BaseRobot&) = delete;

    void update(Map& mp, int public_id, int good_num, Point p) {
        this->public_id = public_id;
        // 主要是记录机器人携带货物的变化
        // 变少了，说明放到了泊位上
        if (this->good_num > good_num) {
            Berth* berth;
            if (mp.check_berth(this->p)) {
                berth = &mp.get_land_nearest_berth(this->p);
            } else {
                berth = &mp.get_land_nearest_berth(p);
            }
            while (!good_ids.empty()) {
                int good_id = good_ids.top();
                good_ids.pop();
                auto& good = mp.get_good(good_id);
                berth->add_good(good.val);
            }
            // 变多了，说明捡起了货物
        } else if (this->good_num < good_num) {
            // TODO: 可能会存在消失货物的bug
            // 在之前的位置捡了之后移动的
            bool flag = false;
            if (mp.check_good(this->p)) {
                auto& good = mp.get_good(this->p);
                if (mp.cache_goods.count(good.id)) {
                    good_ids.push(good.id);
                    if (is_ours) {
                        mp.our_robots_goods_val_1000 += good.val;
                        mp.our_robots_goods_num_1000++;
                    }
                    flag = true;
                }
            }
            // 先移动再捡的
            if (!flag && mp.check_good(p)) {
                auto& good = mp.get_good(p);
                if (mp.cache_goods.count(good.id)) {
                    good_ids.push(good.id);
                    if (is_ours) {
                        mp.our_robots_goods_val_1000 += good.val;
                        mp.our_robots_goods_num_1000++;
                    }
                }
            }
            if (good_num > capacity)
                capacity = good_num;
            mp.working_robots[public_id] = frame_id;
        } else if (mp.check_good(this->p)) {
            // TODO: 怎么统计嗷嗷
            auto& good = mp.get_good(this->p);
            if (p == this->p) {
                if (good.val > GOOD_VAL_SPLIT) {
                    good.set_pending();
                    // if (mp.precious_goods.count(good.id))
                    //     mp.precious_goods.erase(good.id);
                }
            } else {
                // TODO: 注意别人的机器人如果没有答题成功，还可以继续去
                // 但是自己的机器人答题成功了就不能再去了
                // 别人答题成功 || 自己答题失败 置为fade，前者已经在前面处理了
                if (good.is_pending()) {
                    if (!is_ours) {
                        good.set_exist();
                        // mp.precious_goods.insert(good.id);
                    } else {
                        good.set_fade();
                        // if (mp.precious_goods.count(good.id))
                        //     mp.precious_goods.erase(good.id);
                        mp.get_patch(this->p).high_goods_num--;
                    }
                }
            }
        }
        if (this->p == p) {
            stay_time++;
        } else {
            stay_time = 0;
        }
        this->p = p;
        this->good_num = good_num;
        // 更新地图的patch信息
        if (mp.map_split.count(p)) {
            auto& patch = mp.get_patch(p);
            patch.robot_num += (good_num == 0);
            patch.now_our_robot_num += (good_num < capacity && is_ours);
        }
    }
};