#pragma once
#pragma GCC optimize("O2")
#include "boat_pos.hpp"
#include "map.hpp"
#include "utils.hpp"

enum BoatStatus { MOVE = 0, RECOVER = 1, LOAD = 2 };
enum BoatExtraStatus { WAITING_ = 0, GOTOBERTH_ = 1, GOTODELIVERY = 2 };

class BaseBoat {
   public:
    virtual ~BaseBoat() {}
    // 船只的id
    int public_id{-1};
    // 船只当前的货物数量
    int good_num{0};
    // 船只核心点位置
    Point p{};
    // 船只的朝向
    Direction dir{RIGHT};
    // 船只的状态
    BoatStatus status{BoatStatus::MOVE};
    // 船只当前的货物价值
    int good_val{0};
    // 船只的货物容量
    int capacity{0};
    // 是否是我们的
    bool is_ours{false};
    int load_cnt{0};

    BaseBoat() {}
    BaseBoat(int id, int good_num, Point p, Direction dir, BoatStatus status, int capacity)
        : public_id(id), good_num(good_num), p(p), dir(dir), status(status), capacity(capacity) {}
    BaseBoat(const BaseBoat&) = delete;
    BaseBoat& operator=(const BaseBoat&) = delete;

    void update(Map& mp, int id, int good_num, Point p, Direction dir, BoatStatus status) {
        this->public_id = id;
        // 主要是记录船只携带货物的变化
        // 变少了，说明放到了交货点上
        if (this->good_num > good_num) {
            this->good_val = 0;
            // 变多了，说明从泊位上取货
        } else if (this->good_num < good_num) {
            auto& berth = mp.get_ocean_nearest_berth(p);
            berth.set_busy();
            berth.load_once(this->good_num, this->good_val, this->capacity);
            mp.working_boats[id] = frame_id;
            if (this->good_num != good_num) {
                std::cerr << "maybe some bug in boat update" << std::endl;
            }
            // 判断是否离开了泊位
        } else if (this->status == LOAD && (status == MOVE || status == RECOVER)) {
            auto& berth = mp.get_ocean_nearest_berth(this->p);
            berth.current_boat_id = -1;
            berth.is_ours_boat = false;
            berth.set_idle();
        }
        if ((mp.check_near_berth_area(p) || mp.check_berth(p)) && this->p == p && status != LOAD && good_num < capacity){
            auto& berth = mp.get_ocean_nearest_berth(p);
            berth.near_boat_ids.insert(id);
        }
        this->good_num = good_num;
        this->p = p;
        this->dir = dir;
        this->status = status;
        if (this->status == LOAD) {
            auto& berth = mp.get_ocean_nearest_berth(this->p);
            berth.current_boat_id = public_id;
            berth.is_ours_boat = is_ours;
            load_cnt++;
            if (load_cnt > occupy_time)
                mp.occupied_berths.insert(berth.id);
        } else {
            if (load_cnt > occupy_time) {
                int berth_id = mp.get_ocean_nearest_berth(this->p).id;
                if (mp.occupied_berths.count(berth_id))
                    mp.occupied_berths.erase(berth_id);
                else
                    std::cerr << "err3413" << std::endl;
            }
            load_cnt = 0;
        }
    }
    // check
    inline bool check_full() { return good_num >= capacity; }
    inline bool check_status(BoatStatus status) { return this->status == status; }
    inline bool check_move() { return check_status(BoatStatus::MOVE); }
    inline bool check_recover() { return check_status(BoatStatus::RECOVER); }
    inline bool check_load() { return check_status(BoatStatus::LOAD); }
};