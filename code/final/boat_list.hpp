#pragma once
#pragma GCC optimize("O2")
#include "boat.hpp"

class BoatList {
   public:
    std::vector<BaseBoat*> boats;
    std::queue<int> buy_boats_type;
    std::vector<int> private_id_to_id;

    BoatList() {}
    ~BoatList() {
        for (auto boat : boats) {
            delete boat;
        }
    }
    BoatList(const BoatList&) = delete;
    BoatList& operator=(const BoatList&) = delete;

    size_t get_size() { return boats.size(); }
    size_t get_private_size() { return private_id_to_id.size(); }
    void add_boat(int id, int good_num, Point p, Direction dir, BoatStatus status, int capacity) {
        boats.push_back(new BaseBoat(id, good_num, p, dir, status, capacity));
    }
    BaseBoat& get_base_boat(int id) { return *boats[id]; }
    Boat& get_our_boat(int private_id) {
        auto id = private_id_to_id[private_id];
        return *dynamic_cast<Boat*>(boats[id]);
    }
    std::vector<Boat*> get_our_boats() {
        std::vector<Boat*> res;
        for (auto id : private_id_to_id) {
            res.push_back(dynamic_cast<Boat*>(boats[id]));
        }
        return res;
    }
    void update_boat(Map& mp, int id, int good_num, Point p, Direction dir, BoatStatus status) {
        auto& boat = get_base_boat(id);
        boat.update(mp, id, good_num, p, dir, status);
    }
    void add_our_boat(int boat_type = 0) { buy_boats_type.push(boat_type); }
    bool check_our_boat(int id) { return boats[id]->is_ours; }
    void update_our_boat(int id, int private_id) {
        if (buy_boats_type.empty()) {
            throw std::runtime_error("No boat to buy");
        }
        // auto boat_type = buy_boats_type.front();
        buy_boats_type.pop();
        auto temp = boats[id];
        boats[id] = new Boat(temp, private_id);
        boats[id]->is_ours = true;
        delete temp;
        if ((int)private_id_to_id.size() <= private_id) {
            private_id_to_id.resize(private_id + 1, -1);
        }
        private_id_to_id[private_id] = id;
    }
    void update_map(Map& mp) {
        // 最近船只
        timer.start();
        for (int i = 0; i < (int)mp.berths.size(); i++) {
            mp.nearest_boat[i] = -1;
            int min_dis = max_int_;
            for (int j = 0; j < (int)boats.size(); j++) {
                auto& boat = *boats[j];
                if (boat.good_num >= boat.capacity || (boat.check_load() && (!boat_real_time_decide || !boat.is_ours)))
                    continue;
                int dis = mp.berths_ocean_distance[i][boat.p.x][boat.p.y];
                if (dis < min_dis) {
                    min_dis = dis;
                    mp.nearest_boat[i] = j;
                }
            }
        }
        timer.print_duration_with_reset("nearest_boat", global_debug, LOG_TIME_THRESHOLD * 0.2);
        // 某泊位最近的我方船
        for (int i = 0; i < (int)mp.berths.size(); i++) {
            mp.berths[i].nearest_our_boat_id = -1;
            mp.berths[i].nearest_our_going_boat_id = -1;
            int min_dis1 = max_int_, min_dis2 = max_int_;
            for (int j = 0; j < (int)get_private_size(); j++) {
                auto& boat = get_our_boat(j);
                if (boat.good_num >= boat.capacity)
                    continue;
                int dis = mp.berths_ocean_distance[i][boat.p.x][boat.p.y];
                if (dis < min_dis1) {
                    min_dis1 = dis;
                    mp.berths[i].nearest_our_boat_id = j;
                }
                if (boat.check_load() || !(boat.check_goto_berth() && boat.obj_berth_id == i))
                    continue;
                if (dis < min_dis2) {
                    min_dis2 = dis;
                    mp.berths[i].nearest_our_going_boat_id = j;
                }
            }
        }
        mp.our_loading_boats_num = 0;
        for (int j = 0; j < (int)get_private_size(); j++) {
            if (get_our_boat(j).check_load())
                mp.our_loading_boats_num++;
        }
    }
};