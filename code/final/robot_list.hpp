#pragma once
#pragma GCC optimize("O2")
#include "robot.hpp"

class RobotList {
   public:
    std::vector<BaseRobot*> robots;
    std::queue<int> buy_robots_type;
    std::vector<int> private_id_to_public_id;

    RobotList() {}
    ~RobotList() {
        for (auto robot : robots) {
            delete robot;
        }
    }
    RobotList(const RobotList&) = delete;
    RobotList& operator=(const RobotList&) = delete;

    size_t get_size() { return robots.size(); }
    size_t get_private_size() { return private_id_to_public_id.size(); }
    size_t get_now_private_size() { return private_id_to_public_id.size() + buy_robots_type.size(); }
    void add_robot(int public_id, int good_num, Point p) { robots.push_back(new BaseRobot(public_id, p, good_num, 1)); }
    BaseRobot& get_base_robot(int public_id) { return *robots[public_id]; }
    Robot& get_our_robot_by_id(int public_id) { return *dynamic_cast<Robot*>(robots[public_id]); }
    Robot& get_our_robot(int private_id) {
        auto public_id = private_id_to_public_id[private_id];
        return *dynamic_cast<Robot*>(robots[public_id]);
    }
    std::vector<Robot*> get_our_robots() {
        std::vector<Robot*> res;
        for (auto public_id : private_id_to_public_id) {
            res.push_back(dynamic_cast<Robot*>(robots[public_id]));
        }
        return res;
    }
    void update_robot(Map& mp, int public_id, int good_num, Point p) {
        auto& robot = get_base_robot(public_id);
        robot.update(mp, public_id, good_num, p);
        if (check_our_robot(public_id)) {
            auto& robot_ = get_our_robot_by_id(public_id);
            if (robot_.nxt_good_num != good_num) {
                if (!robot_.is_pending()) {
                    std::cerr << frame_id << ": robot " << robot_.private_id << " good num not match\t"
                              << robot_.nxt_good_num << "<->" << good_num << std::endl;
                }
                robot_.nxt_good_num = good_num;
            }
        }
    }
    void match_good(Map& mp) {}
    void add_our_robot(int robot_type) { buy_robots_type.push(robot_type); }
    bool check_our_robot(int public_id) { return robots[public_id]->is_ours; }
    void update_our_robot(int public_id, int private_id) {
        if (buy_robots_type.empty()) {
            throw std::runtime_error("No robot to buy");
        }
        auto robot_type = buy_robots_type.front();
        buy_robots_type.pop();
        auto p = robots[public_id]->p;
        auto good_num = robots[public_id]->good_num;
        int robot_capacity = ROBOT_CAPACITY[robot_type];
        delete robots[public_id];
        robots[public_id] = new Robot(public_id, p, good_num, robot_capacity, private_id);
        if ((int)private_id_to_public_id.size() <= private_id) {
            private_id_to_public_id.resize(private_id + 1, -1);
        }
        private_id_to_public_id[private_id] = public_id;
    }
    void add_good(Map& mp, int good_id) {
        auto& good = mp.get_good(good_id);
        if (good.val < GOOD_VAL_SPLIT || !mp.check_patch(good.p))
            return;
        std::priority_queue<std::pair<int, int>> pq;
        for (auto robot_ptr : get_our_robots()) {
            auto& robot = *robot_ptr;
            if (robot.is_pending() || robot.is_disabled() || robot.good_num >= robot.capacity) {
                continue;
            }
            auto dis = mp.search_max_distance(robot.p, good.p);
            if (dis > SEARCH_GOOD_STEP) {
                continue;
            }
            pq.push({-dis, robot.private_id});
        }
        if (pq.empty()) {
            return;
        }
        auto [dis, private_id] = pq.top();
        dis = -dis;
        std::set<int> diss;
        for (Point& p : mp.robots_pos) {
            diss.insert(l1_dis(p, good.p));
            if (diss.size() > nearest_robots_num)
                diss.erase(--diss.end());
        }
        if (!diss.empty() && dis > *diss.rbegin())
            return;
        auto& robot = get_our_robot(private_id);
        if (robot.good_signal == -1 || dis < robot.good_distance) {
            robot.good_signal = good.id;
            robot.good_distance = dis;
        }
    }
    void update_map(Map& mp) {
        if (frame_id % 1000 == 0) {
            mp.our_robots_efficiency = mp.our_robots_efficiency * 0.5 + mp.our_robots_goods_val_1000 / 1000.0 * 0.5;
            mp.our_robots_num_efficiency = mp.our_robots_num_efficiency * 0.5 + mp.our_robots_goods_num_1000 / 1000.0 * 0.5;
            mp.our_robots_goods_val_1000 = 0;
            mp.our_robots_goods_num_1000 = 0;
        }
        mp.our_robots_goods_num = 0;
        mp.our_robots_goods_val = 0;
        for (const auto& it : get_our_robots()) {
            mp.our_robots_goods_num += it->good_num;
            mp.our_robots_goods_val += it->get_goods_val(mp);
        }
    }
};