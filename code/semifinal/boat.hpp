#pragma once
#pragma GCC optimize("O2")
#include "map.hpp"

class Boat {
   public:
    // 船只的id
    int id;
    // 船只要去的泊位的id
    int obj_berth_id{-1};
    // 船只要去的交货点id
    int obj_delivery_id{-1};
    // 船只的状态
    BoatStatus status{BoatStatus::MOVE};
    // 船只额外的状态
    BoatExtraStatus extra_status{BoatExtraStatus::GOTOBERTH_};
    // 船只当前的货物数量
    int good_num{0};
    int real_good_num{0};
    // 船只当前的货物价值
    int good_val{0};
    // 船只的货物容量
    int capacity{0};
    // 船只到达虚拟点的时间
    int arrive_time{0};
    // 船只核心点位置
    Point p;
    // 船只的朝向
    Direction dir{RIGHT};
    // 船只的规划
    BoatSchedule schedule;
    // 上次离开交货点或购买点的时间
    int last_time{1};
    // 避让恢复处
    ppd col_recover_ppd{{-1, -1}, STAY};

    Boat() {}
    Boat(int id, int capacity, Point p, int frame) : id(id), capacity(capacity), p(p), last_time(frame) {}

    // 每帧的操作
    int do_op(Map& mp, size_t robots_num) {
        // if (frame_id == 13030)
        //     std::cerr << frame_id << " " << good_num << " " << good_val << std::endl;
        // if (frame_id == 1)
        //     std::cerr << stay_until << " " << move_at_beginning << std::endl;
        if (frame_id == 2 && move_at_beginning >= 0) {
            obj_berth_id = move_at_beginning % mp.berth_num;
            schedule = mp.get_boat_schedule({p, dir}, {mp.berths[obj_berth_id].p, mp.berth_direction[obj_berth_id]});
        }
        if (frame_id <= stay_until)
            return 0;
        if (check_goto_delivery() && schedule.check_invalid() && check_recover())
            if (mp.check_delivery_point(p))
                unload(mp);
        if (good_num != real_good_num)
            std::cerr << frame_id << ": good_num != real_good_num " << good_num << " " << real_good_num << std::endl;
        if (check_recover())
            return 0;
        if (check_goto_berth() && check_move()) {
            // 到达泊位
            if (obj_berth_id != -1 && mp.get_berth(obj_berth_id).p == p && mp.berth_direction[obj_berth_id] == dir) {
                do_berth();
                auto& berth = mp.get_berth(obj_berth_id);
                mp.working_berths.insert(obj_berth_id);
                mp.berths[obj_berth_id].up_boat_data();
                if (berth.is_disabled()) {
                    mp.working_berths.erase(berth.id);
                    if (global_debug)
                        std::cerr << frame_id << ": go to disabled berth " << berth.id << std::endl;
                }
                schedule.clear();
                return 0;
            } else if (schedule.check_invalid()) {
                if (mp.check_boat_purchase_point(p))  // 购买点
                    get_plan(mp, robots_num);
                if (obj_berth_id != -1) {  // 购买点
                    schedule = mp.get_boat_schedule({p, dir}, {mp.berths[obj_berth_id].p, mp.berth_direction[obj_berth_id]});
                }
            }
            // else if (mp.check_near_berth_area(p) && mp.berth_near_area[p] == obj_berth_id) {
        }
        // 去交货点
        if (check_goto_delivery() && check_move() && schedule.check_invalid()) {
            if (mp.check_delivery_point(p)) {
                last_time = frame_id;
                get_plan(mp, robots_num);
                if (check_goto_berth())
                    schedule = mp.get_boat_schedule({p, dir}, {mp.berths[obj_berth_id].p, mp.berth_direction[obj_berth_id]});
                if (deliver_delay1)
                    return 0;
            } else {
                schedule = mp.get_boat_schedule({p, dir}, {mp.delivery_points[obj_delivery_id],
                                                           mp.berth_to_delivery_dir[{obj_berth_id, obj_delivery_id}]});
            }
        }
        // 如果是在运动的
        if (check_move() && !schedule.check_invalid()) {
            return 1;
        }
        // 如果装满了或没货了，去其他地方
        if (check_load()) {
            auto& berth = mp.get_berth(obj_berth_id);
            if ((quickly_buy == 2 || (quickly_buy == 1 && (check_full() || berth.is_empty()))) &&
                robots_num < robot_max_num && current_money + good_val >= robot_price) {
                do_dept();
                set_goto_delivery();
                mp.berths[obj_berth_id].set_idle();
                obj_delivery_id = berth.nearest_delivery_id;
                return 0;
            }
            // 优解搜索
            if (check_full() || berth.is_empty()) {
                int tag = get_plan(mp, robots_num);
                // 返回0代表去交货点或泊位，返回1代表停留
                if (tag == 0)
                    return 0;
            }
        }
        // 如果是在装货
        if (check_load()) {
            auto& berth = mp.get_berth(obj_berth_id);
            // fram_id+1出发，nearest_delivery_dis-1产生价值，最后1帧不产生价值
            if (frame_id + berth.nearest_delivery_dis + boat_col_time >= FRAME_NUM) {
                do_dept();
                set_goto_delivery();
                mp.berths[obj_berth_id].set_idle();
                obj_delivery_id = berth.nearest_delivery_id;
                if (global_debug)
                    std::cerr << frame_id << ": boat " << id << " no time no load" << std::endl;
                return 0;
            }
            berth.load_once(good_num, good_val, capacity);
        }
        return 0;
    }

    // 获取下一个目的地, 设置状态
    int get_plan(Map& mp, size_t robots_num) {
        double max_time_val = -1;
        double val_end_max = -1;
        int next_plan = -1;
        // int time_end = -1;
        std::vector<int> berths_num_minus;
        berths_num_minus.assign(mp.berths.size(), 0);
        std::vector<int> berths_val_minus;
        berths_val_minus.assign(mp.berths.size(), 0);
        int purchase_point_id = mp.get_boat_purchase_point_id(p);
        ppd from = {p, dir};
        // 去交货点(必须在泊位)
        if (!mp.check_delivery_point(p) && purchase_point_id == -1) {
            for (size_t i = 0; i < mp.delivery_points.size(); i++) {
                if (mp.berth_to_delivery_dir.count({obj_berth_id, i}) == 0)
                    continue;
                from = mp.get_berth(obj_berth_id).boat_ppd;
                ppd to = {mp.delivery_points[i], mp.berth_to_delivery_dir[{obj_berth_id, i}]};
                if (mp.boat_berths_dis[from].count(to) == 0 && global_debug)
                    std::cerr << frame_id << " err180" << std::endl;
                if (to.second == STAY)
                    std::cerr << frame_id << " " << i << " err53" << std::endl;
                int dis_tmp = mp.boat_berths_dis[from][to] + boat_col_time;
                double val_tmp = get_val_re(mp, i + const100000, good_num, good_num, good_val, good_val, frame_id + dis_tmp,
                                            1, berths_num_minus, berths_val_minus, 0, to.second);
                double val_end_tmp = get_val_re(mp, i + const100000, 0, 0, 0, 0, frame_id + dis_tmp, 0, berths_num_minus,
                                                berths_val_minus, 0, to.second);
                if (val_tmp > max_time_val) {
                    max_time_val = val_tmp;
                    next_plan = i + const100000;
                }
                if (val_end_tmp > val_end_max) {
                    val_end_max = val_end_tmp;
                    // time_end = frame_id + dis_tmp;
                }
                // if (frame_id > 14000 && good_num == 50)
                //     std::cerr << frame_id << " " << max_time_val << std::endl;
            }
        }
        // 去泊位
        for (size_t i = 0; i < mp.berths.size(); i++) {
            if ((check_load() && int(i) == obj_berth_id) || good_num >= capacity)
                continue;
            auto& target_berth = mp.get_berth(i);
            if (target_berth.is_busy())
                continue;
            int dis_tmp;
            ppd target = {target_berth.p, mp.berth_direction[target_berth.id]};
            // 如果是购买点
            if (purchase_point_id != -1 && mp.boat_purchase_dis[purchase_point_id].count(target)) {
                dis_tmp = mp.boat_purchase_dis[purchase_point_id][target];
                // if (frame_id == 2)
                //     std::cerr << frame_id << " " << dis_tmp << std::endl;
            }
            // 如果是交货点
            else if (mp.check_delivery_point(p) && mp.boat_delivery_dis[from].count(target)) {
                dis_tmp = mp.boat_delivery_dis[from][target];
            }
            // 如果是泊位
            else if (check_load() && mp.boat_berths_dis[mp.get_berth(obj_berth_id).boat_ppd].count(target)) {
                dis_tmp = mp.boat_berths_dis[mp.get_berth(obj_berth_id).boat_ppd][target];
            } else {
                // std::cerr << frame_id << " err210" << std::endl;
                continue;
            }
            int num_pred = target_berth.good_vals.size() +
                           round(target_berth.weighted_num_efficiency * dis_tmp * efficiency_decay);  // 往少了估
            if (num_pred == 0)
                continue;
            double val_pred = target_berth.val_sum + target_berth.weighted_efficiency * dis_tmp * efficiency_decay;
            int new_num_pred = std::min(capacity - good_num, num_pred);
            int new_val_pred = round(val_pred * new_num_pred / num_pred);
            dis_tmp += new_num_pred / target_berth.loading_speed + 2 + boat_col_time;  // 1帧berth，1帧dept
            berths_num_minus[i] += new_num_pred;
            berths_val_minus[i] += new_val_pred;
            double val_tmp = get_val_re(mp, i, good_num + new_num_pred, good_num + new_num_pred, good_val + new_val_pred,
                                        good_val + new_val_pred, frame_id + dis_tmp, 1, berths_num_minus, berths_val_minus);
            berths_num_minus[i] -= new_num_pred;
            berths_val_minus[i] -= new_val_pred;
            if (val_tmp > max_time_val) {
                max_time_val = val_tmp;
                next_plan = i;
            }
            // if (frame_id == 2)
            //     std::cerr << frame_id << " " << i << " " << val_tmp << " " << target_berth.get_goods_num() << " " <<
            //     target_berth.weighted_num_efficiency
            //               << " " << new_num_pred << " " << dis_tmp << " " << purchase_point_id << " "
            //               << target_berth.loading_speed << std::endl;
        }
        // if (frame_id > 11240 && frame_id < 11250) {
        //     int val =
        //         mp.boat_delivery_dis[{mp.delivery_points[0], mp.berth_to_delivery_dir[{2, 0}]}][{mp.berths[2].p, RIGHT}];
        //     ;
        //     int val2 =
        //         mp.boat_berths_dis[mp.get_berth(2).boat_ppd][{mp.delivery_points[0], mp.berth_to_delivery_dir[{2, 0}]}];
        //     int val3 = mp.boat_berths_dis[mp.get_berth(1).boat_ppd][{mp.berths[3].p, RIGHT}];
        //     std::cerr << frame_id << " " << val << " " << val2 <<" "<<val3<< std::endl;
        // }
        // if (frame_id == 2)
        //     std::cerr << frame_id << " " << next_plan << " " << max_time_val << std::endl;
        if (next_plan == -1) {
            if (frame_id > 1000 && global_debug)
                std::cerr << frame_id << " err211" << std::endl;
            return 0;
        }
        if (frame_id > 1000 && max_time_val <= 0 && global_debug)
            std::cerr << frame_id << ": boat " << id << " no time" << std::endl;
        // 开始的时候
        if (max_time_val == 0)
            return 1;
        if (next_plan >= const100000) {
            obj_delivery_id = next_plan - const100000;
            // 最后的时候
            if (frame_id > disable_start_time && val_end_max <= 0 /*&& time_end < FRAME_NUM*/ && !check_full()) {
                // std::cerr << frame_id << ": stay " << obj_berth_id << " " << time_end << std::endl;
                return 1;
            }
            set_goto_delivery();
            mp.berths[obj_berth_id].set_idle();
        } else {
            set_goto_berth();
        }
        if (check_load()) {
            if (mp.need_dept.count(obj_berth_id))
                do_dept();
            else {
                if (check_goto_delivery())
                    schedule = mp.get_boat_schedule({p, dir}, {mp.delivery_points[obj_delivery_id],
                                                               mp.berth_to_delivery_dir[{obj_berth_id, obj_delivery_id}]});
                else
                    schedule = mp.get_boat_schedule({p, dir}, {mp.berths[next_plan].p, mp.berth_direction[next_plan]});
                auto op = schedule.get_op();
                if (op == BoatOp::ANTICLOCKWISE)
                    do_anticlockwise_rotate();
                else if (op == BoatOp::CLOCKWISE)
                    do_clockwise_rotate();
                else if (op == BoatOp::FORWARD)
                    do_ship();
            }
        }
        if (check_goto_berth()) {
            mp.berths[obj_berth_id].set_idle();
            obj_berth_id = next_plan;
            mp.berths[obj_berth_id].set_busy();
        }
        return 0;
    }

    // 递归搜索，now表示到达泊位后或到达交货点前。现在大部分时候不进行递归
    double get_val_re(Map& mp,
                      int uni_id,
                      int num_now,
                      int num_all,
                      int val_now,
                      int val_all,
                      int now_time,
                      int depth,
                      std::vector<int> berths_num_minus,
                      std::vector<int> berths_val_minus,
                      int deliver_num = 0,  // 到达交货点的次数，最多recursive_del_num_max次
                      Direction del_dir = STAY) {
        double max_time_val = -1;
        // std::function<void(int)> log = [&](int l) {
        //     if (frame_id > 6620 && frame_id < 6630 && max_time_val > 0.092)
        //         std::cerr << l << " " << depth << " " << uni_id << " " << max_time_val << " " << num_all << " " <<
        //         now_time
        //                   << std::endl;
        // };
        std::function<double()> time_detection = [&]() {
            if (now_time > FRAME_NUM) {
                if (num_now == num_all) {
                    // 可以考虑减少泊位停留时间
                    if (uni_id >= const100000 && val_now > 0)
                        return -((now_time + 10.0 - FRAME_NUM) / (FRAME_NUM * 2));  // 垂死挣扎，-1到0
                    else
                        return -((now_time + 10.0) / (FRAME_NUM * 2));
                }
                val_all -= val_now;
                num_all -= num_now;
                if (using_val)
                    return double(val_all) / (FRAME_NUM - last_time);
                else
                    return double(num_all) / (FRAME_NUM - last_time);
            }
            return -10.0;
        };
        double ret = time_detection();
        if (ret != -10.0)
            return ret;
        if (depth >= recursive_depth_max) {
            if (uni_id < const100000) {
                auto& berth = mp.get_berth(uni_id);
                int delivery_id = berth.nearest_delivery_id;
                now_time += boat_dis_proportion *
                                mp.boat_berths_dis[berth.boat_ppd][{mp.delivery_points[delivery_id],
                                                                    mp.berth_to_delivery_dir[{uni_id, delivery_id}]}] +
                            (1 - boat_dis_proportion) * average_berth_dis_one;
                double ret = time_detection();
                if (ret != -10.0)
                    return ret;
            }
            // if (frame_id > 11240 && frame_id < 11250 && double(num_all) / (now_time - last_time) > 0.87)
            //     std::cerr << depth << " " << uni_id << " " << num_all << " " << now_time << " "
            //               << double(num_all) / (now_time - last_time) << std::endl;
            if (using_val)
                max_time_val = double(val_all) / (now_time - last_time);
            else
                max_time_val = double(num_all) / (now_time - last_time);
            // log(1);
            return max_time_val;
        }
        // 只有在泊位才会去交货点
        if (uni_id < const100000) {
            for (size_t i = 0; i < mp.delivery_points.size(); i++) {
                if (mp.berth_to_delivery_dir.count({uni_id, i}) == 0)
                    continue;
                ppd from = mp.get_berth(uni_id).boat_ppd;
                ppd to = {mp.delivery_points[i], mp.berth_to_delivery_dir[{uni_id, i}]};
                int dis_tmp = mp.boat_berths_dis[from][to];
                double val_tmp = get_val_re(mp, i + const100000, num_now, num_all, val_now, val_all, now_time + dis_tmp,
                                            depth + 1, berths_num_minus, berths_val_minus, deliver_num, to.second);
                if (val_tmp > max_time_val) {
                    max_time_val = val_tmp;
                }
            }
        }
        // 到交货点清空
        else {
            deliver_num++;
            if (using_val)
                max_time_val = std::max(max_time_val, double(val_all) / (now_time - last_time));
            else
                max_time_val = std::max(max_time_val, double(num_all) / (now_time - last_time));
            // log(0);
            if (deliver_num >= recursive_del_num_max)
                return max_time_val;
            num_now = 0;
            val_now = 0;
            now_time++;  // 可以优化
        }
        // 去泊位
        for (size_t i = 0; i < mp.berths.size(); i++) {
            if (int(i) == uni_id || num_now >= capacity)
                continue;
            auto& target_berth = mp.get_berth(i);
            int dis_tmp = -1;
            ppd target = {target_berth.p, mp.berth_direction[target_berth.id]};
            // 如果是交货点
            if (uni_id >= const100000) {
                ppd from = {mp.delivery_points[uni_id - const100000], del_dir};
                if (mp.boat_delivery_dis[from].count(target))
                    dis_tmp = mp.boat_delivery_dis[from][target];
            }
            // 如果是泊位
            else {
                ppd from = mp.get_berth(uni_id).boat_ppd;
                if (mp.boat_berths_dis[from].count(target))
                    dis_tmp = mp.boat_berths_dis[from][target];
            }
            if (dis_tmp == -1) {
                // std::cerr << frame_id << " " << depth << " " << uni_id << " " << i << " " << del_dir << " warning210"
                //           << std::endl;
                continue;
            }
            int gap_time = now_time + dis_tmp * efficiency_decay - frame_id;
            int num_pred =
                target_berth.good_vals.size() + round(target_berth.weighted_num_efficiency * gap_time) - berths_num_minus[i];
            if (num_pred == 0)
                continue;
            double val_pred = target_berth.val_sum + target_berth.weighted_efficiency * gap_time - berths_val_minus[i];
            int new_num_pred = std::min(capacity - num_now, num_pred);
            int new_val_pred = round(val_pred * new_num_pred / num_pred);
            dis_tmp += new_num_pred / target_berth.loading_speed + 2;  // 1帧berth，1帧dept
            berths_num_minus[i] += new_num_pred;
            berths_val_minus[i] += new_val_pred;
            double val_tmp = get_val_re(mp, i, num_now + new_num_pred, num_all + new_num_pred, val_now + new_val_pred,
                                        val_all + new_val_pred, now_time + dis_tmp, depth + 1, berths_num_minus,
                                        berths_val_minus, deliver_num);
            berths_num_minus[i] -= new_num_pred;
            berths_val_minus[i] -= new_val_pred;
            if (val_tmp > max_time_val) {
                max_time_val = val_tmp;
            }
        }
        // log(2);
        return max_time_val;
    }

    void update(int carried_goods_num, Point p, Direction dir, BoatStatus status) {
        this->real_good_num = carried_goods_num;
        this->p = p;
        this->dir = dir;
        this->status = status;
    }

    // 对货物的处理
    void load(int cnt, int val) {
        good_num += cnt;
        good_val += val;
    }
    void unload(Map& mp) {
        if (global_debug && (global_info || frame_id > 14800)) {
            int num_now = 0;
            for (Berth& berth : mp.berths) {
                num_now += berth.good_vals.size();
            }
            std::cerr << frame_id << ": boat " << id << " unload " << good_num << " " << good_val << " " << num_now
                      << std::endl;
        }
        final_score += good_val;
        delivered_goods_val += good_val;
        good_num = 0;
        good_val = 0;
    }
    // check
    inline bool check_full() { return good_num >= capacity; }
    inline bool check_status(BoatStatus status) { return this->status == status; }
    inline bool check_extra_status(BoatExtraStatus status) { return this->extra_status == status; }
    inline bool check_move() { return check_status(BoatStatus::MOVE); }
    inline bool check_recover() { return check_status(BoatStatus::RECOVER); }
    inline bool check_load() { return check_status(BoatStatus::LOAD); }
    inline bool check_waiting() { return check_extra_status(BoatExtraStatus::WAITING_); }
    inline bool check_goto_berth() { return check_extra_status(BoatExtraStatus::GOTOBERTH_); }
    inline bool check_goto_delivery() { return check_extra_status(BoatExtraStatus::GOTODELIVERY); }
    inline void set_goto_berth() { extra_status = BoatExtraStatus::GOTOBERTH_; }
    inline void set_goto_delivery() { extra_status = BoatExtraStatus::GOTODELIVERY; }
    inline void set_waiting() { extra_status = BoatExtraStatus::WAITING_; }
    // 船只的操作
    // 尝试将对应船位置重置到主航道上，会导致船进入恢复状态。
    void do_dept() { printf("dept %d\n", id); }
    // 尝试将对应船靠泊到泊位上，会导致船进入恢复状态。
    void do_berth() { printf("berth %d\n", id); }
    // 旋转，0为顺时针旋转。1为逆时针旋转。
    void do_rotate(int d) { printf("rot %d %d\n", id, d); }
    void do_clockwise_rotate() { do_rotate(0); }
    void do_anticlockwise_rotate() { do_rotate(1); }
    // 向正方向前进1格（主航道）
    void do_ship() { printf("ship %d\n", id); }
};