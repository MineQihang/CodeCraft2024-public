#pragma once
#pragma GCC optimize("O2")
#include "utils.hpp"

int pulled_goods_num, pulled_goods_val;

class Robot {
   public:
    Point p;
    Point next_p[2];
    Point target;
    int bot_target_berth{-1};
    bool has_target;
    char going;  // g:good b:berth r:边去berth边搜
    int id;
    // int good_id{-1};
    int has_good;
    int target_dis{0x3f3f3f3f};
    Good good;
    Good cache_good;
    RobotStatus status;
    int scale_search{60};
    Schedule schedule;
    Schedule cache_schedule;
    bool schedule_need_pop;
    int has_col = 0;
    Robot() {}
    Robot(int x, int y, int has_good, RobotStatus status) : p(x, y), has_good(has_good), status(status) {}
    void set_id(int id) { this->id = id; }
    void update() {
        int temp = 0;
        scanf("%d%d%d%d", &has_good, &p.x, &p.y, &temp);
        this->status = RobotStatus(temp);
    }

    void do_op(Map& mp, std::map<int, Direction>& move_commands, int frame_id) {
        move_commands[id] = STAY;
        Direction next_commands = STAY;
        next_p[0] = p;
        next_p[1] = p;
        schedule_need_pop = 0;
        if (!mp.robots_available[id])
            return;
        bool debug = 0;
        int debug_start_frame = 11643;
        int debug_end_frame = 11643;
        int debug_id = 0;
        if (debug && frame_id >= debug_start_frame && frame_id <= debug_end_frame && id == debug_id)
            std::cerr << "\n"
                      << "here45 " << frame_id << " target:" << target.x << " " << target.y << " now_p:" << p.x << " " << p.y
                      << "  has_good:" << has_good << std::endl;
        if (debug && frame_id >= debug_start_frame && frame_id <= debug_end_frame && id == debug_id) {
            std::cerr << frame_id << " " << going << " has_target:" << has_target << " on_berth:" << mp.check_berth(p)
                      << " ch:" << mp.ch[123][144] << std::endl;
        }
        if (is_recover()) {
            has_col = 1;
            mp.robots_map[1][next_p[0].x][next_p[0].y] = id;
            mp.robots_map[2][next_p[1].x][next_p[1].y] = id;
            // schedule.ops.clear();
            return;
        }
        if (has_col) {
            std::cerr << id << " collision at frame " << frame_id - 20 << std::endl;
            schedule.recover_op();
            has_col = 0;
        }
        // if(frame_id==256&&id>2) return;

        // if(frame_id>=debug_start_frame&&frame_id<=debug_start_frame&&id==debug_id)std::cerr<<"\n"<<"here53 "<<frame_id<<"
        // "<<target.x<<" "<<target.y<<std::endl; if(frame_id>=debug_start_frame&&frame_id<=debug_start_frame&&id==debug_id)
        // {std::cerr<<frame_id<<" "<<going<<" "<<has_target<<std::endl;} if(frame_id==272&&id==2) {std::cerr<<"okk2
        // "<<going<<std::endl; return;} 可以分移动前，移动，移动后(我没有)
        if (!has_good && has_target && p == target && going == 'g') {
            if ((!oringal_test && map_name != "") ||
                (oringal_test && map_cnt_up.count(map_name) && map_cnt < map_cnt_up[map_name])) {
                printf("get %d\n", id);
                map_cnt++;
            }
            if (mp.goods.goods[p].fade_time > frame_id) {
                good = mp.goods.goods[p];
                mp.goods.delete_good(p);
            }
            // target = mp.get_nearest_berth_point(p.x, p.y);  // 可调参
            if (!fixed_berth)
                target = mp.get_nearest_able_berth_point(p.x, p.y, frame_id);
            else
                target = mp.berths[init_robot_plan[map_name][id]].p1;
            mp.target_berth[id] = mp.berths_map[target];
            has_target = 1;
            has_good = 1;
            going = 'b';
            // mp.robot_col_valid[id] = 0;
            schedule.ops.clear();
        }
        if (!has_good && has_target && going == 'f' && mp.goods.goods.count(p)) {
            if ((!oringal_test && map_name != "") ||
                (oringal_test && map_cnt_up.count(map_name) && map_cnt < map_cnt_up[map_name])) {
                printf("get %d\n", id);
                map_cnt++;
            }
            Good& oh_good = mp.goods.goods[p];
            if (oh_good.fade_time <= frame_id)
                std::cerr << "err273 ";
            oh_good.booked = 1;
            good = oh_good;
            mp.goods.delete_good(p);
            has_good = 1;
        }

        if (mp.check_berth(p) && has_target && (going == 'b' || going == 'f')) {
            int berth_id = mp.berths_map[p];
            int target_id = mp.point_to_berth_idx(target) / 16;
            if (berth_id == target_id) {
                if (has_good) {
                    printf("pull %d\n", id);
                    mp.berths[berth_id].add_good(good.val);
                    has_good = 0;
                    pulled_goods_num++;
                    pulled_goods_val += good.val;
                }
                has_target = 0;
            }
        }
        if (!has_good && !has_target && mp.check_berth(p)) {
            if (cache_schedule.is_valid() && p == target) {
                schedule = cache_schedule;
                target = cache_good.p;
            } else {
                auto [new_good, new_schedule] = mp.search_good(p, scale_search, frame_id, id);
                schedule = new_schedule;
                if (schedule.is_valid())
                    target = new_good.p;
            }
            if (schedule.is_valid()) {
                has_target = 1;
                going = 'g';
            }
            cache_schedule.ops.clear();
        }
        if (debug && frame_id >= debug_start_frame && frame_id <= debug_end_frame && id == debug_id)
            std::cerr << "\n"
                      << "here99 " << frame_id << " target:" << target.x << " " << target.y << "  has_good:" << has_good
                      << std::endl;
        if (debug && frame_id >= debug_start_frame && frame_id <= debug_end_frame && id == debug_id) {
            std::cerr << frame_id << " " << going << " has_target:" << has_target << " on_berth:" << mp.check_berth(p)
                      << std::endl;
        }
        if (!has_good && !has_target && !mp.check_berth(p)) {
            if (!manual_init_robot)
                target = mp.get_nearest_berth_point(p.x, p.y);
            else
                target = mp.berths[init_robot_plan[map_name][id]].p1;
            has_target = 1;
            if (!select_nearby_at_beginning)
                going = 'b';
            else
                going = 'f';  // 第1次去泊位
            mp.target_berth[id] = mp.berths_map[target];
            // if(!(target== mp.berths[mp.berths_map[target]].p)) std::cerr<<"err34 "<<target.x<<" "<<target.y<<std::endl;
        }

        if (has_target) {
            if (going == 'f') {
                int berth_idx = mp.point_to_berth_idx(target);
                if (!mp.berths_path_valid[0][berth_idx]) {
                    mp.init_berth(berth_idx, 0);
                    mp.berths_path_valid[0][berth_idx] = 1;
                }
                if (!has_good)
                    move_commands[id] = mp.get_nearby_good_dir(p, first_search_scale);
                // target_dis = mp.berths_distance[berth_idx][p.x][p.y];
                if (move_commands[id] == STAY)
                    move_commands[id] = mp.berths_path[0][berth_idx][p.x][p.y];
                next_p[0] = p + move_commands[id];
                if (target != next_p[0])
                    next_commands = mp.berths_path[0][berth_idx][next_p[0].x][next_p[0].y];  // 懒得改
                else {
                    next_commands = reverse_dir(move_commands[id]);  // 近似处理和不处理都不影响分数
                }
            }
            if (going == 'b') {
                if (mp.berth_disabled[mp.berths_map[target]]) {
                    target = mp.get_nearest_berth_point(p.x, p.y);
                    mp.target_berth[id] = mp.berths_map[target];
                }
                int berth_idx = mp.point_to_berth_idx(target);
                if (!mp.berths_path_valid[0][berth_idx]) {
                    mp.init_berth(berth_idx, 0);
                    mp.berths_path_valid[0][berth_idx] = 1;
                }
                // target_dis = mp.berths_distance[berth_idx][p.x][p.y];
                move_commands[id] = mp.berths_path[0][berth_idx][p.x][p.y];
                next_p[0] = p + move_commands[id];
                if (target != next_p[0])
                    next_commands = mp.berths_path[0][berth_idx][next_p[0].x][next_p[0].y];
                else {
                    // auto [new_good, new_schedule] = mp.search_good(target, scale_search, frame_id, id);
                    // cache_schedule = new_schedule;
                    // cache_good = new_good;
                    // if (new_schedule.is_valid())
                    //     next_commands = new_schedule.get_op(false);
                    next_commands = reverse_dir(move_commands[id]);  // 近似处理和不处理都不影响分数
                }
            }
            if (going == 'g') {
                if (!schedule.is_valid()) {
                    std::cerr << "err91 ";
                } else {
                    move_commands[id] = schedule.get_op();
                    next_p[0] = p + move_commands[id];
                    int berth_idx = mp.point_to_berth_idx(mp.get_nearest_berth_point(target.x, target.y));
                    target_dis = abs(mp.berths_distance[berth_idx][p.x][p.y] -
                                     mp.berths_distance[berth_idx][target.x][target.y]);  // 不对
                    // if(p.x==137 && p.y==87)std::cerr<<frame_id<<" "<<target.x<<" "<<target.y<<"
                    // "<<mp.point_to_berth_idx(target)<<" "<<(int)move_commands[id]<<std::endl;
                    if (move_commands[id] < 0 || move_commands[id] > 4) {
                        std::cerr << "err34 frame " << frame_id << " move " << id << " " << int(move_commands[id])
                                  << std::endl;
                    }
                    if (schedule.is_valid())
                        next_commands = schedule.get_op(false);
                    else {
                        next_commands = mp.berths_path[0][berth_idx][target.x][target.y];
                        if (target != next_p[0])
                            std::cerr << "err756" << std::endl;
                    }
                }
            }
        } else if (mp.check_berth(p)) {
            int berth_id = mp.berths_map[p];
            Point berth_p1 = mp.berths[berth_id].p;
            berth_p1.x += 1;
            berth_p1.y += 1;
            Point berth_p2 = berth_p1;
            berth_p2.x += 1;
            berth_p2.y += 1;
            if (debug && frame_id == debug_start_frame && id == debug_id)
                std::cerr << " berth_p1:" << berth_p1.x << " " << berth_p1.y << " " << std::endl;
            if (p == berth_p1 || p == berth_p2) {
                going = 'b';
            } else if (going != 'r') {
                if (mp.robots_map[1][berth_p1.x][berth_p1.y] < 0 && mp.robots_map[2][berth_p1.x][berth_p1.y] < 0) {
                    if (debug && frame_id == debug_start_frame && id == debug_id)
                        std::cerr << "okk berth_p1:" << berth_p1.x << " " << berth_p1.y << " " << std::endl;
                    going = 'r';
                    target = berth_p1;
                } else if (mp.robots_map[1][berth_p2.x][berth_p2.y] < 0 && mp.robots_map[2][berth_p2.x][berth_p2.y] < 0) {
                    int berth_idx = mp.point_to_berth_idx(berth_p2);
                    if (!mp.berths_path_valid[0][berth_idx]) {
                        mp.init_berth(berth_idx, 0);
                        mp.berths_path_valid[0][berth_idx] = 1;
                    }
                    going = 'r';
                    target = berth_p2;
                } else {
                }
            }
        }
        if (going == 'r') {
            int berth_idx = mp.point_to_berth_idx(target);
            move_commands[id] = mp.berths_path[0][berth_idx][p.x][p.y];
            if (debug && frame_id == debug_start_frame && id == debug_id)
                std::cerr << " target:" << target.x << " " << target.y << " " << mp.berths_path_valid[0][berth_idx]
                          << " commmand:" << (int)move_commands[id] << std::endl;
            next_p[0] = p + move_commands[id];
            next_commands = mp.berths_path[0][berth_idx][next_p[0].x][next_p[0].y];
        }
        // if(debug&&frame_id==3086&&id==2)std::cerr<<"2:"<<target_dis<<std::endl;
        // if(debug&&frame_id==3086&&id==7)std::cerr<<"7:"<<target_dis<<std::endl;
        // if(frame_id>254&&id==2&&frame_id<=272) {std::cerr<<"here152 "<<frame_id<<" "<<going<<" "<<has_target<<std::endl;}
        mp.robots_map[1][next_p[0].x][next_p[0].y] = id;
        next_p[1] = next_p[0] + next_commands;
        mp.robots_map[2][next_p[1].x][next_p[1].y] = id;
    }
    Point next_pos(int time) {
        if (schedule.is_valid()) {  // TODO: time
            return p + schedule.get_op(false);
        } else {
            return p;
        }
    }
    bool is_valid() { return status == RobotStatus::NORMAL; }
    bool is_recover() { return status == RobotStatus::RECOVER; }
};