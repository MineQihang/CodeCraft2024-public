#pragma GCC optimize("O2")
#include "berth.hpp"
#include "boat_list.hpp"
#include "map.hpp"
#include "robot_list.hpp"
#include "scheduler.hpp"
#include "utils.hpp"

using namespace std;

RobotList robots;
BoatList boats;
Map mp(MAP_SIZE);
// DEBUG
int skip_frames, all_goods_val_sum, all_goods_num;
int robots_money_sum;
std::chrono::system_clock::time_point frame_start_time;
double time_sum;
std::map<int, Direction> move_commands;
std::vector<int> boat_goods_num;

pii decide_buy_what();
void buy_boat(int);
void buy_robot(int);
void final_log();

void init() {
    timer.start();
    // 输入地图和泊位
    mp.input();
    // 初始化船只容积
    scanf("%d", &boat_capacity);
    scanf("%d", &boat_capacity1);
    get_ok();
    // 预处理
    mp.preprocess();
#ifdef DEBUG
    cerr << "robot_max_num:" << robot_max_num << "\tboat_capacity:" << boat_capacity << "\tmap_name:" << map_name << endl;
#endif  // DEBUG
    put_ok();
}

void interaction_input() {
    scanf("%d%d", &frame_id, &current_money);
    if (global_debug && frame_id == 1)
        timer.print_duration_with_reset("frame 1", global_debug);
    if (global_debug && frame_id % 100 == 0)
        cerr << "[" << p_id << " " << frame_id << "] ";
    if (global_debug && frame_id % 1000 == 0)
        cerr << "current_money:" << current_money << endl;
    timer.start();
    // 更新patch的信息
    mp.update_patch();
    for (auto& berth : mp.berths) {
        berth.near_boat_ids.clear();
    }
    // 改变的货物
    goods_get_val_sum.push_back(0);
    if (goods_get_val_sum.size() > 1000)
        goods_get_val_sum.pop_front();
    int change_good_num;
    scanf("%d", &change_good_num);
    for (int i = 0; i < change_good_num; i++) {
        int x, y, val;
        scanf("%d%d%d", &x, &y, &val);
        if (val > 0) {  // 新增的
            auto fade_time = val > 1000 ? VALUED_FADE_TIME : FADE_TIME;
            mp.add_good({x, y}, val, frame_id + fade_time);
            robots.add_good(mp, mp.goods.back().id);
#ifdef DEBUG
            all_goods_val_sum += val;
            all_goods_num++;
#endif                          // DEBUG
        } else if (val <= 0) {  // 消失/拿走的
            mp.remove_good_cached({x, y});
        }
    }
    // 所有机器人
    int robots_num;
    scanf("%d", &robots_num);
    mp.robots_map.clear();
    mp.robots_num.clear();
    mp.robots_pos.clear();
    int now_robots_num = robots.get_size();
    for (int i = 0; i < robots_num; i++) {
        int id, good_num, x, y;
        scanf("%d%d%d%d", &id, &good_num, &x, &y);
        if (id >= now_robots_num) {
            robots.add_robot(id, good_num, {x, y});
        } else {
            if (robots.get_base_robot(id).is_ours) {
                robots.get_our_robot_by_id(id).check_ans_result(good_num, mp);
            }
            robots.update_robot(mp, id, good_num, {x, y});
        }
        mp.robots_map[{x, y}] = id;
        mp.robots_num[{x, y}]++;
        if (good_num >= robots.get_base_robot(id).capacity)
            mp.robots_pos.push_back({x, y});
    }
    // 所有船只
    mp.boats_map.clear();
    int boats_num;
    scanf("%d", &boats_num);
    boat_goods_num.resize(boats_num);
    for (int i = 0; i < boats_num; i++) {
        int id, goods_num, x, y, dir, status;
        scanf("%d%d%d%d%d%d", &id, &goods_num, &x, &y, &dir, &status);
        boat_goods_num[i] = goods_num;
        if (id >= (int)boats.get_size()) {
            boats.add_boat(id, goods_num, {x, y}, (Direction)dir, (BoatStatus)status, boat_capacity);
        } else {
            boats.update_boat(mp, id, goods_num, {x, y}, (Direction)dir, (BoatStatus)status);
        }
        bool is_ours = boats.check_our_boat(id);
        vector<Point> boat_points = mp.get_boat_points(ppd{{x, y}, Direction(dir)});
        for (auto& point : boat_points) {
            if (mp.check_ocean_main(point))
                continue;
            if (mp.boats_map.count(point) == 0)
                mp.boats_map[point] = id;
            else if (is_ours)
                mp.boats_map[point] = id;
        }
    }
    // 己方机器人
    int our_robots_num;
    scanf("%d", &our_robots_num);
    for (int i = 0; i < our_robots_num; i++) {
        int id;
        scanf("%d", &id);
        if (!robots.check_our_robot(id)) {
            robots.update_our_robot(id, i);
        }
    }
    // 贵重物品答题机器人
    int pending_robot_nums;
    scanf("%d", &pending_robot_nums);
    for (int i = 0; i < pending_robot_nums; i++) {
        char robot_id[10];
        scanf("%s", robot_id);
        int private_id = atoi(robot_id);
        std::string question;
        std::getline(std::cin, question);
        question = question.substr(1);  // 去除前面的空格
        auto& robot = robots.get_our_robot(private_id);
        robot.set_question(question);
        robot.set_pending();
    }
    // 己方船只
    int our_boat_num;
    scanf("%d", &our_boat_num);
    for (int i = 0; i < our_boat_num; i++) {
        int id;
        scanf("%d", &id);
        if (liuyishou && i >= boat_num_max)
            continue;
        if (p_id == -1) {
            p_id = id + 1;
        }
        update_rand(id);
        if (!boats.check_our_boat(id)) {
            boats.update_our_boat(id, i);
        }
    }
    // 统计船只去泊位的情况
    boats_going_berths.clear();
    for (auto& boat : boats.get_our_boats())
        if (boat->check_goto_berth() && boat->obj_berth_id != -1)  // 在去或在某个泊位
            boats_going_berths.insert(boat->obj_berth_id);
    // std::cerr << "here1" << std::endl;
    // 清除消失/拿走的货物
    mp.clear_cache_goods();
    get_ok();
}

void interaction_output() {
    // if (frame_id >= 2630) {
    //     LOG_TIME_THRESHOLD = 20;
    //     put_ok();
    //     return;
    // }
    // if (frame_id >= 357 && frame_id <= 359) {
    //     LOG_TIME_THRESHOLD = 0;
    // }
    // 跳帧检测
    skip_frames += frame_skip_detection(frame_id);
    if (skip_frames && SKIP_FRAME_ABORT && frame_id >= FRAME_NUM - 5) {
        throw std::runtime_error("skip_frames");
    }
    // 预处理
    timer.start();
    mp.update();
    timer.print_duration_with_reset("mp.update time", global_debug, LOG_TIME_THRESHOLD * 0.4);
    timer.start();
    boats.update_map(mp);
    timer.print_duration_with_reset("boats.update_map time", global_debug, LOG_TIME_THRESHOLD * 0.4);
    timer.start();
    robots.update_map(mp);
    timer.print_duration_with_reset("robots.match_good time", global_debug, LOG_TIME_THRESHOLD * 0.4);
    // 机器人
    patch_bfs_cnt = 0;
    good_bfs_cnt = 0;
    int our_robots_num = robots.get_private_size();
    for (int i = 0; i < our_robots_num; i++) {
        robots.get_our_robot(i).do_op(mp, move_commands);
    }
    timer.start();
    if (robots.get_private_size())
        Scheduler::do_commands(move_commands, mp, robots.get_our_robots());
    timer.print_duration_with_reset("do_commands time", global_debug, LOG_TIME_THRESHOLD * 0.3);
    // 船舶
    timer.start();
    // if (boats.get_size() > mp.berths.size()) {
    //     boat_real_time_decide = 0;
    // }
    if (robots.get_private_size() >= robot_max_num && boats.get_private_size() >= boat_num_max) {
        boat_real_time_decide = 0;
    }
    int our_boats_num = boats.get_private_size();
    set<int> do_schedule_boats;
    for (int i = 0; i < our_boats_num; i++) {
        if (boats.get_our_boat(i).do_op(mp, our_robots_num, our_boats_num, boats.get_size(), boat_goods_num)) {
            do_schedule_boats.insert(i);
        }
    }
    timer.print_duration_with_reset("boats do_op time", global_debug, LOG_TIME_THRESHOLD * 0.5);
    Scheduler::do_boats_commands(mp, boats.get_our_boats(), do_schedule_boats);
    // 打印结果
    if (global_debug && frame_id == FRAME_NUM) {
        // 睡眠方便打印大量自己的东西
        std::cerr << endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
        std::cerr << endl;
        final_log();
        exit(0);
    }
    int will_buy_boat = 0, will_buy_robot = 0;
    // auto [will_buy_boat, will_buy_robot] = decide_buy_what();
    buy_boat(will_buy_boat);
    buy_robot(will_buy_robot);
    put_ok();
}

int main() {
#ifdef DEBUG
    global_debug = true;
    LLM_DEBUG = true;
    SKIP_FRAME_ABORT = false;
#ifdef LOG
    // 判断文件是否已经存在，存在则切换文件名
    int file_id = 1;
    while (true) {
        err_path = ERR_FILE_PATH + std::to_string(file_id) + std::string(".txt");
        std::ifstream file_test(err_path);
        if (file_test.good()) {
            file_test.close();
            file_id++;
        } else {
            break;
        }
    }
    update_rand(file_id);
    std::ofstream file_err(err_path.c_str());
    std::streambuf* oldCerrStreamBuf = std::cerr.rdbuf();
    std::cerr.rdbuf(file_err.rdbuf());
#endif  // LOG
#endif  // DEBUG
    init();
#ifdef DEBUG
    timer.print_duration("init");
#endif  // DEBUG
    for (int frame = 1; frame <= FRAME_NUM; frame++) {
        // 输入
        interaction_input();
        timer.print_duration("interaction_input time", global_debug, LOG_TIME_THRESHOLD * 0.4);
        // 输出
        try {
            interaction_output();
        } catch (const std::exception& e) {
            std::cerr << "exception: " << e.what() << std::endl;
        }
        frame = frame_id;
        auto duration = timer.get_duration();
        time_sum += duration;
        timer.print_duration_with_reset("time", global_debug, LOG_TIME_THRESHOLD * 0.9);
    }
#ifdef DEBUG
#ifdef LOG
    std::cerr.rdbuf(oldCerrStreamBuf);
    file_err.close();
#endif  // LOG
#endif  // DEBUG
    return 0;
}

pii decide_buy_what() {
    int will_buy_boat = 0, will_buy_robot = 0;
    if (frame_id <= 100)
        return {will_buy_boat, will_buy_robot};
    if (goods_get_val_sum.size() == 0)
        return {will_buy_boat, will_buy_robot};
    double goods_get_val_sums = 0;
    for (auto val : goods_get_val_sum) {
        goods_get_val_sums += val;
    }
    goods_get_val_sums /= goods_get_val_sum.size();
    double revenue_per_frame = goods_get_val_sums / 2;
    int waiting_buy_boat = double(BOAT_PRICE[0] - current_money) / 10;
    if (current_money < robot_price || current_money > BOAT_PRICE[0])
        waiting_buy_boat = 0;
    double boat_revenue = revenue_per_frame / (mp.working_boats.size() + boats.get_private_size()) *
                              (FRAME_NUM - frame_id - 1500 - waiting_buy_boat) * 1.1 -
                          BOAT_PRICE[0];
    double robot_revenue =
        revenue_per_frame / (mp.working_robots.size() + robots.get_private_size()) * (FRAME_NUM - frame_id - 2000) -
        robot_price;
    if (boat_revenue > 0)
        will_buy_boat = 1;
    if (robot_revenue > 0)
        will_buy_robot = 1;
    if (boat_revenue > robot_revenue)
        will_buy_robot = 0;
    return {will_buy_boat, will_buy_robot};
}

void buy_boat(int will_buy_boat) {
    static int last_buy_boat_time = -100000;
    // if (frame_id - last_buy_boat_time < CANT_BUY_BOAT_TIME)
    //     return;
    if (manual_boat_num && 1.0 * boats.get_private_size() / boat_num_max > 1.0 * robots.get_private_size() / robot_max_num)
        return;
    for (auto& i : mp.ocean_zone_boat_num) {
        int zone_val_sum = 0;
        for (int& berth_id : mp.ocean_zone_to_berths[i.first]) {
            zone_val_sum += mp.berths[berth_id].val_sum;
        }
        int val_threshold = 50000 + frame_id * 2 + boats.get_private_size() * 4000 + boats.get_size() * 1000;  // 待分区
        // frame_id < 16000 && zone_val_sum > val_threshold
        if (current_money < BOAT_PRICE[0])
            break;
        bool lflag = liuyishou && mp.ocean_zone_boat_num_now[i.first] < boat_num_max + liuyishou &&
                     robots.get_now_private_size() >= robot_max_num && current_money >= BOAT_PRICE[0] * 2;
        while (i.second > mp.ocean_zone_boat_num_now[i.first] || (will_buy_boat > 0 && !manual_boat_num) ||
               (!manual_boat_num && frame_id < 16000 && zone_val_sum > val_threshold) ||
               (manual_boat_num && frame_id > 10 && mp.ocean_zone_boat_num_now[i.first] < boat_num_max) || (lflag)) {
            current_money -= BOAT_PRICE[0];
            final_score -= BOAT_PRICE[0];
            last_buy_boat_time = frame_id;
            mp.ocean_zone_boat_num_now[i.first]++;
            int purchase_point_id = random_int(0, mp.zone_to_boat_purchase_points[i.first].size() - 1);
            if (manual_buy_boat_order && mp.boat_purchase_ids[i.first].size() > 0) {
                purchase_point_id = mp.boat_purchase_ids[i.first].front();
                // if(global_debug)
                // std::cerr << "manual_buy_boat_at:" << purchase_point_id << std::endl;
                mp.boat_purchase_ids[i.first].pop();
            }
            if (lflag)
                purchase_point_id = 0;
            Point purchase_point = mp.boat_purchase_points[purchase_point_id];
            printf("lboat %d %d 0\n", purchase_point.x, purchase_point.y);
            if (!lflag)
                boats.add_our_boat();
            if (global_debug)
                std::cerr << "[buy boat " << mp.ocean_zone_boat_num_now[i.first] << "] ";
            will_buy_boat--;
            if (current_money < BOAT_PRICE[0])
                break;
            if (current_money < BOAT_PRICE[0] || frame_id > 100)
                return;
            if (lflag)
                lflag = false;
            val_threshold = 50000 + frame_id * 2 + boats.get_private_size() * 4000 + boats.get_size() * 1000;
        }
    }
    if (frame_id >= 19994 && liuyishou && mp.ocean_zone_boat_num_now.at(0) < boat_num_max + liuyishou &&
        current_money >= BOAT_PRICE[0]) {
        current_money -= BOAT_PRICE[0];
        final_score -= BOAT_PRICE[0];
        mp.ocean_zone_boat_num_now[0]++;
        printf("lboat %d %d 0\n", mp.boat_purchase_points[1].x, mp.boat_purchase_points[1].y);
    }
}

void buy_robot(int will_buy_robot) {
    if (current_money < robot_price)
        return;
    if (manual_boat_num && 1.0 * boats.get_private_size() / boat_num_max < 1.0 * robots.get_private_size() / robot_max_num)
        return;
    int cnt = 0;
    // robots.get_now_private_size() < robot_init_num
    while (robots.get_now_private_size() < robot_max_num || will_buy_robot > 0) {
        // std::cerr << frame_id << ": " << robots.get_private_size() << " " << robot_max_num << std::endl;
        int ronot_purchase_point_id = random_int(0, mp.robot_purchase_points.size() - 1);
        Point robot_purchase_point = mp.robot_purchase_points[ronot_purchase_point_id];
        current_money -= robot_price;
        final_score -= robot_price;
        robots_money_sum += robot_price;
        // buy
        printf("lbot %d %d %d\n", robot_purchase_point.x, robot_purchase_point.y, robot_type);
        robots.add_our_robot(robot_type);
        cnt++;
        if (global_debug)
            std::cerr << "[buy robot " << robots.get_now_private_size() << "] ";
        will_buy_robot--;
        if (frame_id > 100 && cnt > robot_max_num / boat_num_max)
            return;
        if (current_money < robot_price)
            return;
    }
    // static bool robots_num_log = false;
    // if (!robots_num_log && robots.get_now_private_size() == robot_max_num) {
    //     robots_num_log = true;
    //     all_robots_time = frame_id;
    //     if (global_debug)
    //         std::cerr << frame_id << ": all robots" << std::endl;
    // }
}

void final_log() {
    // std::cerr << "final_log: " << std::endl;
    if (skip_frames)
        std::cerr << std::endl;
    if (skip_frames)
        std::cerr << "skipped frames number: " << skip_frames << std::endl;
    int val_sum_sum = 0, on_boats = 0;
    for (int i = 0; i < (int)mp.berths.size(); i++) {
        val_sum_sum += mp.berths[i].val_sum;
        std::cerr << "berth" << i << ", goods_cnt: " << mp.berths[i].get_goods_num() << ",\tval_sum: "
                  << mp.berths[i].val_sum
                  //   << ",\tefficiency: " << mp.berths[i].efficiency
                  //   << ",\tpred_num_per100: " << mp.berths[i].pred_num_per100
                  << ",\tnum_per_100: " << mp.berths[i].good_num_all / 150.0 << std::endl;
    }
    // 打印用于手动初始效率
    // std::cerr << "{";
    // for (int i = 0; i < (int)mp.berths.size(); i++) {
    //     std::cerr << mp.berths[i].efficiency << ",";
    // }
    // std::cerr << "}" << std::endl;
    // std::cerr << "{";
    // for (int i = 0; i < (int)mp.berths.size(); i++) {
    //     std::cerr << mp.berths[i].good_num_all / 150.0 / 100 << ",";
    // }
    // std::cerr << "}" << std::endl;
    // for (auto& boat : boats.get_our_boats()) {
    //     on_boats += boat->good_val;
    // }
    // robots_ability - left_all + 25000 - 8000*boat = score
    std::cerr << "berth_val_sum: " << val_sum_sum << "\ton_boats: " << on_boats << "\tleft_all: " << val_sum_sum + on_boats
              << std::endl;
    std::cerr << "pulled_goods_num: " << pulled_goods_num << "\tpulled_goods_val: " << pulled_goods_val
              << "\trobots_ability: " << pulled_goods_val - robots_money_sum << "\tdelivered_goods_val: "
              << delivered_goods_val
              //  << "\taverage_pulled_goods_val: "
              //  << double(pulled_goods_val) / pulled_goods_num
              << endl;
    std::cerr << "all_goods_num: " << all_goods_num << "\tall_goods_val_sum: " << all_goods_val_sum << std::endl;
    std::cerr << "final_money: " << final_score << std::setprecision(3) << "\ttime: " << double(time_sum) / 1000 << "s"
              << std::endl;
    // LLM的回答时间和正确率
    double llm_avg_time = 0, llm_max_time = 0, llm_min_time = 0x3f3f3f3f, llm_correct_rate = 0;
    for (auto ans_time : llm_answer_time) {
        llm_avg_time += ans_time;
        llm_max_time = std::max(llm_max_time, ans_time);
        llm_min_time = std::min(llm_min_time, ans_time);
    }
    llm_avg_time /= llm_answer_time.size();
    for (auto ans_flag : llm_answer_flag) {
        llm_correct_rate += ans_flag;
    }
    llm_correct_rate /= llm_answer_flag.size();
    std::cerr << "llm_avg_time: " << llm_avg_time << "\tllm_max_time: " << llm_max_time << "\tllm_min_time: " << llm_min_time
              << "\tllm_correct_rate: " << llm_correct_rate << std::endl;
}
