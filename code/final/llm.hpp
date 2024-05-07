#pragma once
#pragma GCC optimize("O2")
#include <curl/curl.h>
#include "utils.hpp"

const std::string API_URL = "xxx"; // 请填写你的API地址
const std::string CONTENT_TYPE = "Content-Type: application/json;charset=utf-8";
const std::string AUTH_TOKEN = "xxx"; // 请填写你的API Token
const std::string qa_file_path = "./qa.txt";

int generate_normal_dist(int mean=LLM_TIME_AVG, int stddev=LLM_TIME_STD) {
    static int seed = 0;
    static std::mt19937 gen(seed); // 使用Mersenne Twister算法生成随机数，种子为参数指定的值
    static std::normal_distribution<> d(mean, stddev); // 均值和标准差由参数指定
    return round(d(gen)); // 使用round函数将生成的浮点数四舍五入为整数
}

class LLM {
   private:
    static size_t writeCallback(char* contents, size_t size, size_t nmemb, std::string* data) {
        data->append(contents, size * nmemb);
        return size * nmemb;
    }
    std::string prompt;
    std::future<std::string> future_result;
    std::chrono::steady_clock::time_point start;
    std::map<std::string, int> question_answer;
    bool debug{false};


   public:
    LLM(bool debug = false){
        this->debug = debug;
        prompt = "";
        if (debug) {
            std::ifstream fin(qa_file_path);
            std::string line, question;
            int answer;
            int cnt = 0;
            while (std::getline(fin, line)) {
                if (cnt % 3 == 0) {
                    question = line.substr(line.find_first_of(" ") + 1);
                } else if (cnt % 3 == 1) {
                    answer = line[0] - 'A';
                    question_answer[question] = answer;
                }
                cnt++;
            }
        }
    }
    ~LLM() {}
    LLM(const LLM&) = delete;
    LLM& operator=(const LLM&) = delete;

    std::string get_response_without_prompt(const std::string& text, double temperature = LLM_TEMPERATURE) {
        CURL* curl = curl_easy_init();
        std::string response;

        // Set URL
        curl_easy_setopt(curl, CURLOPT_URL, API_URL.c_str());
        // Set request type: POST
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        // Set request params.
        std::string postData = "{\"prompt\":\"" + text + "\",\"temperature\":" + std::to_string(temperature) + "}";
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
        // Set headers
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, CONTENT_TYPE.c_str());
        headers = curl_slist_append(headers, AUTH_TOKEN.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        // Set callback function and response
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);

        // Send this request
        CURLcode res = curl_easy_perform(curl);

        // Clean up
        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        if (res != CURLE_OK) {
            std::cerr << "curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
            return "";
        }
        return response;
    }
    std::string get_response(const std::string& problem) {
        std::string converted_prompt = prompt + problem + "。答案是：";
        auto response = get_response_without_prompt(converted_prompt);
        return response;
    }
    std::string get_response_debug(const std::string& problem, double temperature = LLM_TEMPERATURE) {
        if (question_answer.find(problem) == question_answer.end()) {
            throw std::runtime_error("problem do NOT exist!" + problem);
        }
        // 睡眠一段随机时间，模拟大模型运行时间（符合正态分布：均值400ms，方差100ms）
        std::this_thread::sleep_for(std::chrono::milliseconds(generate_normal_dist()));
        return "{\"text\":\"" + std::string(1, 'A' + question_answer[problem]) + "\"}";
    }
    void ask_question(const std::string& question, bool reset_time = false, double temperature = LLM_TEMPERATURE) {
        if (reset_time)
            start = std::chrono::steady_clock::now();
        // 创建一个新线程来运行大模型，并将结果存储在future_result中
        if (debug)
            future_result = std::async(std::launch::async, &LLM::get_response_debug, this, question, temperature);
        else
            future_result = std::async(std::launch::async, &LLM::get_response, this, question);
    }

    bool is_result_ready() {
        // 检查future_result是否已经准备好
        return future_result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
    }

    std::string get_result() {
        // 获取结果。如果结果还没有准备好，这将阻塞当前线程，直到结果准备好为止
        return future_result.get();
    }

    int get_result_option(double& duration) {
        // 统计时间，毫秒数
        duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        // 获取结果，并返回选项
        auto res = get_result();
        // 返回必须是包含{"text": ""}这样的格式
        if (res.find("text") == std::string::npos) {
            return -1;
        }
        // 遍历字符串，找到选项
        for (int i = 0; i < (int)res.size(); i++) {
            if (res[i] == 'A' || res[i] == 'B' || res[i] == 'C' || res[i] == 'D') {
                return res[i] - 'A';
            }
        }
        return -1;
    }
};