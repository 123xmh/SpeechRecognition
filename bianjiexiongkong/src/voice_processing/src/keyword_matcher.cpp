#include "voice_processing/keyword_matcher.hpp"
#include <algorithm>
#include <cctype>
#include <iostream>

KeywordMatcher::KeywordMatcher() {
    // 初始化关键词库
    commands_ = {
        {"启动", "start"},
        {"停止", "stop"},
        {"开启", "on"},
        {"关闭", "off"},
        {"切换", "switch"},
        {"选择", "select"},
        {"确认", "confirm"},
        {"取消", "cancel"},
        {"前进", "forward"},
        {"后退", "backward"},
        {"左转", "turn_left"},
        {"右转", "turn_right"},
        {"加速", "accelerate"},
        {"减速", "decelerate"},
        {"状态", "status"},
        {"报告", "report"},
        {"目标", "target"},
        {"攻击", "attack"},
        {"防御", "defend"},
        {"侦察", "recon"}
    };
}

std::string KeywordMatcher::match_command(const std::string& text, bool debug) const {
    if (debug) {
        std::cout << "Matching text: " << text << std::endl;
    }
    
    // 转换为小写以进行不区分大小写的匹配
    std::string lower_text = text;
    std::transform(lower_text.begin(), lower_text.end(), lower_text.begin(),
                   [](unsigned char c){ return std::tolower(c); });
    
    // 查找匹配的关键词
    for (const auto& [keyword, command] : commands_) {
        if (lower_text.find(keyword) != std::string::npos) {
            if (debug) {
                std::cout << "Matched keyword: " << keyword << " -> Command: " << command << std::endl;
            }
            return command;
        }
    }
    
    if (debug) {
        std::cout << "No keyword matched" << std::endl;
    }
    
    return ""; // 未找到匹配
}