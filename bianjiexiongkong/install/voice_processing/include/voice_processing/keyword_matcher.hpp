#ifndef KEYWORD_MATCHER_HPP
#define KEYWORD_MATCHER_HPP

#include <vector>
#include <string>
#include <utility>  // 对于 std::pair

class KeywordMatcher {
public:
    KeywordMatcher();
    std::string match_command(const std::string& input, bool debug = false) const;

private:
    std::vector<std::pair<std::string, std::string>> commands_;
};

#endif // KEYWORD_MATCHER_HPP