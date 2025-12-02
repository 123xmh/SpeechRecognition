std_msgs/Header header
# 语音识别结果
string raw_text         # 原始识别文本(中文、英文、数字等，不超过100个字符)
bool is_final           # 是否为最终结果 (true=最终, false=中间结果)
float32 confidence      # 整个文本的置信度 [0, 1]