std_msgs/Header header
# 唯一标识符 (用于前端绑定)
uint8 id
string ip_address       # IP地址(包含端口号，格式: "ip:port")
string source_name      # 显示名称
uint32 fps              # 帧率
uint32 width            # 分辨率宽
uint32 height           # 分辨率高
string encoding         # 编码格式 (e.g. "h264", "h265")
bool is_available       # 是否可用