std_msgs/Header header
# 语音指令
uint16 frame_header      # 前8位为分类码（Category）后8位为操作码（Operation），构成： (Category << 8) | Operation
uint32 command_id        # 指令唯一ID 
int32 param1             # 参数 1 (使用缩放因子后的整数值，例如温度×10，纬度×1e7等)
int32 param2             # 参数 2 (使用缩放因子后的整数值，例如经度×1e7等)
uint8 param3             # 参数 3 (模式、开关、档位、查询子项等)
# 命令有效性
bool is_valid            # 是否有效命令 (由解析节点设置，0 无效 1 有效)
