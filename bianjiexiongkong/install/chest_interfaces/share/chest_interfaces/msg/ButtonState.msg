std_msgs/Header header

# 侧按键1：头动开关
bool head_tracking_enabled

# 侧按键2：语音开关
bool voice_command_enabled

# 正按键1：正面屏开关
bool front_screen_enabled

# 正按键2：正面屏显示模式 (语义化枚举)
uint8 front_screen_mode
uint8 FRONT_SCREEN_DEFAULT=0
uint8 FRONT_SCREEN_MODE_A=1
uint8 FRONT_SCREEN_MODE_B=2
uint8 FRONT_SCREEN_MODE_C=3

# 正按键3：头显视频源 (索引)
uint8 hmd_video_source # 选中的视频源索引

# 正按键4：头显叠加模式 (语义化枚举)
uint8 hmd_overlay_mode
uint8 HMD_OVERLAY_NONE=0
uint8 HMD_OVERLAY_MINIMAL=1
uint8 HMD_OVERLAY_FULL=2

# 正按键5：头显开关
bool hmd_enabled