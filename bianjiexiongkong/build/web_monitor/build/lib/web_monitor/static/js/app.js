class WebMonitor {
    constructor() {
        this.ws = null;
        this.connectionStatus = document.getElementById('connection-status');
        this.messageCount = document.getElementById('message-count');
        this.msgCount = 0;
        this.lastData = {};
        
        // 检测WSL2环境
        this.isWSL2 = this.isWSL2Environment();
        if (this.isWSL2) {
            console.log('检测到WSL2环境');
            this.showWSL2Info();
        }
        
        this.init();
    }
    
    init() {
        this.connectWebSocket();
        this.setupEventListeners();
    }
    
    connectWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        
        // 检测WSL2环境并调整连接策略
        let wsUrl;
        if (this.isWSL2Environment()) {
            // 在WSL2环境中，尝试多种连接方式
            const hostname = window.location.hostname;
            if (hostname === 'localhost' || hostname === '127.0.0.1') {
                // 从Windows访问，使用localhost
                wsUrl = `${protocol}//localhost:9090/ws`;
            } else {
                // 从WSL2内部访问，使用当前hostname
                wsUrl = `${protocol}//${hostname}:9090/ws`;
            }
        } else {
            // 非WSL2环境，使用标准方式
            wsUrl = `${protocol}//${window.location.hostname}:9090/ws`;
        }
        
        console.log(`尝试连接WebSocket: ${wsUrl}`);
        
        this.ws = new WebSocket(wsUrl);
        
        this.ws.onopen = () => {
            console.log('Connected to WebSocket server');
            this.connectionStatus.textContent = '已连接';
            this.connectionStatus.className = 'connected';
        };
        
        this.ws.onclose = () => {
            console.log('Disconnected from WebSocket server');
            this.connectionStatus.textContent = '未连接';
            this.connectionStatus.className = 'disconnected';
            
            // 尝试重新连接
            setTimeout(() => this.connectWebSocket(), 3000);
        };
        
        this.ws.onerror = (error) => {
            console.error('WebSocket error:', error);
            this.connectionStatus.textContent = '连接错误';
            this.connectionStatus.className = 'disconnected';
            
            // 如果是WSL2环境，显示帮助信息
            if (this.isWSL2Environment()) {
                this.showWSL2Help();
            }
        };
        
        this.ws.onmessage = (event) => {
            this.msgCount++;
            this.messageCount.textContent = `消息: ${this.msgCount}`;
            
            try {
                const data = JSON.parse(event.data);
                this.handleMessage(data);
            } catch (error) {
                console.error('Error parsing message:', error);
            }
        };
    }
    
    handleMessage(message) {
        if (message.type === 'initial_data' || message.type === 'update') {
            this.updateUI(message.data);
        }
    }
    
    updateUI(data) {
        // 更新按钮状态
        if (data.button_state && JSON.stringify(data.button_state) !== JSON.stringify(this.lastData.button_state)) {
            this.updateButtonState(data.button_state);
            this.lastData.button_state = data.button_state;
        }
        
        // 更新头部跟踪
        if (data.head_tracking && JSON.stringify(data.head_tracking) !== JSON.stringify(this.lastData.head_tracking)) {
            this.updateHeadTracking(data.head_tracking);
            this.lastData.head_tracking = data.head_tracking;
        }
        
        // 更新语音文本
        if (data.voice_text && JSON.stringify(data.voice_text) !== JSON.stringify(this.lastData.voice_text)) {
            this.updateVoiceText(data.voice_text);
            this.lastData.voice_text = data.voice_text;
        }
        
        // 更新语音命令
        if (data.voice_command && JSON.stringify(data.voice_command) !== JSON.stringify(this.lastData.voice_command)) {
            this.updateVoiceCommand(data.voice_command);
            this.lastData.voice_command = data.voice_command;
        }
        
        // 更新视频源
        if (data.video_source && JSON.stringify(data.video_source) !== JSON.stringify(this.lastData.video_source)) {
            this.updateVideoSource(data.video_source);
            this.lastData.video_source = data.video_source;
        }
        
        // 更新设备状态
        if (data.equipment_state && JSON.stringify(data.equipment_state) !== JSON.stringify(this.lastData.equipment_state)) {
            this.updateEquipmentState(data.equipment_state);
            this.lastData.equipment_state = data.equipment_state;
        }
        
        // 更新IMU数据
        if (data.imu_data && JSON.stringify(data.imu_data) !== JSON.stringify(this.lastData.imu_data)) {
            this.updateIMUData(data.imu_data);
            this.lastData.imu_data = data.imu_data;
        }
    }
    
    updateButtonState(data) {
        const container = document.getElementById('button-state');
        const timestamp = this.formatTimestamp(data.timestamp);
        
        const modes = ['默认模式', '模式A', '模式B', '模式C'];
        const overlayModes = ['无叠加', '最小叠加', '完全叠加'];
        
        container.innerHTML = `
            <div class="data-item">
                <span>头动开关: <span class="${data.head_tracking_enabled ? 'status-on' : 'status-off'}">${data.head_tracking_enabled ? '开' : '关'}</span></span>
            </div>
            <div class="data-item">
                <span>语音开关: <span class="${data.voice_command_enabled ? 'status-on' : 'status-off'}">${data.voice_command_enabled ? '开' : '关'}</span></span>
            </div>
            <div class="data-item">
                <span>正面屏开关: <span class="${data.front_screen_enabled ? 'status-on' : 'status-off'}">${data.front_screen_enabled ? '开' : '关'}</span></span>
            </div>
            <div class="data-item">
                <span>正面屏模式: ${modes[data.front_screen_mode] || '未知'}</span>
            </div>
            <div class="data-item">
                <span>视频源选择: 源${data.hmd_video_source}</span>
            </div>
            <div class="data-item">
                <span>叠加模式: ${overlayModes[data.hmd_overlay_mode] || '未知'}</span>
            </div>
            <div class="data-item">
                <span>头显开关: <span class="${data.hmd_enabled ? 'status-on' : 'status-off'}">${data.hmd_enabled ? '开' : '关'}</span></span>
            </div>
            <div class="timestamp">更新时间: ${timestamp}</div>
        `;
    }
    
    updateHeadTracking(data) {
        const container = document.getElementById('head-tracking');
        const timestamp = this.formatTimestamp(data.timestamp);
        const pitchDeg = (data.pitch * 180 / Math.PI).toFixed(2);
        const yawDeg = (data.yaw * 180 / Math.PI).toFixed(2);
        
        container.innerHTML = `
            <div class="data-item">
                <span>俯仰角: ${pitchDeg}° (${data.pitch.toFixed(4)} rad)</span>
            </div>
            <div class="data-item">
                <span>偏航角: ${yawDeg}° (${data.yaw.toFixed(4)} rad)</span>
            </div>
            <div class="data-item">
                <span>跟踪状态: <span class="${data.is_tracking ? 'status-on' : 'status-off'}">${data.is_tracking ? '开启' : '关闭'}</span></span>
            </div>
            <div class="data-item">
                <span>置信度: <span class="${data.confidence > 0.7 ? 'value-high' : data.confidence > 0.4 ? 'value-medium' : 'value-low'}">${(data.confidence * 100).toFixed(1)}%</span></span>
            </div>
            <div class="timestamp">更新时间: ${timestamp}</div>
        `;
    }
    
    updateVoiceText(data) {
        const container = document.getElementById('voice-text');
        const timestamp = this.formatTimestamp(data.timestamp);
        
        container.innerHTML = `
            <div class="data-item">
                <span>文本: "${data.raw_text}"</span>
            </div>
            <div class="data-item">
                <span>最终结果: <span class="${data.is_final ? 'status-on' : 'status-off'}">${data.is_final ? '是' : '否'}</span></span>
            </div>
            <div class="data-item">
                <span>置信度: <span class="${data.confidence > 0.7 ? 'value-high' : data.confidence > 0.4 ? 'value-medium' : 'value-low'}">${(data.confidence * 100).toFixed(1)}%</span></span>
            </div>
            <div class="timestamp">更新时间: ${timestamp}</div>
        `;
    }
    
    updateVoiceCommand(data) {
        const container = document.getElementById('voice-command');
        const timestamp = this.formatTimestamp(data.timestamp);
        
        container.innerHTML = `
            <div class="data-item">
                <span>有效性: <span class="${data.is_valid ? 'status-on' : 'status-off'}">${data.is_valid ? '有效' : '无效'}</span></span>
            </div>
            <div class="data-item">
                <span>命令ID: ${data.command_id}</span>
            </div>
            <div class="data-item">
                <span>帧头: 0x${data.frame_header.toString(16).toUpperCase()}</span>
            </div>
            <div class="data-item">
                <span>参数1: ${data.param1}</span>
            </div>
            <div class="data-item">
                <span>参数2: ${data.param2}</span>
            </div>
            <div class="data-item">
                <span>参数3: ${data.param3}</span>
            </div>
            <div class="timestamp">更新时间: ${timestamp}</div>
        `;
    }
    
    updateVideoSource(data) {
        const container = document.getElementById('video-source');
        const timestamp = this.formatTimestamp(data.timestamp);
        
        container.innerHTML = `
            <div class="data-item">
                <span>ID: ${data.id}</span>
            </div>
            <div class="data-item">
                <span>名称: ${data.source_name}</span>
            </div>
            <div class="data-item">
                <span>IP地址: ${data.ip_address}</span>
            </div>
            <div class="data-item">
                <span>分辨率: ${data.width}×${data.height}</span>
            </div>
            <div class="data-item">
                <span>帧率: ${data.fps} FPS</span>
            </div>
            <div class="data-item">
                <span>编码: ${data.encoding}</span>
            </div>
            <div class="data-item">
                <span>可用性: <span class="${data.is_available ? 'status-on' : 'status-off'}">${data.is_available ? '可用' : '不可用'}</span></span>
            </div>
            <div class="timestamp">更新时间: ${timestamp}</div>
        `;
    }
    
    updateEquipmentState(data) {
        const container = document.getElementById('equipment-state');
        const timestamp = this.formatTimestamp(data.timestamp);
        
        // 格式化弹药信息
        let ammoInfo = '无';
        if (data.ammo_types && data.ammo_types.length > 0) {
            ammoInfo = '';
            for (let i = 0; i < data.ammo_types.length; i++) {
                if (i > 0) ammoInfo += ', ';
                ammoInfo += `${data.ammo_types[i]}: ${data.ammo_counts[i]}`;
            }
        }
        
        // 格式化警告信息
        let warningsInfo = '无';
        if (data.warnings && data.warnings.length > 0) {
            warningsInfo = data.warnings.join(', ');
            if (warningsInfo.length > 100) {
                warningsInfo = warningsInfo.substring(0, 100) + '...';
            }
        }
        
        container.innerHTML = `
            <div class="data-item">
                <span>位置: 经度 ${data.longitude.toFixed(6)}, 纬度 ${data.latitude.toFixed(6)}</span>
            </div>
            <div class="data-item">
                <span>速度: ${data.speed.toFixed(1)} km/h</span>
            </div>
            <div class="data-item">
                <span>燃油: <span class="${data.fuel_level > 0.3 ? 'value-high' : data.fuel_level > 0.1 ? 'value-medium' : 'value-low'}">${(data.fuel_level * 100).toFixed(1)}%</span></span>
            </div>
            <div class="data-item">
                <span>电量: <span class="${data.battery_level > 0.3 ? 'value-high' : data.battery_level > 0.1 ? 'value-medium' : 'value-low'}">${(data.battery_level * 100).toFixed(1)}%</span></span>
            </div>
            <div class="data-item">
                <span>姿态: 横滚 ${(data.roll * 180 / Math.PI).toFixed(2)}°, 俯仰 ${(data.pitch * 180 / Math.PI).toFixed(2)}°, 偏航 ${(data.yaw * 180 / Math.PI).toFixed(2)}°</span>
            </div>
            <div class="data-item">
                <span>光电转台: 俯仰 ${(data.gimbal_pitch * 180 / Math.PI).toFixed(2)}°, 偏航 ${(data.gimbal_yaw * 180 / Math.PI).toFixed(2)}°, 状态: <span class="${data.gimbal_is_active ? 'status-on' : 'status-off'}">${data.gimbal_is_active ? '激活' : '未激活'}</span></span>
            </div>
            <div class="data-item">
                <span>弹药: ${ammoInfo}</span>
            </div>
            <div class="data-item">
                <span>警告: <span class="${data.warnings && data.warnings.length > 0 ? 'warning' : ''}">${warningsInfo}</span></span>
            </div>
            <div class="timestamp">更新时间: ${timestamp}</div>
        `;
    }
    
    updateIMUData(data) {
        const container = document.getElementById('imu-data');
        const timestamp = this.formatTimestamp(data.timestamp);
        
        container.innerHTML = `
            <div class="data-item">
                <span>线性加速度: X=${data.linear_acceleration.x.toFixed(4)}, Y=${data.linear_acceleration.y.toFixed(4)}, Z=${data.linear_acceleration.z.toFixed(4)} m/s²</span>
            </div>
            <div class="data-item">
                <span>角速度: X=${data.angular_velocity.x.toFixed(4)}, Y=${data.angular_velocity.y.toFixed(4)}, Z=${data.angular_velocity.z.toFixed(4)} rad/s</span>
            </div>
            <div class="timestamp">更新时间: ${timestamp}</div>
        `;
    }
    
    formatTimestamp(timestamp) {
        if (!timestamp) return '未知时间';
        
        const date = new Date(timestamp.sec * 1000 + timestamp.nanosec / 1000000);
        return date.toLocaleString('zh-CN', {
            year: 'numeric',
            month: '2-digit',
            day: '2-digit',
            hour: '2-digit',
            minute: '2-digit',
            second: '2-digit'
        });
    }
    
    setupEventListeners() {
        // 可以添加一些交互功能，如手动刷新等
    }
    
    isWSL2Environment() {
        // 通过用户代理和主机名检测WSL2环境
        const userAgent = navigator.userAgent;
        const hostname = window.location.hostname;
        
        // 检查是否包含WSL2相关标识
        return userAgent.includes('Linux') && 
               (hostname.includes('wsl') || 
                hostname.includes('microsoft') || 
                hostname.match(/^172\.\d+\.\d+\.\d+$/) ||
                hostname.match(/^192\.168\.\d+\.\d+$/));
    }
    
    showWSL2Info() {
        // 在页面上显示WSL2环境信息
        const infoDiv = document.createElement('div');
        infoDiv.className = 'wsl2-info';
        infoDiv.innerHTML = `
            <div class="wsl2-banner">
                <strong>WSL2环境检测</strong>
                <p>检测到您正在使用WSL2环境。如果连接失败，请确保已配置Windows端口转发。</p>
                <button onclick="this.parentElement.style.display='none'">关闭</button>
            </div>
        `;
        
        // 添加样式
        const style = document.createElement('style');
        style.textContent = `
            .wsl2-info { position: fixed; top: 10px; right: 10px; z-index: 1000; }
            .wsl2-banner { 
                background: #fff3cd; 
                border: 1px solid #ffeaa7; 
                border-radius: 5px; 
                padding: 15px; 
                max-width: 300px;
                box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            }
            .wsl2-banner button { 
                background: #007bff; 
                color: white; 
                border: none; 
                padding: 5px 10px; 
                border-radius: 3px; 
                cursor: pointer; 
                margin-top: 10px;
            }
        `;
        document.head.appendChild(style);
        document.body.appendChild(infoDiv);
    }
    
    showWSL2Help() {
        // 显示WSL2连接帮助信息
        const helpDiv = document.createElement('div');
        helpDiv.className = 'wsl2-help';
        helpDiv.innerHTML = `
            <div class="help-modal">
                <div class="help-content">
                    <h3>WSL2连接问题解决方案</h3>
                    <p>在WSL2环境中，需要配置Windows端口转发才能从Windows浏览器访问。</p>
                    <h4>解决步骤：</h4>
                    <ol>
                        <li>在Windows PowerShell中运行（管理员权限）：</li>
                        <li><code>netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=WSL2_IP</code></li>
                        <li>将WSL2_IP替换为实际的WSL2 IP地址</li>
                        <li>配置防火墙：<code>netsh advfirewall firewall add rule name="WSL2 Web Monitor" dir=in action=allow protocol=TCP localport=9090</code></li>
                    </ol>
                    <button onclick="this.parentElement.parentElement.remove()">关闭</button>
                </div>
            </div>
        `;
        
        // 添加样式
        const style = document.createElement('style');
        style.textContent = `
            .wsl2-help { position: fixed; top: 0; left: 0; width: 100%; height: 100%; background: rgba(0,0,0,0.5); z-index: 1001; }
            .help-modal { display: flex; align-items: center; justify-content: center; height: 100%; }
            .help-content { 
                background: white; 
                padding: 20px; 
                border-radius: 10px; 
                max-width: 500px; 
                max-height: 80vh; 
                overflow-y: auto;
            }
            .help-content button { 
                background: #007bff; 
                color: white; 
                border: none; 
                padding: 10px 20px; 
                border-radius: 5px; 
                cursor: pointer; 
                margin-top: 15px;
            }
            .help-content code { 
                background: #f8f9fa; 
                padding: 2px 5px; 
                border-radius: 3px; 
                font-family: monospace;
            }
        `;
        document.head.appendChild(style);
        document.body.appendChild(helpDiv);
    }
}

// 页面加载完成后初始化监控器
document.addEventListener('DOMContentLoaded', () => {
    new WebMonitor();
});