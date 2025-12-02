/**
 * 车辆节点模拟器前端应用
 */

class VehicleSimulatorApp {
    constructor() {
        this.ws = null;
        this.isConnected = false;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectInterval = 3000;
        
        // 数据统计
        this.stats = {
            vehicleStatusSent: 0,
            headTrackingReceived: 0,
            voiceTextReceived: 0,
            voiceCommandReceived: 0,
            ackSent: 0
        };
        
        this.init();
    }
    
    init() {
        this.log('初始化车辆模拟器应用...', 'info');
        this.setupEventListeners();
        this.connectWebSocket();
        this.startStatusUpdate();
        this.log('应用初始化完成', 'info');
    }
    
    setupEventListeners() {
        // 车辆状态控制
        const toggleBtn = document.getElementById('toggle-vehicle-status');
        if (toggleBtn) {
            toggleBtn.addEventListener('click', () => {
                this.toggleVehicleStatus();
            });
        } else {
            console.warn('找不到toggle-vehicle-status元素');
        }
        
        // 设置发送频率
        const intervalBtn = document.getElementById('set-interval');
        if (intervalBtn) {
            intervalBtn.addEventListener('click', () => {
                this.setVehicleStatusInterval();
            });
        } else {
            console.warn('找不到set-interval元素');
        }
    }
    
    connectWebSocket() {
        const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
        const wsUrl = `${protocol}//${window.location.host}/ws`;
        
        this.log(`尝试连接WebSocket: ${wsUrl}`, 'info');
        
        try {
            this.ws = new WebSocket(wsUrl);
            
            this.ws.onopen = () => {
                console.log('WebSocket onopen事件触发');
                this.isConnected = true;
                this.reconnectAttempts = 0;
                this.updateConnectionStatus(true); // 先更新连接状态
                this.log('WebSocket连接已建立', 'info');
                
                // 请求初始状态
                this.sendMessage({
                    type: 'get_status'
                });
            };
            
            this.ws.onmessage = (event) => {
                try {
                    const message = JSON.parse(event.data);
                    this.handleMessage(message);
                } catch (error) {
                    this.log(`解析消息失败: ${error.message}`, 'error');
                }
            };
            
            this.ws.onclose = (event) => {
                this.isConnected = false;
                this.updateConnectionStatus(false);
                this.log(`WebSocket连接已断开: 代码=${event.code}, 原因=${event.reason}`, 'warning');
                
                // 自动重连
                if (this.reconnectAttempts < this.maxReconnectAttempts) {
                    this.reconnectAttempts++;
                    this.log(`尝试重连 (${this.reconnectAttempts}/${this.maxReconnectAttempts})...`, 'info');
                    setTimeout(() => this.connectWebSocket(), this.reconnectInterval);
                } else {
                    this.log('重连失败，请手动刷新页面', 'error');
                }
            };
            
            this.ws.onerror = (error) => {
                this.log(`WebSocket错误: ${error}`, 'error');
                console.error('WebSocket错误详情:', error);
            };
            
        } catch (error) {
            this.log(`连接WebSocket失败: ${error.message}`, 'error');
        }
    }
    
    handleMessage(message) {
        switch (message.type) {
            case 'status':
                this.handleStatusMessage(message.data);
                break;
            case 'data_update':
                this.handleDataUpdate(message.data_type, message.data);
                break;
            case 'received_data':
                this.handleReceivedData(message.data);
                break;
            default:
                this.log(`未知消息类型: ${message.type}`, 'warning');
        }
    }
    
    handleStatusMessage(data) {
        // 更新车辆状态按钮
        const button = document.getElementById('toggle-vehicle-status');
        const statusText = document.getElementById('vehicle-status-text');
        
        if (data.vehicle_status_enabled) {
            button.textContent = '停止';
            button.className = 'btn btn-danger';
            statusText.textContent = '运行中';
        } else {
            button.textContent = '启动';
            button.className = 'btn btn-primary';
            statusText.textContent = '已停止';
        }
        
        // 更新发送频率
        document.getElementById('vehicle-status-interval').value = Math.round(1 / data.vehicle_status_interval);
        
        // 更新服务器状态
        const serverStatus = document.getElementById('server-status');
        const serverStatusText = document.getElementById('server-status-text');
        
        if (data.connected_clients > 0) {
            serverStatus.className = 'status-indicator status-online';
            serverStatusText.textContent = `在线 (${data.connected_clients} 客户端)`;
        } else {
            serverStatus.className = 'status-indicator status-offline';
            serverStatusText.textContent = '离线';
        }
    }
    
    handleDataUpdate(dataType, data) {
        switch (dataType) {
            case 'vehicle_status':
                this.stats.vehicleStatusSent++;
                this.updateStats();
                this.addRealtimeData(`车辆状态发送: 位置(${data.data.longitude/1e7:.6f}, ${data.data.latitude/1e7:.6f}), 速度${data.data.speed/10}km/h`);
                break;
                
            case 'head_tracking':
                this.stats.headTrackingReceived++;
                this.updateStats();
                this.addHeadTrackingData(`偏航: ${data.yaw.toFixed(2)}°, 俯仰: ${data.pitch.toFixed(2)}°, 置信度: ${data.confidence}%`);
                break;
                
            case 'voice_text':
                this.stats.voiceTextReceived++;
                this.updateStats();
                this.addVoiceTextData(`操作: 0x${data.operation.toString(16).toUpperCase()}, 文本: "${data.text}"`);
                break;
                
            case 'voice_command':
                this.stats.voiceCommandReceived++;
                this.updateStats();
                this.addVoiceCommandData(`类别: 0x${data.category.toString(16).toUpperCase()}, 操作: 0x${data.operation.toString(16).toUpperCase()}, ID: ${data.command_id}`);
                break;
                
            case 'voice_command_ack':
                this.stats.ackSent++;
                this.updateStats();
                const statusText = data.status === 0x01 ? '成功' : '失败';
                this.addAckData(`确认包: ID=${data.command_id}, 状态=${statusText}`);
                break;
        }
    }
    
    handleReceivedData(data) {
        // 更新各个数据面板
        this.updateDataPanel('head-tracking-data', data.head_tracking, (item) => 
            `偏航: ${item.yaw.toFixed(2)}°, 俯仰: ${item.pitch.toFixed(2)}°, 置信度: ${item.confidence}%`
        );
        
        this.updateDataPanel('voice-text-data', data.voice_text, (item) => 
            `操作: 0x${item.operation.toString(16).toUpperCase()}, 文本: "${item.text}"`
        );
        
        this.updateDataPanel('voice-command-data', data.voice_commands, (item) => 
            `类别: 0x${item.category.toString(16).toUpperCase()}, 操作: 0x${item.operation.toString(16).toUpperCase()}, ID: ${item.command_id}`
        );
        
        this.updateDataPanel('ack-data', data.sent_acks, (item) => 
            `确认包: ID=${item.command_id}, 状态=${item.status === 0x01 ? '成功' : '失败'}`
        );
    }
    
    updateDataPanel(panelId, data, formatter) {
        const panel = document.getElementById(panelId);
        if (data && data.length > 0) {
            panel.innerHTML = data.map(item => 
                `<div class="data-item">${formatter(item)}</div>`
            ).join('');
        } else {
            panel.innerHTML = '<div class="data-item">暂无数据</div>';
        }
    }
    
    addRealtimeData(text) {
        this.addDataToList('realtime-data', text);
    }
    
    addHeadTrackingData(text) {
        this.addDataToList('head-tracking-data', text);
    }
    
    addVoiceTextData(text) {
        this.addDataToList('voice-text-data', text);
    }
    
    addVoiceCommandData(text) {
        this.addDataToList('voice-command-data', text);
    }
    
    addAckData(text) {
        this.addDataToList('ack-data', text);
    }
    
    addDataToList(listId, text) {
        const list = document.getElementById(listId);
        const timestamp = new Date().toLocaleTimeString();
        const item = document.createElement('div');
        item.className = 'data-item';
        item.innerHTML = `[${timestamp}] ${text}`;
        
        list.insertBefore(item, list.firstChild);
        
        // 保持最多50条记录
        while (list.children.length > 50) {
            list.removeChild(list.lastChild);
        }
    }
    
    updateStats() {
        document.getElementById('vehicle-status-count').textContent = this.stats.vehicleStatusSent;
        document.getElementById('head-tracking-count').textContent = this.stats.headTrackingReceived;
        document.getElementById('voice-text-count').textContent = this.stats.voiceTextReceived;
        document.getElementById('voice-command-count').textContent = this.stats.voiceCommandReceived;
        document.getElementById('ack-sent-count').textContent = this.stats.ackSent;
    }
    
    updateConnectionStatus(connected) {
        const statusElement = document.getElementById('connection-status');
        if (!statusElement) {
            console.error('找不到connection-status元素');
            return;
        }
        
        const indicator = statusElement.querySelector('.status-indicator');
        const text = statusElement.querySelector('span:last-child');
        
        if (!indicator || !text) {
            console.error('找不到状态指示器或文本元素');
            return;
        }
        
        if (connected) {
            indicator.className = 'status-indicator status-online';
            text.textContent = '已连接';
            console.log('连接状态更新为: 已连接');
        } else {
            indicator.className = 'status-indicator status-offline';
            text.textContent = '未连接';
            console.log('连接状态更新为: 未连接');
        }
    }
    
    toggleVehicleStatus() {
        if (!this.isConnected) {
            this.log('WebSocket未连接，无法发送控制命令', 'error');
            return;
        }
        
        this.sendMessage({
            type: 'control',
            data: {
                action: 'toggle_vehicle_status'
            }
        });
    }
    
    setVehicleStatusInterval() {
        if (!this.isConnected) {
            this.log('WebSocket未连接，无法发送控制命令', 'error');
            return;
        }
        
        const interval = parseFloat(document.getElementById('vehicle-status-interval').value);
        if (isNaN(interval) || interval <= 0) {
            this.log('请输入有效的发送频率', 'error');
            return;
        }
        
        const intervalSeconds = 1 / interval;
        
        this.sendMessage({
            type: 'control',
            data: {
                action: 'set_vehicle_status_interval',
                interval: intervalSeconds
            }
        });
        
        this.log(`设置车辆状态发送频率为 ${interval}Hz`, 'info');
    }
    
    sendMessage(message) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(message));
        } else {
            this.log('WebSocket未连接，无法发送消息', 'error');
        }
    }
    
    log(message, level = 'info') {
        const logContent = document.getElementById('log-content');
        if (!logContent) {
            console.log(`[${level.toUpperCase()}] ${message}`);
            return;
        }
        
        const timestamp = new Date().toLocaleTimeString();
        const entry = document.createElement('div');
        entry.className = 'log-entry';
        
        const levelClass = `log-${level}`;
        entry.innerHTML = `
            <span class="log-timestamp">[${timestamp}]</span>
            <span class="${levelClass}">${message}</span>
        `;
        
        logContent.insertBefore(entry, logContent.firstChild);
        
        // 保持最多100条日志
        while (logContent.children.length > 100) {
            logContent.removeChild(logContent.lastChild);
        }
        
        // 同时输出到控制台
        console.log(`[${timestamp}] [${level.toUpperCase()}] ${message}`);
    }
    
    startStatusUpdate() {
        // 定期请求状态更新
        setInterval(() => {
            if (this.isConnected) {
                this.sendMessage({
                    type: 'get_status'
                });
                
                this.sendMessage({
                    type: 'get_received_data'
                });
            }
        }, 5000);
    }
}

// 页面加载完成后初始化应用
document.addEventListener('DOMContentLoaded', () => {
    console.log('页面加载完成，开始初始化应用...');
    try {
        new VehicleSimulatorApp();
        console.log('应用初始化成功');
    } catch (error) {
        console.error('应用初始化失败:', error);
    }
});
