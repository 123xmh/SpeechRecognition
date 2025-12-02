# Windows端WSL2端口转发配置说明

## 问题说明

您遇到的错误是因为批处理文件编码问题导致的。我已经提供了两个解决方案：

## 解决方案1：使用修复后的批处理文件（推荐）

### 文件位置
`src/web_monitor/tools/setup_wsl2_port_forward.bat`

### 运行步骤
1. **以管理员身份运行**（重要！）
   - 右键点击 `setup_wsl2_port_forward.bat`
   - 选择"以管理员身份运行"

2. **如果仍有问题**，请使用PowerShell脚本

## 解决方案2：使用PowerShell脚本（最稳定）

### 文件位置
`src/web_monitor/tools/setup_wsl2_port_forward.ps1`

### 运行步骤

#### 方法1：右键运行
1. **右键点击** `setup_wsl2_port_forward.ps1`
2. **选择** "使用PowerShell运行"
3. **如果出现安全策略错误**，使用下面的方法

#### 方法2：通过PowerShell运行
1. **按 `Win + X`**，选择"Windows PowerShell (管理员)"
2. **导航到脚本目录**：
   ```powershell
   cd "C:\Users\您的用户名\TK30V2\tk_chest_controller_ws\src\web_monitor\tools"
   ```
3. **设置执行策略**（如果需要）：
   ```powershell
   Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
   ```
4. **运行脚本**：
   ```powershell
   .\setup_wsl2_port_forward.ps1
   ```

## 手动配置方法（如果脚本都不工作）

### 步骤1：获取WSL2 IP地址
在WSL2中运行：
```bash
hostname -I
```
记下显示的IP地址（如：172.xx.xx.xx）

### 步骤2：配置端口转发
在Windows PowerShell（管理员）中运行：
```powershell
# 删除现有规则
netsh interface portproxy delete v4tov4 listenport=9090 listenaddress=0.0.0.0

# 添加新规则（替换WSL2_IP为实际IP）
netsh interface portproxy add v4tov4 listenport=9090 listenaddress=0.0.0.0 connectport=9090 connectaddress=WSL2_IP

# 配置防火墙
netsh advfirewall firewall add rule name="WSL2 Web Monitor" dir=in action=allow protocol=TCP localport=9090
```

### 步骤3：验证配置
```powershell
netsh interface portproxy show all
```

## 常见问题解决

### 问题1：脚本运行后显示乱码
- 使用PowerShell脚本版本
- 或者手动配置

### 问题2：权限不足
- 确保以管理员身份运行
- 右键选择"以管理员身份运行"

### 问题3：WSL2未运行
- 在Windows中运行 `wsl --status`
- 确保WSL2正在运行

### 问题4：端口被占用
- 检查9090端口是否被其他程序占用
- 可以修改web_monitor使用其他端口

## 测试连接

配置完成后，在Windows浏览器中访问：
`http://localhost:9090`

如果能看到web_monitor界面，说明配置成功！

## 注意事项

1. **每次WSL2重启后**，IP地址可能变化，需要重新配置
2. **确保Windows防火墙**允许9090端口
3. **某些防病毒软件**可能需要添加例外规则
4. **建议保存配置命令**，以便重复使用

