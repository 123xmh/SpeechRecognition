#!/bin/bash

# 性能测试脚本
# 用于测试communication节点的性能表现

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 显示帮助信息
show_help() {
    echo "性能测试脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助信息"
    echo "  -f, --frequency HZ      发送频率(默认: 10.0 Hz)"
    echo "  -d, --duration SECONDS  测试持续时间(默认: 60秒)"
    echo "  -t, --test-type TYPE    测试类型: high_freq, stress, burst"
    echo "  -v, --verbose           详细输出模式"
    echo ""
    echo "测试类型:"
    echo "  high_freq               高频测试 (50Hz)"
    echo "  stress                  压力测试 (100Hz)"
    echo "  burst                   突发测试 (间歇性高频)"
    echo ""
    echo "示例:"
    echo "  $0 -t high_freq -f 50.0 -d 30"
    echo "  $0 -t stress -f 100.0 -d 60"
}

# 检查依赖
check_dependencies() {
    log_info "检查依赖..."
    
    if [ -z "$ROS_DISTRO" ]; then
        log_error "ROS2环境未设置"
        exit 1
    fi
    
    # 检查communication节点是否在运行
    if ! pgrep -f "communication_node" > /dev/null; then
        log_error "communication节点未运行，请先启动communication节点"
        log_info "启动命令: ros2 run communication communication_node"
        exit 1
    fi
    
    # 检查test_sender包是否存在
    if ! ros2 pkg list | grep -q "test_sender"; then
        log_error "test_sender包未找到，请先编译工作空间"
        exit 1
    fi
    
    log_success "依赖检查通过"
}

# 监控系统资源
monitor_system_resources() {
    local duration=$1
    local log_file="/tmp/performance_monitor.log"
    
    log_info "开始监控系统资源..."
    
    # 获取communication节点PID
    local comm_pid=$(pgrep -f "communication_node")
    if [ -z "$comm_pid" ]; then
        log_error "无法找到communication节点PID"
        return 1
    fi
    
    # 监控资源使用情况
    {
        echo "timestamp,cpu_percent,memory_mb,packets_received"
        for ((i=0; i<duration; i++)); do
            local timestamp=$(date +%s)
            local cpu_percent=$(ps -p $comm_pid -o %cpu --no-headers | tr -d ' ')
            local memory_mb=$(ps -p $comm_pid -o rss --no-headers | awk '{print $1/1024}')
            local packets_received=$(netstat -su | grep "packets received" | awk '{print $1}')
            echo "$timestamp,$cpu_percent,$memory_mb,$packets_received"
            sleep 1
        done
    } > "$log_file"
    
    log_info "系统资源监控完成，数据保存到: $log_file"
}

# 运行性能测试
run_performance_test() {
    local test_type=$1
    local frequency=$2
    local duration=$3
    local verbose=$4
    
    log_info "开始性能测试: $test_type"
    log_info "测试参数: 频率=${frequency}Hz, 持续时间=${duration}秒"
    
    # 启动系统资源监控
    monitor_system_resources $duration &
    local monitor_pid=$!
    
    # 启动测试发送器
    log_info "启动测试发送器..."
    ros2 launch test_sender test_sender.launch.py \
        test_type:="vehicle_state" \
        frequency:="$frequency" \
        duration:="$duration" \
        log_level:=$(if [ "$verbose" = "true" ]; then echo "debug"; else echo "info"; fi) \
        > /tmp/performance_test.log 2>&1 &
    local sender_pid=$!
    
    # 等待测试完成
    log_info "等待测试完成..."
    wait $sender_pid
    wait $monitor_pid
    
    # 分析结果
    analyze_performance_results "$test_type" "$frequency" "$duration"
}

# 分析性能测试结果
analyze_performance_results() {
    local test_type=$1
    local frequency=$2
    local duration=$3
    local test_log="/tmp/performance_test.log"
    local monitor_log="/tmp/performance_monitor.log"
    
    log_info "分析性能测试结果: $test_type"
    
    if [ ! -f "$test_log" ] || [ ! -f "$monitor_log" ]; then
        log_error "日志文件不存在"
        return 1
    fi
    
    # 统计数据包发送数量
    local packets_sent=$(grep -c "Sent.*packet" "$test_log" 2>/dev/null || echo "0")
    local test_completed=$(grep -c "Test completed" "$test_log" 2>/dev/null || echo "0")
    
    # 计算性能指标
    local expected_packets=$(echo "$frequency * $duration" | bc)
    local actual_throughput=$(echo "scale=2; $packets_sent / $duration" | bc)
    local efficiency=$(echo "scale=2; $packets_sent * 100 / $expected_packets" | bc)
    
    # 分析系统资源使用
    local avg_cpu=$(tail -n +2 "$monitor_log" | cut -d',' -f2 | awk '{sum+=$1} END {print sum/NR}')
    local max_cpu=$(tail -n +2 "$monitor_log" | cut -d',' -f2 | sort -n | tail -1)
    local avg_memory=$(tail -n +2 "$monitor_log" | cut -d',' -f3 | awk '{sum+=$1} END {print sum/NR}')
    local max_memory=$(tail -n +2 "$monitor_log" | cut -d',' -f3 | sort -n | tail -1)
    
    # 显示结果
    echo ""
    log_info "=== 性能测试结果统计 ==="
    echo "测试类型: $test_type"
    echo "测试频率: ${frequency}Hz"
    echo "测试时长: ${duration}秒"
    echo "预期数据包: $expected_packets"
    echo "实际发送: $packets_sent"
    echo "实际吞吐量: ${actual_throughput} 包/秒"
    echo "发送效率: ${efficiency}%"
    echo "平均CPU使用率: ${avg_cpu}%"
    echo "最大CPU使用率: ${max_cpu}%"
    echo "平均内存使用: ${avg_memory}MB"
    echo "最大内存使用: ${max_memory}MB"
    
    # 性能评估
    local performance_score=0
    
    # 吞吐量评估
    if (( $(echo "$efficiency >= 95" | bc -l) )); then
        log_success "吞吐量性能: 优秀 (${efficiency}%)"
        ((performance_score += 3))
    elif (( $(echo "$efficiency >= 90" | bc -l) )); then
        log_warning "吞吐量性能: 良好 (${efficiency}%)"
        ((performance_score += 2))
    elif (( $(echo "$efficiency >= 80" | bc -l) )); then
        log_warning "吞吐量性能: 一般 (${efficiency}%)"
        ((performance_score += 1))
    else
        log_error "吞吐量性能: 较差 (${efficiency}%)"
    fi
    
    # CPU使用率评估
    if (( $(echo "$max_cpu <= 50" | bc -l) )); then
        log_success "CPU使用率: 优秀 (最大${max_cpu}%)"
        ((performance_score += 2))
    elif (( $(echo "$max_cpu <= 70" | bc -l) )); then
        log_warning "CPU使用率: 良好 (最大${max_cpu}%)"
        ((performance_score += 1))
    else
        log_error "CPU使用率: 较高 (最大${max_cpu}%)"
    fi
    
    # 内存使用评估
    if (( $(echo "$max_memory <= 100" | bc -l) )); then
        log_success "内存使用: 优秀 (最大${max_memory}MB)"
        ((performance_score += 2))
    elif (( $(echo "$max_memory <= 200" | bc -l) )); then
        log_warning "内存使用: 良好 (最大${max_memory}MB)"
        ((performance_score += 1))
    else
        log_error "内存使用: 较高 (最大${max_memory}MB)"
    fi
    
    # 总体评估
    echo ""
    log_info "=== 性能评估 ==="
    echo "性能得分: $performance_score/7"
    
    if [ $performance_score -ge 6 ]; then
        log_success "性能测试通过 - 优秀"
        return 0
    elif [ $performance_score -ge 4 ]; then
        log_warning "性能测试通过 - 良好"
        return 0
    else
        log_error "性能测试失败 - 需要优化"
        return 1
    fi
}

# 运行所有性能测试
run_all_performance_tests() {
    local verbose=$1
    
    local test_configs=(
        "high_freq:50.0:30"
        "stress:100.0:60"
        "burst:200.0:10"
    )
    
    local total_tests=${#test_configs[@]}
    local passed_tests=0
    local failed_tests=0
    
    log_info "开始运行所有性能测试 (共 $total_tests 个)"
    
    for config in "${test_configs[@]}"; do
        IFS=':' read -r test_type frequency duration <<< "$config"
        
        echo ""
        log_info "=== 运行性能测试: $test_type ==="
        
        if run_performance_test "$test_type" "$frequency" "$duration" "$verbose"; then
            ((passed_tests++))
        else
            ((failed_tests++))
        fi
        
        # 测试间隔
        sleep 5
    done
    
    # 显示总结
    echo ""
    log_info "=== 性能测试总结 ==="
    echo "总测试数: $total_tests"
    echo "通过: $passed_tests"
    echo "失败: $failed_tests"
    
    if [ "$failed_tests" -eq 0 ]; then
        log_success "所有性能测试通过"
        return 0
    else
        log_error "有 $failed_tests 个性能测试失败"
        return 1
    fi
}

# 主函数
main() {
    local test_type=""
    local frequency=10.0
    local duration=60.0
    local verbose=false
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -f|--frequency)
                frequency="$2"
                shift 2
                ;;
            -d|--duration)
                duration="$2"
                shift 2
                ;;
            -t|--test-type)
                test_type="$2"
                shift 2
                ;;
            -v|--verbose)
                verbose=true
                shift
                ;;
            -*)
                log_error "未知选项: $1"
                show_help
                exit 1
                ;;
            *)
                log_error "未知参数: $1"
                show_help
                exit 1
                ;;
        esac
    done
    
    # 检查依赖
    check_dependencies
    
    # 运行测试
    if [ -z "$test_type" ]; then
        run_all_performance_tests "$verbose"
    else
        run_performance_test "$test_type" "$frequency" "$duration" "$verbose"
    fi
}

# 运行主函数
main "$@"
