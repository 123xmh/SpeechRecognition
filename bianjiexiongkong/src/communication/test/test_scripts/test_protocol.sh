#!/bin/bash

# 协议测试脚本
# 用于测试communication节点的协议解析能力

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
    echo "协议测试脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助信息"
    echo "  -t, --test-type TYPE    测试类型: vehicle_state, head_tracking, voice_text, voice_command"
    echo "  -f, --frequency HZ      发送频率(默认: 1.0 Hz)"
    echo "  -d, --duration SECONDS  测试持续时间(默认: 10秒)"
    echo "  -v, --verbose           详细输出模式"
    echo ""
    echo "示例:"
    echo "  $0 -t vehicle_state -f 5.0 -d 30"
    echo "  $0 -t voice_command -f 0.5 -d 60"
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

# 运行协议测试
run_protocol_test() {
    local test_type=$1
    local frequency=$2
    local duration=$3
    local verbose=$4
    
    log_info "开始协议测试: $test_type"
    log_info "测试参数: 频率=${frequency}Hz, 持续时间=${duration}秒"
    
    # 启动测试发送器
    log_info "启动测试发送器..."
    ros2 launch test_sender test_sender.launch.py \
        test_type:="$test_type" \
        frequency:="$frequency" \
        duration:="$duration" \
        log_level:=$(if [ "$verbose" = "true" ]; then echo "debug"; else echo "info"; fi) \
        > /tmp/protocol_test.log 2>&1 &
    local sender_pid=$!
    
    # 等待测试完成
    log_info "等待测试完成..."
    wait $sender_pid
    
    # 分析结果
    analyze_protocol_results "$test_type"
}

# 分析协议测试结果
analyze_protocol_results() {
    local test_type=$1
    local log_file="/tmp/protocol_test.log"
    
    log_info "分析协议测试结果: $test_type"
    
    if [ ! -f "$log_file" ]; then
        log_error "日志文件不存在: $log_file"
        return 1
    fi
    
    # 统计数据包发送数量
    local packets_sent=$(grep -c "Sent.*packet" "$log_file" 2>/dev/null || echo "0")
    local test_completed=$(grep -c "Test completed" "$log_file" 2>/dev/null || echo "0")
    
    # 检查错误
    local errors=$(grep -c "ERROR\|Failed" "$log_file" 2>/dev/null || echo "0")
    local warnings=$(grep -c "WARN" "$log_file" 2>/dev/null || echo "0")
    
    # 显示结果
    echo ""
    log_info "=== 协议测试结果统计 ==="
    echo "测试类型: $test_type"
    echo "发送数据包: $packets_sent"
    echo "测试完成: $test_completed"
    echo "错误数量: $errors"
    echo "警告数量: $warnings"
    
    # 判断测试是否成功
    if [ "$errors" -eq 0 ] && [ "$packets_sent" -gt 0 ] && [ "$test_completed" -gt 0 ]; then
        log_success "协议测试通过"
        return 0
    else
        log_error "协议测试失败"
        if [ "$errors" -gt 0 ]; then
            log_error "发现错误:"
            grep "ERROR\|Failed" "$log_file" | head -5
        fi
        return 1
    fi
}

# 运行所有协议测试
run_all_protocol_tests() {
    local frequency=${1:-1.0}
    local duration=${2:-10.0}
    local verbose=$3
    
    local test_types=("vehicle_state" "head_tracking" "voice_text" "voice_command")
    
    local total_tests=${#test_types[@]}
    local passed_tests=0
    local failed_tests=0
    
    log_info "开始运行所有协议测试 (共 $total_tests 个)"
    
    for test_type in "${test_types[@]}"; do
        echo ""
        log_info "=== 运行协议测试: $test_type ==="
        
        if run_protocol_test "$test_type" "$frequency" "$duration" "$verbose"; then
            ((passed_tests++))
        else
            ((failed_tests++))
        fi
        
        # 测试间隔
        sleep 2
    done
    
    # 显示总结
    echo ""
    log_info "=== 协议测试总结 ==="
    echo "总测试数: $total_tests"
    echo "通过: $passed_tests"
    echo "失败: $failed_tests"
    
    if [ "$failed_tests" -eq 0 ]; then
        log_success "所有协议测试通过"
        return 0
    else
        log_error "有 $failed_tests 个协议测试失败"
        return 1
    fi
}

# 主函数
main() {
    local test_type=""
    local frequency=1.0
    local duration=10.0
    local verbose=false
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -t|--test-type)
                test_type="$2"
                shift 2
                ;;
            -f|--frequency)
                frequency="$2"
                shift 2
                ;;
            -d|--duration)
                duration="$2"
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
        run_all_protocol_tests "$frequency" "$duration" "$verbose"
    else
        run_protocol_test "$test_type" "$frequency" "$duration" "$verbose"
    fi
}

# 运行主函数
main "$@"
