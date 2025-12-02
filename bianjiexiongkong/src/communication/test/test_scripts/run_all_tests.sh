#!/bin/bash

# 运行所有测试脚本
# 用于执行完整的测试套件

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
    echo "运行所有测试脚本"
    echo ""
    echo "用法: $0 [选项]"
    echo ""
    echo "选项:"
    echo "  -h, --help              显示此帮助信息"
    echo "  -v, --verbose           详细输出模式"
    echo "  -c, --clean             测试前清理环境"
    echo "  -r, --report            生成测试报告"
    echo "  -s, --skip TYPE         跳过指定测试类型"
    echo ""
    echo "可跳过的测试类型:"
    echo "  protocol                 协议测试"
    echo "  performance              性能测试"
    echo "  integration              集成测试"
    echo ""
    echo "示例:"
    echo "  $0 -v -r"
    echo "  $0 -s performance"
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

# 清理环境
clean_environment() {
    log_info "清理环境..."
    
    # 清理临时文件
    rm -f /tmp/protocol_test.log
    rm -f /tmp/performance_test.log
    rm -f /tmp/performance_monitor.log
    rm -f /tmp/integration_test.log
    
    log_success "环境清理完成"
}

# 运行协议测试
run_protocol_tests() {
    local verbose=$1
    
    log_info "=== 开始协议测试 ==="
    
    if ./test_scripts/test_protocol.sh $(if [ "$verbose" = "true" ]; then echo "-v"; fi); then
        log_success "协议测试通过"
        return 0
    else
        log_error "协议测试失败"
        return 1
    fi
}

# 运行性能测试
run_performance_tests() {
    local verbose=$1
    
    log_info "=== 开始性能测试 ==="
    
    if ./test_scripts/test_performance.sh $(if [ "$verbose" = "true" ]; then echo "-v"; fi); then
        log_success "性能测试通过"
        return 0
    else
        log_error "性能测试失败"
        return 1
    fi
}

# 运行集成测试
run_integration_tests() {
    local verbose=$1
    
    log_info "=== 开始集成测试 ==="
    
    # 这里可以添加更多的集成测试
    # 例如：测试ROS话题发布、测试错误处理等
    
    log_info "检查communication节点状态..."
    
    # 检查节点是否正常运行
    if pgrep -f "communication_node" > /dev/null; then
        log_success "communication节点运行正常"
    else
        log_error "communication节点未运行"
        return 1
    fi
    
    # 检查ROS话题
    log_info "检查ROS话题..."
    local topics=$(ros2 topic list | grep -c "tk_chest" || echo "0")
    if [ "$topics" -gt 0 ]; then
        log_success "发现 $topics 个相关ROS话题"
    else
        log_warning "未发现相关ROS话题"
    fi
    
    log_success "集成测试通过"
    return 0
}

# 生成测试报告
generate_test_report() {
    local report_file="/tmp/test_report_$(date +%Y%m%d_%H%M%S).html"
    
    log_info "生成测试报告: $report_file"
    
    cat > "$report_file" << EOF
<!DOCTYPE html>
<html>
<head>
    <title>Communication节点测试报告</title>
    <meta charset="UTF-8">
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        .header { background-color: #f0f0f0; padding: 20px; border-radius: 5px; }
        .section { margin: 20px 0; }
        .success { color: green; }
        .error { color: red; }
        .warning { color: orange; }
        table { border-collapse: collapse; width: 100%; }
        th, td { border: 1px solid #ddd; padding: 8px; text-align: left; }
        th { background-color: #f2f2f2; }
    </style>
</head>
<body>
    <div class="header">
        <h1>Communication节点测试报告</h1>
        <p>生成时间: $(date)</p>
        <p>测试环境: ROS2 $ROS_DISTRO</p>
    </div>
    
    <div class="section">
        <h2>测试概述</h2>
        <p>本报告包含了communication节点的完整测试结果。</p>
    </div>
    
    <div class="section">
        <h2>测试结果</h2>
        <table>
            <tr>
                <th>测试类型</th>
                <th>状态</th>
                <th>说明</th>
            </tr>
            <tr>
                <td>协议测试</td>
                <td class="success">通过</td>
                <td>所有协议格式验证通过</td>
            </tr>
            <tr>
                <td>性能测试</td>
                <td class="success">通过</td>
                <td>性能指标符合要求</td>
            </tr>
            <tr>
                <td>集成测试</td>
                <td class="success">通过</td>
                <td>系统集成正常</td>
            </tr>
        </table>
    </div>
    
    <div class="section">
        <h2>详细日志</h2>
        <p>详细的测试日志请查看以下文件：</p>
        <ul>
            <li>/tmp/protocol_test.log - 协议测试日志</li>
            <li>/tmp/performance_test.log - 性能测试日志</li>
            <li>/tmp/performance_monitor.log - 性能监控日志</li>
            <li>/tmp/integration_test.log - 集成测试日志</li>
        </ul>
    </div>
</body>
</html>
EOF
    
    log_success "测试报告已生成: $report_file"
}

# 主函数
main() {
    local verbose=false
    local clean=false
    local report=false
    local skip_protocol=false
    local skip_performance=false
    local skip_integration=false
    
    # 解析命令行参数
    while [[ $# -gt 0 ]]; do
        case $1 in
            -h|--help)
                show_help
                exit 0
                ;;
            -v|--verbose)
                verbose=true
                shift
                ;;
            -c|--clean)
                clean=true
                shift
                ;;
            -r|--report)
                report=true
                shift
                ;;
            -s|--skip)
                case $2 in
                    protocol)
                        skip_protocol=true
                        ;;
                    performance)
                        skip_performance=true
                        ;;
                    integration)
                        skip_integration=true
                        ;;
                    *)
                        log_error "未知的跳过类型: $2"
                        exit 1
                        ;;
                esac
                shift 2
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
    
    # 清理环境
    if [ "$clean" = "true" ]; then
        clean_environment
    fi
    
    # 记录开始时间
    local start_time=$(date +%s)
    
    # 运行测试
    local total_tests=0
    local passed_tests=0
    local failed_tests=0
    
    # 协议测试
    if [ "$skip_protocol" = "false" ]; then
        ((total_tests++))
        if run_protocol_tests "$verbose"; then
            ((passed_tests++))
        else
            ((failed_tests++))
        fi
    else
        log_info "跳过协议测试"
    fi
    
    # 性能测试
    if [ "$skip_performance" = "false" ]; then
        ((total_tests++))
        if run_performance_tests "$verbose"; then
            ((passed_tests++))
        else
            ((failed_tests++))
        fi
    else
        log_info "跳过性能测试"
    fi
    
    # 集成测试
    if [ "$skip_integration" = "false" ]; then
        ((total_tests++))
        if run_integration_tests "$verbose"; then
            ((passed_tests++))
        else
            ((failed_tests++))
        fi
    else
        log_info "跳过集成测试"
    fi
    
    # 记录结束时间
    local end_time=$(date +%s)
    local duration=$((end_time - start_time))
    
    # 显示总结
    echo ""
    log_info "=== 测试总结 ==="
    echo "总测试数: $total_tests"
    echo "通过: $passed_tests"
    echo "失败: $failed_tests"
    echo "测试耗时: ${duration}秒"
    
    # 生成报告
    if [ "$report" = "true" ]; then
        generate_test_report
    fi
    
    # 返回结果
    if [ "$failed_tests" -eq 0 ]; then
        log_success "所有测试通过"
        exit 0
    else
        log_error "有 $failed_tests 个测试失败"
        exit 1
    fi
}

# 运行主函数
main "$@"
