#include "widget.h"
#include "ui_widget.h"

#include <QDebug>
#include <QDateTime>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>

Widget::Widget(QWidget *parent)
    : QWidget(parent),
    ui(new Ui::Widget),
    isRecording(true),
    currentProtocol("无"),
    lastCurrentText("")
{
    ui->setupUi(this);

    // 设置UI元素
    connect(ui->toggleButton, &QPushButton::clicked, this, &Widget::onToggleButtonClicked);

    // 启动Python脚本
    asrProcess = new QProcess(this);
    connect(asrProcess, &QProcess::readyReadStandardOutput, this, &Widget::readProcessOutput);
    connect(asrProcess, &QProcess::readyReadStandardError, this, &Widget::readProcessOutput);
    connect(asrProcess, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &Widget::processFinished);

    ASRStart();
}

Widget::~Widget()
{
    if (asrProcess && asrProcess->state() == QProcess::Running) {
        asrProcess->terminate();
        if (!asrProcess->waitForFinished(3000)) {
            asrProcess->kill();
        }
    }
    delete ui;
}
// 调用语音识别代码
void Widget::ASRStart()
{
    // 检查Python脚本是否存在
    QString scriptPath = "/home/zkxt/Documents/TK30/speech-recognition-from-microphone-with-endpoint-detection.py"; // 替换为实际路径
    if (!QFile::exists(scriptPath)) {
        QMessageBox::critical(this, "错误", "未找到Python脚本文件");
        return;
    }

    // 设置Python脚本路径（根据实际位置修改）
    QString program = "/home/zkxt/Downloads/yes/envs/rknn/bin/python";
    QStringList arguments;
    arguments << scriptPath
              << "--tokens" << "/home/zkxt/Documents/TK30/moder/tokens.txt"
              << "--encoder" << "/home/zkxt/Documents/TK30/moder/encoder.rknn"
              << "--decoder" << "/home/zkxt/Documents/TK30/moder/decoder.rknn"
              << "--joiner" << "/home/zkxt/Documents/TK30/moder/joiner.rknn"
              << "--provider" << "rknn"
              << "--hr-dict-dir" << "/home/zkxt/Documents/TK30/PhraseReplacement/dict"
              << "--hr-lexicon" << "/home/zkxt/Documents/TK30/PhraseReplacement/lexicon.txt"
              << "--hr-rule-fsts" << "/home/zkxt/Documents/TK30/PhraseReplacement/replace.fst";
//              << "--udp-ip" << "127.0.0.1"
//              << "--udp-port" << "6000"
//              << "--ack-port" << "6001";

    // 启动前设置工作目录（如果需要）
    // asrProcess->setWorkingDirectory("/path/to/your/working/dir");

    asrProcess->start(program, arguments);

    if (!asrProcess->waitForStarted(5000)) {
        appendStyledMessage("错误: 无法启动语音识别程序", true);
        ui->toggleButton->setEnabled(false);
    } else {
        appendStyledMessage("语音识别程序已启动...");
    }
}

// 添加气泡特效消息的函数
void Widget::appendStyledMessage(const QString &message, bool isError)
{
    // 如果消息仅包含空格或不可见字符，也跳过
    if (message.trimmed().isEmpty()) {
        return;
    }

    QTextCursor cursor(ui->historyTextEdit->textCursor());
    cursor.movePosition(QTextCursor::End);

    // 添加边距和圆角效果（通过HTML实现）
    QString bubbleHtml = QString("<div style=\"background-color: %1; color: %2; border-radius: 12px; "
                                 "padding: 8px 12px; margin: 4px 0; "
                                 "border: 10px solid #B0C4DE;\">%3</div>")
                         .arg(isError ? "#FFE0E0" : "#15B5F9")
                         .arg(isError ? "#8B0000" : "black")
                         .arg(message.toHtmlEscaped());

    // 创建段落格式并设置行间距
    QTextBlockFormat blockFormat;
    blockFormat.setLineHeight(50, QTextBlockFormat::FixedHeight);

    // 将格式应用到所有选中段落
    cursor.mergeBlockFormat(blockFormat);

    cursor.insertText("\n");

    cursor.insertHtml(bubbleHtml);


    // 自动滚动到底部
    QScrollBar *scrollbar = ui->historyTextEdit->verticalScrollBar();
    if (scrollbar) {
        scrollbar->setValue(scrollbar->maximum());
    }
}
// 语音识别切换按钮
void Widget::onToggleButtonClicked()
{
    isRecording = !isRecording;

    if (isRecording) {
        ui->toggleButton->setText("暂停录音");
        ui->toggleButton->setStyleSheet("background-color: red; color: white;");
        appendStyledMessage("开始录音...");
    } else {
        ui->toggleButton->setText("开始录音");
        ui->toggleButton->setStyleSheet("background-color: green; color: white;");
        appendStyledMessage("暂停录音...");
    }

    // 发送换行符给Python进程
    if (asrProcess && asrProcess->state() == QProcess::Running) {
        asrProcess->write("\n");
        asrProcess->waitForBytesWritten(100); // 等待写入完成
    }
}

void Widget::readProcessOutput()
{
    // 读取标准输出
    QByteArray output = asrProcess->readAllStandardOutput();
    if (!output.isEmpty()) {
        QString text = QString::fromLocal8Bit(output);
        qDebug() << "收到标准输出:" << text;
        parseOutput(text);
        updateDisplay();
    }

    // 读取标准错误
    QByteArray error = asrProcess->readAllStandardError();
    if (!error.isEmpty()) {
        QString errorText = QString::fromLocal8Bit(error);
        qDebug() << "收到标准错误:" << errorText;
        appendStyledMessage("错误: " + errorText, true);
    }
}

void Widget::parseOutput(const QString &output)
{
    // 使用正则表达式解析输出
    QRegularExpression recognizingRegex("Recognizing[:：]\\s*(.*)$"); // 修复：兼容中英文冒号

    // 过滤空行
    QStringList lines = output.split('\n', QString::SkipEmptyParts);

    // 协议匹配标志
    bool protocolUpdated = false;

    for (const QString &line : lines) {
        // 跳过空行
        if (line.trimmed().isEmpty()) continue;

//        qDebug() << "当前信息：" << lines << endl;

        // 检测当前识别内容
        QRegularExpressionMatch recognizingMatch = recognizingRegex.match(line);
        if (recognizingMatch.hasMatch()) {
            QString newText = recognizingMatch.captured(1).trimmed();
            // 只有当内容改变时才更新
            if (newText != currentText) {
                currentText = newText;
            }
            continue;
        }

        // 其他信息（标题、分隔线等）
        if (!line.contains("=== Speech Recognition") &&
            !line.contains("---") && !line.contains("Time:")) {

            // 只有当信息与上次不同时才添加
            if (line != lastGeneral) {

                if (line.contains("    ")){
                    QStringList lineList = line.split("    ");
                    qDebug() << "当前信息：" << lineList << endl;
                    if (line.contains("未确认")){
                        appendStyledMessage(lineList[0],"False");
                    } else {
                        appendStyledMessage(lineList[0]);
                    }
                } else {
                    appendStyledMessage(line);
                }
                lastGeneral = line;
            }
        }
    }

    // 如果有协议更新，强制刷新UI
    if (protocolUpdated) {
        updateDisplay();
    }
}

void Widget::updateDisplay()
{
    // 只有当识别内容改变时才更新当前识别显示
    if (currentText != lastCurrentText) {
        if (currentText.length() > 10){
            ui->currentLabel->setText("当前识别: ..." + currentText.right(10));
        } else {
            ui->currentLabel->setText("当前识别: " + currentText);
        }

        lastCurrentText = currentText;
    }

    // 自动滚动历史记录到底部
    QScrollBar *scrollbar = ui->historyTextEdit->verticalScrollBar();
    if (scrollbar) {
        scrollbar->setValue(scrollbar->maximum());
    }
}

void Widget::processFinished(int exitCode, QProcess::ExitStatus exitStatus)
{
    Q_UNUSED(exitStatus)
    appendStyledMessage(QString("语音识别进程已退出，代码: %1").arg(exitCode));
    ui->toggleButton->setEnabled(false);

    // 清空当前显示
    currentProtocol = "无";
    currentText = "";
    updateDisplay();
}
