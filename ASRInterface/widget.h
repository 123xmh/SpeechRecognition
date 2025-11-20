#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include <QProcess>
#include <QTextEdit>
#include <QPushButton>
#include <QLabel>
#include <QRegularExpression>
#include <QScrollBar>
#include <QTextCursor>  // 添加文本光标支持
#include <QTextDocumentFragment> // 添加富文本支持
#include <QTextBlock> // 添加文本块支持

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class ArtificialHorizon;

class Widget : public QWidget
{
    Q_OBJECT

public:
    explicit Widget(QWidget *parent = nullptr);
    ~Widget();

private slots:
    void onToggleButtonClicked();
    void readProcessOutput();
    void processFinished(int exitCode, QProcess::ExitStatus exitStatus);
//    void updateHorizon(); // 更新水平球数据

private:
    void ASRStart();

private:
    Ui::Widget *ui;
    QProcess *asrProcess;
    bool isRecording;
    QString currentProtocol;
    QString currentText;
    QString lastCurrentText;   // 添加：上次显示的当前识别内容
    QString lastHistory;       // 添加：上次显示的历史记录
    QString lastGeneral;       // 添加：上次显示的一般信息
    QString lastError;         // 添加：上次显示的错误信息

private:
    void updateDisplay();
    void parseOutput(const QString &output);
    void appendStyledMessage(const QString &message, bool isError = false); // 添加气泡特效

private:

};
#endif // WIDGET_H
