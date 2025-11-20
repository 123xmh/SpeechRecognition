#ifndef ARTIFICIALHORIZON_H
#define ARTIFICIALHORIZON_H

#include <QWidget>
#include <QPainter>
#include <QTimer>

class ArtificialHorizon : public QWidget {
    Q_OBJECT
public:
    explicit ArtificialHorizon(QWidget *parent = nullptr);
    ~ArtificialHorizon();

    // 设置角度（偏航角yaw不影响显示）
    void setAngles(float pitch, float roll, float yaw = 0.0f);

    // 模拟控制接口
    void startSimulation();
    void stopSimulation();
    bool isSimulating() const;

    void randomizeAngles();
    void resetAngles();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    void updateSimulation();

    const float MAX_PITCH = 45.0f;  // 最大俯仰显示角度
    float pitch;                   // 俯仰角（度）
    float roll;                    // 横滚角（度）

    QTimer *simulationTimer;        // 数据模拟定时器
    bool simulating = false;        // 是否正在模拟
};

#endif // ARTIFICIALHORIZON_H
