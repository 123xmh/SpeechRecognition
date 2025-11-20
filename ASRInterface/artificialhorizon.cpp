#include "artificialhorizon.h"
#include <QtMath>
#include <QDateTime>
#include <QRandomGenerator>
#include <QDebug>

ArtificialHorizon::ArtificialHorizon(QWidget *parent)
    : QWidget(parent), pitch(0), roll(0)
{
     setMinimumSize(200, 200);  // 设置最小尺寸确保可见性
     setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

     // 创建模拟定时器
     simulationTimer = new QTimer(this);
     connect(simulationTimer, &QTimer::timeout, this, &ArtificialHorizon::updateSimulation);

     // 初始随机种子
     QRandomGenerator::securelySeeded();

     // 开始时自动启动模拟
     startSimulation();
}

ArtificialHorizon::~ArtificialHorizon()
{
    stopSimulation();
}

void ArtificialHorizon::setAngles(float pitch, float roll, float /* yaw */)
{
    // 停止模拟（如果正在运行）
    if(simulating) stopSimulation();

    // 存储角度值并触发重绘
    this->pitch = pitch;
    this->roll = roll;
    update();
}

void ArtificialHorizon::startSimulation()
{
    if(simulating) return;

    simulating = true;
    simulationTimer->start(50); // 每50ms更新一次
}

void ArtificialHorizon::stopSimulation()
{
    simulating = false;
    simulationTimer->stop();
}

bool ArtificialHorizon::isSimulating() const
{
    return simulating;
}

void ArtificialHorizon::randomizeAngles()
{
    static QRandomGenerator rand;

    // 随机生成角度（-45° 到 +45° 俯仰，-90° 到 +90° 横滚）
    pitch = rand.bounded(-45.0f, 45.0f);
    roll = rand.bounded(-90.0f, 90.0f);

    update();
}

void ArtificialHorizon::resetAngles()
{
    pitch = 0;
    roll = 0;
    update();
}

void ArtificialHorizon::updateSimulation()
{
    static qint64 lastTime = QDateTime::currentMSecsSinceEpoch();
    static float t = 0;

    qint64 now = QDateTime::currentMSecsSinceEpoch();
    float delta = (now - lastTime) / 1000.0f; // 转换为秒
    lastTime = now;

    t += delta;

    // 生成平滑的姿态变化
    float newPitch = sin(t * 0.8) * 20.0f;        // 俯仰在±20°之间摆动
    float newRoll = cos(t * 0.5) * 60.0f;          // 横滚在±60°之间摆动

    // 平滑过渡
    pitch += (newPitch - pitch) * 0.2;
    roll += (newRoll - roll) * 0.2;

    // 触发重绘
    update();
}

void ArtificialHorizon::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // 计算控件中心点
    const int side = qMin(width(), height());
    const int radius = side * 0.4;
    const QPointF center(width() / 2.0f, height() / 2.0f);

    // === 绘制背景 ===
    painter.fillRect(rect(), Qt::black);

    // === 坐标系旋转 ===
    painter.save();
    painter.translate(center);
    painter.rotate(-roll);  // Qt坐标系Y轴向下，所以取负值

    // === 计算俯仰偏移 ===
    const float normalizedPitch = qBound(-MAX_PITCH, pitch, MAX_PITCH);
    const float offsetY = (normalizedPitch / MAX_PITCH) * radius * 0.6;

    // === 绘制天空 ===
    QLinearGradient skyGrad(0, -radius, 0, offsetY);
    skyGrad.setColorAt(0, QColor(65, 135, 245));   // 顶部深蓝色
    skyGrad.setColorAt(1, QColor(170, 230, 255)); // 底部浅蓝色
    painter.setPen(Qt::NoPen);
    painter.setBrush(skyGrad);
    painter.drawRect(-radius * 1.5, -radius * 1.5,
                     radius * 3.0, radius * 1.5 + offsetY);

    // === 绘制地面 ===
    QLinearGradient groundGrad(0, offsetY, 0, radius);
    groundGrad.setColorAt(0, QColor(139, 69, 19));   // 顶部土褐色
    groundGrad.setColorAt(1, QColor(210, 180, 140)); // 底部浅褐色
    painter.setBrush(groundGrad);
    painter.drawRect(-radius * 1.5, offsetY,
                     radius * 3.0, radius * 1.5);

    // === 绘制地平线 ===
    QPen horizonPen(Qt::black);
    horizonPen.setWidth(2);
    painter.setPen(horizonPen);
    painter.drawLine(-radius, offsetY, radius, offsetY);

    // === 恢复坐标系 ===
    painter.restore();

    // === 绘制固定标志（外圆和十字） ===
    QPen fixedPen(Qt::white);
    fixedPen.setWidth(2);
    painter.setPen(fixedPen);
    painter.setBrush(Qt::transparent);
    painter.drawEllipse(center, radius, radius);

    // 十字瞄准线
    const int lineLen = radius * 0.7;
    painter.drawLine(center.x() - lineLen, center.y(),
                     center.x() + lineLen, center.y());
    painter.drawLine(center.x(), center.y() - lineLen/2,
                     center.x(), center.y() + lineLen/2);

    // === 显示角度数值 ===
    painter.setPen(Qt::white);

    QFont infoFont;
    infoFont.setPointSize(10);
    painter.setFont(infoFont);

    painter.drawText(10, 20,
        QString("俯仰: %1° 横滚: %2°")
            .arg(pitch, 0, 'f', 1)
            .arg(roll, 0, 'f', 1));

    if(simulating) {
        painter.drawText(10, height() - 10, "模拟模式运行中");
    }
}
