#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QOpenGLWindow>
#include <QSurfaceFormat>
#include <QOpenGLFunctions>
#include <QtOpenGL>
#include <GL/glu.h>
#include <QCursor>
#include "myudp.h"
#include <QDebug>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QOpenGLWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseHoldEvent(QCursor *cursor);
    void wheelEvent(QWheelEvent *event) override;
    virtual void initializeGL() override;
    virtual void resizeGL(int w, int h) override;
    virtual void paintGL() override;
    void resizeEvent(QResizeEvent *event) override;
    void paintEvent(QPaintEvent *event) override;
    QQuaternion quatMultiply(QQuaternion q_left, QQuaternion q_right);


public slots:
    void UpdateAnimation();

private:
    // Ui::MainWindow *ui;
    QOpenGLContext *context;
    QOpenGLFunctions *openGLFunctions;

    MyUDP m_client;

    bool mouseHold = false;
    bool mouseHoldFirstCall;
    QPoint mousePos;
    QPoint lastMousePos;
    QQuaternion rotationQuatTotal = QQuaternion::fromEulerAngles({110,0,220});
    QQuaternion rotationQuatLast = rotationQuatTotal;
    float zoomScale = 1;

    int frameCount = 0;
    QTime frameTime;
    double fps;

};
#endif // MAINWINDOW_H
