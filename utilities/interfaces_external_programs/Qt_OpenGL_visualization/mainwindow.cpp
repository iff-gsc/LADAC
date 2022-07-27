#include "mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
{
    setSurfaceType(QWindow::OpenGLSurface);

    QSurfaceFormat format;
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setVersion(2,1);
    setFormat(format);

    context = new QOpenGLContext;
    context->setFormat(format);
    context->create();
    context->makeCurrent(this);

    openGLFunctions = context->functions();

    QTimer *timer = new QTimer(this);

    connect(timer, SIGNAL(timeout()), this, SLOT(UpdateAnimation()));

    timer->start();

}

MainWindow::~MainWindow()
{
}

void MainWindow::initializeGL()
{
    glEnable(GL_DEPTH_TEST);
    resizeGL(this->width(), this->height());
    frameTime.start();
}

void MainWindow::resizeGL(int w, int h)
{
    // Set viewport
    glViewport(0,0,w,h);
    qreal aspectratio = qreal(w)/qreal(h);

    // Initialize projection matrix
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    gluPerspective(75, aspectratio, 0.1, 400000000);

    // Initialize model view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

}

void MainWindow::paintGL()
{

    glClearColor(182.0/255.0,201.0/255.0,214.0/255.0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable( GL_BLEND );
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBlendEquation(GL_FUNC_ADD);

    // Reset model view matrix
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // 3D transformation
    glTranslatef(0.0, 0.0, -50.0*zoomScale);
    QCursor cursor;
    if (mouseHold) {
        MainWindow::mouseHoldEvent(&cursor);
    }
    // rotation axis and angle are needed for glRotateF function
    QVector3D rotationAxis;
    float rotationAngle;
    rotationQuatTotal.getAxisAndAngle(&rotationAxis,&rotationAngle);
    glRotatef( rotationAngle, rotationAxis[0], rotationAxis[1], rotationAxis[2] );


    glLineWidth(2.0f);
    glPointSize(4.0f);

    m_client.m_aircraft.plot();


    // draw axis frame
    QVector3D point0 = {0,0,0};
    QVector3D pointX = {1,0,0};
    QVector3D pointY = {0,1,0};
    QVector3D pointZ = {0,0,1};

    glBegin(GL_LINE_STRIP);
        glColor3f(1,0,0);
        glVertex3f(point0.x(),point0.y(),point0.z());
        glVertex3f(pointX.x(),pointX.y(),pointX.z());
    glEnd();
    glBegin(GL_LINE_STRIP);
        glColor3f(0,1,0);
        glVertex3f(point0.x(),point0.y(),point0.z());
        glVertex3f(pointY.x(),pointY.y(),pointY.z());
    glEnd();
    glBegin(GL_LINE_STRIP);
        glColor3f(0,0,1);
        glVertex3f(point0.x(),point0.y(),point0.z());
        glVertex3f(pointZ.x(),pointZ.y(),pointZ.z());
    glEnd();


    ++frameCount;

    if (frameTime.elapsed() >= 1000)
    {
        fps = frameCount / ((double)frameTime.elapsed()/1000.0);
    }

    // qInfo() << "FPS: " + QString::number(fps);

    glFlush();
}

void MainWindow::resizeEvent(QResizeEvent *event)
{
    resizeGL(this->width(), this->height());
    this->update();
}

void MainWindow::paintEvent(QPaintEvent *event)
{
    paintGL();
}

void MainWindow::UpdateAnimation()
{
    this->update();
}

void MainWindow::mousePressEvent(QMouseEvent *event)
{
    mouseHold = true;
    mouseHoldFirstCall = true;
}

void MainWindow::mouseReleaseEvent(QMouseEvent *event)
{
    mouseHold = false;
}

void MainWindow::mouseHoldEvent(QCursor *cursor)
{
    // reset mouse position when the mouse is pressed
    if (mouseHoldFirstCall) {
        mousePos = cursor->pos();
        lastMousePos = mousePos;
        mouseHoldFirstCall = false;
    }
    // incrementation
    else {
        lastMousePos = mousePos;
        mousePos = cursor->pos();
        rotationQuatLast = rotationQuatTotal;
    }

    // rotation axis and angle update
    QPoint diff = mousePos - lastMousePos;
    qreal distance = qSqrt(pow(diff.x(),2)+pow(diff.y(),2));
    float rotationAngleUpdate = distance * 0.5;
    QVector3D rotationAxisCurrent = QVector3D(diff.y(), diff.x(), 0.0).normalized();
    // rotation relative to last rotation
    QQuaternion rotationQuatUpdate = QQuaternion::fromAxisAndAngle(rotationAxisCurrent,rotationAngleUpdate);

    // combined rotation (by quaternion multiplication)
    rotationQuatTotal = rotationQuatUpdate * rotationQuatLast;
}

void MainWindow::wheelEvent(QWheelEvent *event)
{
    QPoint numDegrees = event->angleDelta();
    if (numDegrees.y()<0) zoomScale = zoomScale*1.1;
    if (numDegrees.y()>0) zoomScale = zoomScale/1.1;
}
