#include <QString>
#include <QVector>
#include <qmath.h>
#include <QDataStream>
#include <cstdlib>
#include <QtOpenGL>

#include "structWithFieldnames.h"



class Property {
public:
    Property(QVector<QString> name_identifier) {
        for (int i=0; i<name_identifier.length(); i++) {
            m_name_identifier.append(name_identifier[i]);
        }
    };
    QVector<QString> m_name_identifier;
    QString getName();
    void setName(QString name);
    bool isProperty(QString string_);
private:
    QString m_name;
};

class Wind {
    friend class Math;
    friend class Aircraft;
public:
    void setScaling(double velocity);
    void setWind(StructWithFieldnames wind);
    void plot(double downShift, QVector<QVector<double>> pos, double velocity);
private:
    QVector<QVector<double>> m_V_Wb_i;
    double m_scaling = 1;
    double m_downShift = 20;
};

class Math {
public:
    static QVector3D local2Global(QVector3D vector, QVector3D shift, QGenericMatrix<3,3,double> rotationMatrix );
    static QVector<QVector<double>> reshape2Matrix(QVector<double> vector);
    static QGenericMatrix<3,3,double> quaternion2RotationMatrix(QQuaternion rotationQuaternion);
};

class Vortex : public Property {
    friend class Wing;
public:
    Vortex() : Property({"vortex"}) {};
    void setVortex(StructWithFieldnames vortex);
private:
    QVector<double> m_x;
    QVector<double> m_y;
    QVector<double> m_z;
    QVector<double> m_c;
};

class Cntrl_pt : public Property {
    friend class Wing;
public:
    Cntrl_pt() : Property({"cntrl_pt"}) {};
    void setCntrlPt(StructWithFieldnames cntrl_pt);
private:
    QVector<double> m_x;
    QVector<double> m_y;
    QVector<double> m_z;
    QVector<double> m_local_incidence;
};

class CoeffLocal : public Property {
    friend class Wing;
public:
    CoeffLocal() : Property({"coeff_loc","coeffLocal"}) {};
    void setCoeffLocal(StructWithFieldnames coeffLocal);
private:
    QVector<QVector<double>> m_c_XYZ_b;
    QVector<QVector<double>> m_c_lmn_b;
    QVector<double> m_c_m_airfoil;
};


class Wing : public Property {
public:
    Wing() : Property({"wing","htp","vtp","plane"}) {
        m_rotationMatrix.setToIdentity();
        m_shift={0,0,0};
    };
    void setWing(QString name, Vortex vortex, Cntrl_pt cntrl_pt);
    void setWing(StructWithFieldnames wing);
    void setWing(StructWithFieldnames wing, QString name);
    void setOrigin(QVector<double> origin);
    QVector3D getPointLeadAt(int i);
    QVector3D getPointTrailAt(int i, int side);
    QVector3D getPointFlapAt(int i, int side);
    QVector3D getCenterOfPressureAt(int i);
    void plot();
    void plot(QVector3D shift, QQuaternion rotation);
    void setVelocity(double velocity);
private:
    Vortex m_vortex;
    Cntrl_pt m_cntrl_pt;
    CoeffLocal m_coeffLocal;
    QVector<double> m_cL_act2;
    QVector3D m_origin;
    QGenericMatrix<3,3,double> m_rotationMatrix;
    QVector3D m_shift;
    QVector<double> m_flap_depth;
    QVector<double> m_flap_deflection;
    Wind wind;
    double m_velocity;
    QVector<double> m_trailing_edge_sep_pt;
    float m_lineColor[3] = {0.1,0.1,0.1};
    float m_lineAlpha = 1.0;
    float m_faceColor[3] = {0.99,0.99,0.99};
    float m_faceAlpha = 0.5;
    float m_forceColor[3] = {76.0/255.0,212.0/255.,38.0/255.0};
    float m_forceAlpha = 1.0;
    float m_flapLineColor[3] = {0.1,0.1,0.1};
    float m_flapLineAlpha = 1.0;
    float m_flapFaceColor[3] = {0.65,0.65,0.65};
    float m_flapFaceAlpha = 0.5;
    float m_stallColor[3] = {247.0/255.0,132.0/255.0,17.0/255.0};
    float m_stallAlpha = 0.7;
    float m_ladForceColor[3] = {0.7,0.45,0.9};
    float m_ladForceAlpha = 1;
};


class Fuselage : public Property {
    friend class Math;
public:
    Fuselage() : Property({"fuselage"}) {};
    void setFuselage(StructWithFieldnames fuselage);
    void setFuselage(StructWithFieldnames fuselage, QString name);
    void plot();
    void plot(QVector3D shift, QQuaternion rotation);
    QVector<QVector<double>> drawHollowCircle(QVector3D position, QGenericMatrix<3,3,double> orientation, double radius);
    void drawConicFace(QVector<QVector<double>> positionMat1, QVector<QVector<double>> positionMat2);
    void setVelocity(double velocity);
private:
    QVector<QVector<double>> m_cntrl_pt;
    QVector<QVector<double>> m_border_pt;
    QVector<QVector<double>> m_c_XYZ_b_i;
    QVector<QVector<double>> m_R_Ab_i;
    QVector<double> m_width;
    QVector3D m_shift;
    QGenericMatrix<3,3,double> m_rotationMatrix;
    Wind m_wind;
    double m_velocity;
    int m_lineAmount = 100;
    float m_lineColor[3] = {0.1,0.1,0.1};
    float m_lineAlpha = 1.0;
    float m_faceColor[3] = {0.99,0.99,0.99};
    float m_faceAlpha = 0.5;
    float m_forceColor[3] = {76.0/255.0,212.0/255.,38.0/255.0};
    float m_forceAlpha = 1.0;
};


class RigidBody : public Property {
public:
    RigidBody() : Property({"body"}) {};
    void setRigidBody(StructWithFieldnames name);
    QQuaternion m_q_bg;
    QVector3D m_V_Kb;
    double getVelocity();
};


class Aircraft{
public:
    Aircraft() {
        m_wing_array.append(Wing());
        m_fuselage_array.append(Fuselage());
    };
    void setPart(Wing wing_);
    void setPart(Fuselage fuselage_);
    void setAircraft(StructWithFieldnames aircraft);
    void plot();
    void plot(QQuaternion rotation);
    QVector3D m_posRef;
private:
    QVector<Wing> m_wing_array;
    QVector<Fuselage> m_fuselage_array;
    RigidBody m_rigidBody;
};





