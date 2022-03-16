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
    QVector3D m_origin;
    QGenericMatrix<3,3,double> m_rotationMatrix;
    QVector3D m_shift;
    QVector<double> m_flap_depth;
    QVector<double> m_flap_deflection;
    Wind wind;
    double m_velocity;
};


class Fuselage : public Property {
    friend class Math;
public:
    Fuselage() : Property({"fuselage"}) {};
    void setFuselage(StructWithFieldnames fuselage);
    void setFuselage(StructWithFieldnames fuselage, QString name);
    void plot();
    void plot(QVector3D shift, QQuaternion rotation);
    void drawHollowCircle(QVector3D position, QGenericMatrix<3,3,double> orientation, double radius);
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





