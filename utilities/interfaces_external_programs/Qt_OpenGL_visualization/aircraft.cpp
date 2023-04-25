#include "aircraft.h"


bool Property::isProperty(QString string_) {
    bool propertyFound = false;
    for (int i=0; i<m_name_identifier.length(); i++) {
        if (m_name_identifier[i].contains(string_)) {
            propertyFound = true;
        }
    }
    return propertyFound;
}

QString Property::getName() {
    return m_name;
}

void Property::setName(QString name) {
    m_name = name;
}

void Wind::setWind(StructWithFieldnames state) {
    QVector<QString> fieldnames = state.getSubStructNames();
    for (int i=0; i<fieldnames.length(); i++) {
        QString fieldname = fieldnames[i];
        QVector<QString> wind_identifier = {"external"};
        for (int j=0; j<wind_identifier.length(); j++) {
            if (fieldname.contains(wind_identifier[j])) {
                StructWithFieldnames windStruct = state.getSubStruct(fieldname);
                QVector<double> V_Wb_i = windStruct.getSubData("V_Wb");
                m_V_Wb_i = Math::reshape2Matrix(V_Wb_i);
                break;
            }
        }
    }
}

void Wind::setScaling(double velocity) {
    m_scaling = 50 / velocity;
}

double RigidBody::getVelocity() {
    double velocity = m_V_Kb.length();
    // double velocity = -m_V_Kb.x();
    // double velocity = 15.0f;
    return velocity;
}

void Wing::setVelocity(double velocity) {
    m_velocity = velocity;
}

void Fuselage::setVelocity(double velocity) {
    m_velocity = velocity;
}

void Wind::plot(double downShift, QVector<QVector<double>> pos, double velocity) {
    // draw fuselage geometry

    QVector3D start_pt;
    QVector3D end_pt;

    int length;
    if (m_V_Wb_i.isEmpty()) {
        length = 0;
    }
    else {
        length = m_V_Wb_i[0].length();
    }

    glColor3f(143.0/255.0,169.0/255.0,186.0/255.0);

    QVector3D shift(0,0,downShift);
    QGenericMatrix<3,3,double> rotationMatrix;
    rotationMatrix.setToIdentity();

    setScaling(velocity);

    for (int i=0; i < length; i++) {

        // duplicate (to do)
        start_pt.setX(pos[0][i]);
        start_pt.setY(pos[1][i]);
        start_pt.setZ(0);

        start_pt = Math::local2Global(start_pt,shift,rotationMatrix);

        end_pt.setX(pos[0][i]+m_scaling*m_V_Wb_i[0][i]);
        end_pt.setY(pos[1][i]+m_scaling*m_V_Wb_i[1][i]);
        end_pt.setZ(0+m_scaling*m_V_Wb_i[2][i]);
        end_pt = Math::local2Global(end_pt,shift,rotationMatrix);

        //glPointSize ( 1.0f );
        glBegin ( GL_POINTS );
            glVertex3f ( start_pt.x(), start_pt.y(), start_pt.z() );
        glEnd();

        glBegin(GL_LINE_STRIP);
            glVertex3f(start_pt.x(),start_pt.y(),start_pt.z());
            glVertex3f(end_pt.x(),end_pt.y(),end_pt.z());
        glEnd();
    }
}

void RigidBody::setRigidBody(StructWithFieldnames rigidBody) {
    QVector<double> q_bg = rigidBody.getSubData("q_bg");
    QVector<double> V_Kb = rigidBody.getSubData("V_Kb");
    m_q_bg.setScalar(q_bg[0]);
    m_q_bg.setX(q_bg[1]);
    m_q_bg.setY(q_bg[2]);
    m_q_bg.setZ(q_bg[3]);
    m_V_Kb.setX(V_Kb[0]);
    m_V_Kb.setY(V_Kb[1]);
    m_V_Kb.setZ(V_Kb[2]);
}

void Wing::setWing(QString name, Vortex vortex, Cntrl_pt cntrl_pt) {
    // m_name = Property::m_name;
    Property::setName(name);
    m_vortex = vortex;
    m_cntrl_pt = cntrl_pt;
}

void Vortex::setVortex(StructWithFieldnames vortex) {
    QVector<QString> fieldNames = vortex.getSubDataNames();
    for (int i=0; i<fieldNames.length(); i++) {
        QString fieldName = fieldNames[i];
        if (fieldName=="pos") {
            QVector<double> pos = vortex.getSubData(fieldName);
            int num_points = pos.length()/3;
            for (int i=0; i<num_points; i++) {
                m_x.append(pos[3*i]);
                m_y.append(pos[3*i+1]);
                m_z.append(pos[3*i+2]);
            }
        }
        else if (fieldName=="c") {
            m_c = vortex.getSubData(fieldName);
        }
    }
}

void Cntrl_pt::setCntrlPt(StructWithFieldnames cntrl_pt) {
    QVector<QString> fieldNames = cntrl_pt.getSubDataNames();
    for (int i=0; i<fieldNames.length(); i++) {
        QString fieldname = fieldNames[i];
        if (fieldname=="pos") {
            QVector<double> pos = cntrl_pt.getSubData(fieldname);
            int num_points = pos.length()/3;
            for (int i=0; i<num_points; i++) {
                m_x.append(pos[3*i]);
                m_y.append(pos[3*i+1]);
                m_z.append(pos[3*i+2]);
            }
        }
        else if (fieldname=="local_incidence") {
            m_local_incidence = cntrl_pt.getSubData(fieldname);
        }
    }
}

void CoeffLocal::setCoeffLocal(StructWithFieldnames coeffLocal) {
    QVector<QString> fieldNames = coeffLocal.getSubDataNames();
    for (int i=0; i<fieldNames.length(); i++) {
        QString fieldname = fieldNames[i];
        QVector<double> array;
        QVector<QVector<double>> array2D(3);
        int k = 0;
        if (fieldname=="c_XYZ_b") {
            array = coeffLocal.getSubData(fieldname);
            int arrayLength = array.length();
            int subArrayLength = arrayLength/3;
            for (int i=0; i<3; i++) {
                QVector<double> subArray(subArrayLength);
                k = i;
                for (int j=0; j<subArrayLength; j++) {
                    subArray[j] = array[k];
                    k = k+3;
                }
                array2D[i] = subArray;
            }
            m_c_XYZ_b = array2D;
        }
        else if (fieldname=="c_lmn_b") {
            array = coeffLocal.getSubData(fieldname);
            int arrayLength = array.length();
            int subArrayLength = arrayLength/3;
            for (int i=0; i<3; i++) {
                QVector<double> subArray(subArrayLength);
                k = i;
                for (int j=0; j<subArrayLength; j++) {
                    subArray[j] = array[k];
                    k = k+3;
                }
                array2D[i] = subArray;
            }
            m_c_lmn_b = array2D;
        }
        else if (fieldname=="c_m_airfoil") {
            m_c_m_airfoil = coeffLocal.getSubData(fieldname);
        }
    }
}


void Wing::setWing(StructWithFieldnames wing) {
    QVector<QString> fieldNames = wing.getSubStructNames();
    StructWithFieldnames geometry = wing.getSubStruct("geometry");
    StructWithFieldnames vortex = geometry.getSubStruct("line_25");
    StructWithFieldnames cntrl_pt = geometry.getSubStruct("ctrl_pt");
    StructWithFieldnames aero = wing.getSubStruct("aero");
    StructWithFieldnames coeffLocal = aero.getSubStruct("coeff_loc");
    StructWithFieldnames segments = geometry.getSubStruct("segments");
    StructWithFieldnames actuators = wing.getSubStruct("actuators");
    StructWithFieldnames segments2 = actuators.getSubStruct("segments");
    QVector<double> origin = geometry.getSubData("origin");
    StructWithFieldnames unsteady = aero.getSubStruct("unsteady");
    this->m_vortex.setVortex(vortex);
    this->m_cntrl_pt.setCntrlPt(cntrl_pt);
    this->m_coeffLocal.setCoeffLocal(coeffLocal);
    this->m_flap_depth = segments.getSubData("flap_depth");
    QVector<double> flap_deflection_2x = segments2.getSubData("pos");
    for (int i=0; i<flap_deflection_2x.length()/2; i++) {
        m_flap_deflection.append(flap_deflection_2x[2*i]);
    }
    QVector<double> X = unsteady.getSubData("X");
    for (int i=0; i<X.length()/3; i++) {
        m_trailing_edge_sep_pt.append(X[3*i+1]);
    }
    m_cL_act2 = unsteady.getSubData("c_L_act2");
    this->setOrigin(origin);
    this->wind.setWind(wing);
}

void Wing::setWing(StructWithFieldnames wing, QString name) {
    this->setWing(wing);
    this->setName(name);
}

void Wing::setOrigin(QVector<double> origin) {
    m_origin.setX(origin[0]);
    m_origin.setY(origin[1]);
    m_origin.setZ(origin[2]);
}

QVector3D Wing::getPointLeadAt(int i) {
    QVector3D vortex_i( m_vortex.m_x[i], m_vortex.m_y[i], m_vortex.m_z[i] );
    // shift the point
    vortex_i = vortex_i + m_origin + m_shift;
    double chord_i = m_vortex.m_c[i];
    double twist_i;
    if (i==0) {
        twist_i = m_cntrl_pt.m_local_incidence[0];
    }
    else if (i==m_vortex.m_x.length()-1) {
        twist_i = m_cntrl_pt.m_local_incidence.last();
    }
    else {
        twist_i = ( m_cntrl_pt.m_local_incidence[i-1] + m_cntrl_pt.m_local_incidence[i] ) / 2;
    }
    QGenericMatrix<1,3,double> pointLeadMat;
    pointLeadMat(0,0) = vortex_i.x()+chord_i/4*cos(twist_i);
    pointLeadMat(1,0) = vortex_i.y();
    pointLeadMat(2,0) = vortex_i.z()-chord_i/4*sin(twist_i);

    // rotate the point
    pointLeadMat = m_rotationMatrix * pointLeadMat;

    QVector3D pointLead(pointLeadMat(0,0),pointLeadMat(1,0),pointLeadMat(2,0));

    return pointLead;
}

QVector3D Wing::getPointTrailAt(int i, int side) {
    QVector3D vortex_i( m_vortex.m_x[i], m_vortex.m_y[i], m_vortex.m_z[i] );
    // shift the point
    vortex_i = vortex_i + m_origin + m_shift;
    double chord_i = m_vortex.m_c[i];
    double twist_i;
    if (i==0) {
        twist_i = m_cntrl_pt.m_local_incidence[0];
    }
    else if (i==m_vortex.m_x.length()-1) {
        twist_i = m_cntrl_pt.m_local_incidence.last();
    }
    else {
        twist_i = ( m_cntrl_pt.m_local_incidence[i-1] + m_cntrl_pt.m_local_incidence[i] ) / 2;
    }
    QGenericMatrix<1,3,double> pointTrailMat;
    int flap_idx = 0;
    if ( (side == 0) & (i > 0)) {
        flap_idx = i-1;
    }
    else if (side == 1) {
        if (i <= m_cntrl_pt.m_x.length()) {
            flap_idx = i;
        }
        else {
            flap_idx = m_cntrl_pt.m_x.length();
        }
    }
    double rel_shift = 0.75 - m_flap_depth[flap_idx];
    pointTrailMat(0,0) = vortex_i.x()-chord_i*rel_shift*cos(twist_i);
    pointTrailMat(1,0) = vortex_i.y();
    pointTrailMat(2,0) = vortex_i.z()+chord_i*rel_shift*sin(twist_i);
    // rotate the point
    pointTrailMat = m_rotationMatrix * pointTrailMat;

    QVector3D pointTrail(pointTrailMat(0,0),pointTrailMat(1,0),pointTrailMat(2,0));

    return pointTrail;
}

QVector3D Wing::getPointFlapAt(int i, int side) {
    QVector3D vortex_i( m_vortex.m_x[i], m_vortex.m_y[i], m_vortex.m_z[i] );
    // shift the point
    vortex_i = vortex_i + m_origin + m_shift;
    double chord_i = m_vortex.m_c[i];
    double twist_i;
    if (i==0) {
        twist_i = m_cntrl_pt.m_local_incidence[0];
    }
    else if (i==m_vortex.m_x.length()-1) {
        twist_i = m_cntrl_pt.m_local_incidence.last();
    }
    else {
        twist_i = ( m_cntrl_pt.m_local_incidence[i-1] + m_cntrl_pt.m_local_incidence[i] ) / 2;
    }
    QGenericMatrix<1,3,double> pointFlapMat;
    int flap_idx = 0;
    if ( (side == 0) & (i > 0)) {
        flap_idx = i-1;
    }
    else if (side == 1) {
        if (i <= m_cntrl_pt.m_x.length()) {
            flap_idx = i;
        }
        else {
            flap_idx = m_cntrl_pt.m_x.length();
        }
    }
    double rel_shift = 0.75 - m_flap_depth[flap_idx];
    double rel_shift_flap = m_flap_depth[flap_idx];
    int ii;
    if (i>m_cntrl_pt.m_x.length()) {
        ii = m_cntrl_pt.m_x.length();
    }
    else {
        ii = i;
    }
    // rotate about flap rotation axis
    QVector3D axis = this->getPointTrailAt(ii+1,0) - this->getPointTrailAt(ii,1);
    QQuaternion flap_quat = QQuaternion::fromAxisAndAngle(axis,m_flap_deflection[flap_idx]+57.3*twist_i);
    QVector3D flap_vector;
    flap_vector.setX(-rel_shift_flap*chord_i);
    flap_vector.setY(0);
    flap_vector.setZ(0);
    QVector3D flap_vector_rotated = flap_quat.rotatedVector(flap_vector);
    pointFlapMat(0,0) = vortex_i.x() - chord_i*rel_shift*cos(twist_i) + flap_vector_rotated.x();
    pointFlapMat(1,0) = vortex_i.y() + flap_vector_rotated.y();
    pointFlapMat(2,0) = vortex_i.z() + chord_i*rel_shift*sin(twist_i) + flap_vector_rotated.z();
    // rotate the point
    pointFlapMat = m_rotationMatrix * pointFlapMat;

    QVector3D pointFlap(pointFlapMat(0,0),pointFlapMat(1,0),pointFlapMat(2,0));

    return pointFlap;
}

QVector3D Wing::getCenterOfPressureAt(int i) {
    QVector3D cntrl_pt_i( m_cntrl_pt.m_x[i], m_cntrl_pt.m_y[i], m_cntrl_pt.m_z[i] );
    QVector3D cop_i;
    QGenericMatrix<1,3,double> cop_i_Mat;
    // shift the point
    cntrl_pt_i = cntrl_pt_i + m_origin + m_shift;
    // shift the point from control point to center of pressure
    float chord = ( m_vortex.m_c[i] + m_vortex.m_c[i+1] ) / 2;
    float distance = m_coeffLocal.m_c_m_airfoil[i] / sqrt( pow(m_coeffLocal.m_c_XYZ_b[0][i],2) + pow(m_coeffLocal.m_c_XYZ_b[1][i],2) + pow(m_coeffLocal.m_c_XYZ_b[2][i],2) ) * chord;
    if (distance>chord/4) {
        distance = chord/4;
    }
    else if (distance < -chord*3/4) {
        distance = -chord*3/4;
    }
    // shift from control point to c/4
    distance = distance + chord/2;
    cop_i.setX( cntrl_pt_i.x() + distance * cos(m_cntrl_pt.m_local_incidence[i]) );
    cop_i.setY(cntrl_pt_i.y());
    cop_i.setZ( cntrl_pt_i.z() - distance * sin(m_cntrl_pt.m_local_incidence[i]) );

    // rotate (wow)
    cop_i_Mat(0,0) = cop_i.x();
    cop_i_Mat(1,0) = cop_i.y();
    cop_i_Mat(2,0) = cop_i.z();
    cop_i_Mat = m_rotationMatrix * cop_i_Mat;
    cop_i.setX(cop_i_Mat(0,0));
    cop_i.setY(cop_i_Mat(1,0));
    cop_i.setZ(cop_i_Mat(2,0));

    return cop_i;
}

void Wing::plot(QVector3D shift, QQuaternion rotation) {
    m_shift = shift;
    m_rotationMatrix = Math::quaternion2RotationMatrix(rotation);
    this->plot();
}

void Wing::plot() {

    // draw aerodynamic forces
    QVector3D pointCop;
    QVector3D pointCopPlusForce;
    QVector3D pointCopPlusLadForce;
    for (int i=0; i < m_cntrl_pt.m_x.length(); i++) {
        pointCop = this->getCenterOfPressureAt(i);
        pointCopPlusForce = pointCop;
        pointCopPlusLadForce = pointCop;
        QVector3D forceVector( m_coeffLocal.m_c_XYZ_b[0][i], m_coeffLocal.m_c_XYZ_b[1][i], m_coeffLocal.m_c_XYZ_b[2][i] );
        QVector3D ladForceVector( 0, 0, -m_cL_act2[i] );

        // rotate (wow)
        QGenericMatrix<1,3,double> forceVectorMat;
        forceVectorMat(0,0) = forceVector.x();
        forceVectorMat(1,0) = forceVector.y();
        forceVectorMat(2,0) = forceVector.z();
        forceVectorMat = m_rotationMatrix * forceVectorMat;
        forceVector.setX(forceVectorMat(0,0));
        forceVector.setY(forceVectorMat(1,0));
        forceVector.setZ(forceVectorMat(2,0));
        QGenericMatrix<1,3,double> ladForceVectorMat;
        ladForceVectorMat(0,0) = ladForceVector.x();
        ladForceVectorMat(1,0) = ladForceVector.y();
        ladForceVectorMat(2,0) = ladForceVector.z();
        ladForceVectorMat = m_rotationMatrix * ladForceVectorMat;
        ladForceVector.setX(ladForceVectorMat(0,0));
        ladForceVector.setY(ladForceVectorMat(1,0));
        ladForceVector.setZ(ladForceVectorMat(2,0));

        pointCopPlusForce = pointCop + forceVector * (m_vortex.m_c[i]+m_vortex.m_c[i+1])/2 * 5;
        pointCopPlusLadForce = pointCop + ladForceVector * (m_vortex.m_c[i]+m_vortex.m_c[i+1])/2 * 5;
        QVector3D pointSeparationHelp = pointCop + (1-m_trailing_edge_sep_pt[i])*(pointCopPlusForce-pointCop);
        // pointCopPlusForce = pointCop + forceVector * 10;
        glBegin(GL_LINE_STRIP);
            glColor4f(m_forceColor[0],m_forceColor[1],m_forceColor[2],m_forceAlpha);
            glVertex3f(pointCop.x(),pointCop.y(),pointCop.z());
            glColor4f(m_forceColor[0],m_forceColor[1],m_forceColor[2],m_forceAlpha);
            glVertex3f(pointSeparationHelp.x(),pointSeparationHelp.y(),pointSeparationHelp.z());
            glColor4f(m_stallColor[0],m_stallColor[1],m_stallColor[2],m_stallAlpha);
            glVertex3f(pointCopPlusForce.x(),pointCopPlusForce.y(),pointCopPlusForce.z());
        glEnd();
        glColor4f(m_ladForceColor[0],m_ladForceColor[1],m_ladForceColor[2],m_ladForceAlpha);
        glBegin(GL_LINE_STRIP);
            glVertex3f(pointCop.x(),pointCop.y(),pointCop.z());
            glVertex3f(pointCopPlusLadForce.x(),pointCopPlusLadForce.y(),pointCopPlusLadForce.z());
        glEnd();
    }
    // draw wing geometry
    QVector3D pointLeadLeft;
    QVector3D pointTrailLeft;
    QVector3D pointLeadRight;
    QVector3D pointTrailRight;
    QVector3D pointFlapLeft;
    QVector3D pointFlapRight;
    QVector3D cntrl_pt;
    QVector<QVector<double>> cntrl_pos{3};

    for (int i=0; i < m_cntrl_pt.m_x.length(); i++) {
        pointLeadRight = this->getPointLeadAt(i+1);
        pointTrailRight = this->getPointTrailAt(i+1,0);
        pointLeadLeft = this->getPointLeadAt(i);
        pointTrailLeft = this->getPointTrailAt(i,1);
        glColor4f(m_faceColor[0],m_faceColor[1],m_faceColor[2],m_faceAlpha);
        glBegin(GL_QUADS);
            glVertex3f(pointLeadLeft.x(),pointLeadLeft.y(),pointLeadLeft.z());
            glVertex3f(pointLeadRight.x(),pointLeadRight.y(),pointLeadRight.z());
            glVertex3f(pointTrailRight.x(),pointTrailRight.y(),pointTrailRight.z());
            glVertex3f(pointTrailLeft.x(),pointTrailLeft.y(),pointTrailLeft.z());
        glEnd();
        glColor4f(m_lineColor[0],m_lineColor[1],m_lineColor[2],m_lineAlpha);
        glBegin(GL_LINE_STRIP);
            glVertex3f(pointLeadLeft.x(),pointLeadLeft.y(),pointLeadLeft.z());
            glVertex3f(pointLeadRight.x(),pointLeadRight.y(),pointLeadRight.z());
            glVertex3f(pointTrailRight.x(),pointTrailRight.y(),pointTrailRight.z());
            glVertex3f(pointTrailLeft.x(),pointTrailLeft.y(),pointTrailLeft.z());
            glVertex3f(pointLeadLeft.x(),pointLeadLeft.y(),pointLeadLeft.z());
        glEnd();
        cntrl_pt.setX(m_cntrl_pt.m_x[i]);
        cntrl_pt.setY(m_cntrl_pt.m_y[i]);
        cntrl_pt.setZ(m_cntrl_pt.m_z[i]);
        cntrl_pt = Math::local2Global(cntrl_pt,m_origin+m_shift,m_rotationMatrix);
        cntrl_pos[0].append(cntrl_pt[0]);
        cntrl_pos[1].append(cntrl_pt[1]);
        cntrl_pos[2].append(cntrl_pt[2]);
    }

    for (int i=0; i < m_cntrl_pt.m_x.length()-1; i++) {
        pointTrailRight = this->getPointTrailAt(i+1,0);
        pointFlapRight = this->getPointFlapAt(i+1,0);
        pointFlapLeft = this->getPointFlapAt(i,1);
        pointTrailLeft = this->getPointTrailAt(i,1);
        glColor4f(m_flapLineColor[0],m_flapLineColor[1],m_flapLineColor[2],m_flapLineAlpha);
        glBegin(GL_LINE_STRIP);
            glVertex3f(pointTrailLeft.x(),pointTrailLeft.y(),pointTrailLeft.z());
            glVertex3f(pointTrailRight.x(),pointTrailRight.y(),pointTrailRight.z());
            glVertex3f(pointFlapRight.x(),pointFlapRight.y(),pointFlapRight.z());
            glVertex3f(pointFlapLeft.x(),pointFlapLeft.y(),pointFlapLeft.z());
            glVertex3f(pointTrailLeft.x(),pointTrailLeft.y(),pointTrailLeft.z());
        glEnd();
        glColor4f(m_flapFaceColor[0],m_flapFaceColor[1],m_flapFaceColor[2],m_flapFaceAlpha);
        glBegin(GL_QUADS);
            glVertex3f(pointTrailLeft.x(),pointTrailLeft.y(),pointTrailLeft.z());
            glVertex3f(pointTrailRight.x(),pointTrailRight.y(),pointTrailRight.z());
            glVertex3f(pointFlapRight.x(),pointFlapRight.y(),pointFlapRight.z());
            glVertex3f(pointFlapLeft.x(),pointFlapLeft.y(),pointFlapLeft.z());
        glEnd();
    }

    this->wind.plot(20,cntrl_pos,m_velocity);

}


void Fuselage::setFuselage(StructWithFieldnames fuselage) {
    QVector<QString> fieldNames = fuselage.getSubStructNames();
    StructWithFieldnames geometry = fuselage.getSubStruct("geometry");
    QVector<double> cntrl_pos = geometry.getSubData("cntrl_pos");
    QVector<double> border_pos = geometry.getSubData("border_pos");
    m_width = geometry.getSubData("width");
    StructWithFieldnames aero = fuselage.getSubStruct("aero");
    QVector<double> c_XYZ_b_i = aero.getSubData("C_XYZ_b_i");
    QVector<double> R_Ab_i = aero.getSubData("R_Ab_i");

    m_cntrl_pt = Math::reshape2Matrix(cntrl_pos);
    m_border_pt = Math::reshape2Matrix(border_pos);
    m_c_XYZ_b_i = Math::reshape2Matrix(c_XYZ_b_i);
    m_R_Ab_i = Math::reshape2Matrix(R_Ab_i);

    m_wind.setWind(fuselage);
}

void Fuselage::setFuselage(StructWithFieldnames fuselage, QString name) {
    this->setFuselage(fuselage);
    this->setName(name);
}

QVector<QVector<double>> Math::reshape2Matrix(QVector<double> vector) {
    QVector<QVector<double>> array2D(3);
    int arrayLength = vector.length();
    int subArrayLength = arrayLength/3;
    for (int i=0; i<3; i++) {
        QVector<double> subArray(subArrayLength);
        int k = i;
        for (int j=0; j<subArrayLength; j++) {
            subArray[j] = vector[k];
            array2D[i].append(vector[k]);
            k = k+3;
        }
        // array2D[i] = subArray;
    }
    return array2D;
}

QGenericMatrix<3,3,double> Math::quaternion2RotationMatrix(QQuaternion rotationQuaternion) {
    QGenericMatrix<3,3,double> rotationMatrix;
    float *dataMat = rotationQuaternion.toRotationMatrix().data();
    int k = 0;
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            rotationMatrix(j,i) = *(dataMat + k++);
        }
    }
    return rotationMatrix;
}

void Fuselage::plot(QVector3D shift, QQuaternion rotation) {
    m_shift = shift;
    m_rotationMatrix = Math::quaternion2RotationMatrix(rotation);
    this->plot();
}

void Fuselage::plot() {

    QVector3D cntrl_pt;
    QVector3D border_pt;
    QVector3D next_border_pt;
    QVector3D R_Ab_i_end;

    QVector<QVector<double>> border_pos = m_border_pt;

    QVector<QVector<double>> cycle1;
    QVector<QVector<double>> cycle2;

    int length;
    if (m_border_pt.isEmpty()) {
        length = 0;
    }
    else {
        length = m_border_pt[0].length();
    }
    for (int i=0; i < length-1; i++) {
        border_pt.setX(m_border_pt[0][i]);
        border_pt.setY(m_border_pt[1][i]);
        border_pt.setZ(m_border_pt[2][i]);
        border_pt = Math::local2Global(border_pt,m_shift,m_rotationMatrix);

        border_pos[0][i] = border_pt.x();
        border_pos[1][i] = border_pt.y();
        border_pos[2][i] = border_pt.z();

        next_border_pt.setX(m_border_pt[0][i+1]);
        next_border_pt.setY(m_border_pt[1][i+1]);
        next_border_pt.setZ(m_border_pt[2][i+1]);
        next_border_pt = Math::local2Global(next_border_pt,m_shift,m_rotationMatrix);

        cntrl_pt.setX(m_cntrl_pt[0][i]);
        cntrl_pt.setY(m_cntrl_pt[1][i]);
        cntrl_pt.setZ(m_cntrl_pt[2][i]);
        cntrl_pt = Math::local2Global(cntrl_pt,m_shift,m_rotationMatrix);

        // scaling = (m_width[i]+m_width[i+1])/2;
        double scaling = 5;
        R_Ab_i_end.setX(m_cntrl_pt[0][i]+scaling*m_c_XYZ_b_i[0][i]*(m_width[i]+m_width[i+1])/2);
        R_Ab_i_end.setY(m_cntrl_pt[1][i]+scaling*m_c_XYZ_b_i[1][i]*(m_width[i]+m_width[i+1])/2);
        R_Ab_i_end.setZ(m_cntrl_pt[2][i]+scaling*m_c_XYZ_b_i[2][i]*(m_width[i]+m_width[i+1])/2);
        R_Ab_i_end = Math::local2Global(R_Ab_i_end,m_shift,m_rotationMatrix);

        // draw fuselage geometry: center line
        glColor4f(m_lineColor[0],m_lineColor[1],m_lineColor[2],m_lineAlpha);
        glBegin(GL_LINE_STRIP);
            glVertex3f(border_pt.x(),border_pt.y(),border_pt.z());
            glVertex3f(cntrl_pt.x(),cntrl_pt.y(),cntrl_pt.z());
            glVertex3f(next_border_pt.x(),next_border_pt.y(),next_border_pt.z());
        glEnd();

        // draw aerodynamic forces
        glColor4f(m_forceColor[0],m_forceColor[1],m_forceColor[2],m_forceAlpha);
        glBegin(GL_LINE_STRIP);
            glVertex3f(cntrl_pt.x(),cntrl_pt.y(),cntrl_pt.z());
            glVertex3f(R_Ab_i_end.x(),R_Ab_i_end.y(),R_Ab_i_end.z());
        glEnd();

        // draw fuselage geometry: cycles
        glColor4f(m_lineColor[0],m_lineColor[1],m_lineColor[2],m_lineAlpha);
        cycle1 = drawHollowCircle(border_pt,m_rotationMatrix,m_width[i]/2);
        cycle2 = drawHollowCircle(next_border_pt,m_rotationMatrix,m_width[i+1]/2);

        // draw fuselage geometry: conic faces
        glColor4f(m_faceColor[0],m_faceColor[1],m_faceColor[2],m_faceAlpha);
        drawConicFace(cycle1, cycle2);

        }

    m_wind.plot(20,border_pos,m_velocity);

}


QVector<QVector<double>> Fuselage::drawHollowCircle(QVector3D position, QGenericMatrix<3,3,double> orientation, double radius){
   int i;

   double PI = 3.14159;

   //GLfloat radius = 0.8f; //radius
   double twicePi = 2.0f * PI;

   QVector<QVector<double>> positionMat;
   QVector<double> positionSub (3);

   QGenericMatrix<1,3,double> circlePt;

   glBegin(GL_LINE_LOOP);
       for(i = 0; i <= m_lineAmount;i++) {
           circlePt(2,0) = (radius * cos(i * twicePi / m_lineAmount));
           circlePt(1,0) = (radius * sin(i * twicePi / m_lineAmount));
           circlePt(0,0) = 0;
           circlePt = orientation * circlePt;
           circlePt(0,0) += position.x();
           circlePt(1,0) += position.y();
           circlePt(2,0) += position.z();
           glVertex3f( circlePt(0,0), circlePt(1,0), circlePt(2,0) );
           positionSub[0] = circlePt(0,0);
           positionSub[1] = circlePt(1,0);
           positionSub[2] = circlePt(2,0);
           positionMat.append(positionSub);
       }
   glEnd();
   return positionMat;
}

void Fuselage::drawConicFace(QVector<QVector<double>> positionMat1, QVector<QVector<double>> positionMat2){
   int i;

   QGenericMatrix<1,3,double> circlePt;

   for(i = 0; i <= m_lineAmount-1;i++) {
       glBegin(GL_QUADS);
           glVertex3f( positionMat1[i][0], positionMat1[i][1], positionMat1[i][2] );
           glVertex3f( positionMat1[i+1][0], positionMat1[i+1][1], positionMat1[i+1][2] );
           glVertex3f( positionMat2[i+1][0], positionMat2[i+1][1], positionMat2[i+1][2] );
           glVertex3f( positionMat2[i][0], positionMat2[i][1], positionMat2[i][2] );
       glEnd();
   }
}


QVector3D Math::local2Global(QVector3D vector, QVector3D shift, QGenericMatrix<3,3,double> rotationMatrix ) {

    vector = vector + shift;

    QGenericMatrix<1,3,double> vectorMat;

    vectorMat(0,0) = vector.x();
    vectorMat(1,0) = vector.y();
    vectorMat(2,0) = vector.z();

    vectorMat = rotationMatrix * vectorMat;

    vector.setX(vectorMat(0,0));
    vector.setY(vectorMat(1,0));
    vector.setZ(vectorMat(2,0));

    return vector;
}


void Aircraft::setPart(Wing wing_){
    bool replaced = false;
    for (int i=0; i<m_wing_array.length(); i++) {
        if (m_wing_array[i].getName()==wing_.getName()) {
            m_wing_array[i] = wing_;
            replaced = true;
        }
    }
    if (!replaced) {
        m_wing_array.append(wing_);
    }
}

void Aircraft::setPart(Fuselage fuselage_) {
    bool replaced = false;
    for (int i=0; i<m_fuselage_array.length(); i++) {
        if (m_fuselage_array[i].getName()==fuselage_.getName()) {
            m_fuselage_array[i] = fuselage_;
            replaced = true;
        }
    }
    if (!replaced) {
        m_fuselage_array.append(fuselage_);
    }
}

void Aircraft::setAircraft(StructWithFieldnames aircraft) {
    QVector<QString> fieldnames = aircraft.getSubStructNames();
    for (int i=0; i<fieldnames.length(); i++) {
        QString fieldname = fieldnames[i];
        QVector<QString> wing_identifier = m_wing_array[0].m_name_identifier;
        for (int j=0; j<wing_identifier.length(); j++) {
            if (fieldname.contains(wing_identifier[j])) {
                StructWithFieldnames wingStruct = aircraft.getSubStruct(fieldname);
                Wing wing;
                wing.setWing(wingStruct,fieldname);
                if (m_wing_array[0].getName().length()==0) {
                    m_wing_array[0] = wing;
                }
                else {
                    m_wing_array.append(wing);
                }
                break;
            }
        }
        QVector<QString> fuselage_identifier = m_fuselage_array[0].m_name_identifier;
        for (int j=0; j<fuselage_identifier.length(); j++) {
            if (fieldname.contains(fuselage_identifier[j])) {
                StructWithFieldnames fuselageStruct = aircraft.getSubStruct(fieldname);
                Fuselage fuselage;
                fuselage.setFuselage(fuselageStruct,fieldname);
                if (m_fuselage_array[0].getName().length()==0) {
                    m_fuselage_array[0] = fuselage;
                }
                else {
                    m_fuselage_array.append(fuselage);
                }
                break;
            }
        }
        QVector<QString> rigidBody_identifier = m_rigidBody.m_name_identifier;
        for (int j=0; j<rigidBody_identifier.length(); j++) {
            if (fieldname.contains(rigidBody_identifier[j])) {
                StructWithFieldnames rigidBodyStruct = aircraft.getSubStruct(fieldname);
                // RigidBody rigidBody;
                m_rigidBody.setRigidBody(rigidBodyStruct);
            }
        }
        StructWithFieldnames config = aircraft.getSubStruct("config");
        QVector<double> pos_ref = config.getSubData("xyz_ref_c");
        m_posRef[0] = -pos_ref[0];
        m_posRef[1] = -pos_ref[1];
        m_posRef[2] = -pos_ref[2];
        double velocity = m_rigidBody.getVelocity();
        for (int i = 0; i<m_wing_array.length(); i++) {
            m_wing_array[i].setVelocity(velocity);
        }
        for (int i = 0; i<m_fuselage_array.length(); i++) {
            m_fuselage_array[i].setVelocity(velocity);
        }
    }
}

void Aircraft::plot() {
    for (int i=0; i<m_wing_array.length(); i++) {
        m_wing_array[i].plot(m_posRef,m_rigidBody.m_q_bg);
    }
    for (int i=0; i<m_fuselage_array.length(); i++) {
        m_fuselage_array[i].plot(m_posRef,m_rigidBody.m_q_bg);
    }
}
