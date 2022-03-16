#ifndef MYUDP_H
#define MYUDP_H

#include <QObject>
#include <QUdpSocket>
#include <QDataStream>
#include <iostream>
#include <math.h>
#include "aircraft.h"


class MyUDP : public QObject {
	Q_OBJECT
public:
	explicit MyUDP(QObject *parent = 0);
    void getPoints(QVector<double> &m_x, QVector<double> &m_y, QVector<double> &m_z);
	int16_t getNumberOfDataPoints();
	void setHostAddressAndPort(QString hostAddress, quint16 port);
	QString getHostAddress();
	quint16 getHostPort();
    Aircraft m_aircraft;
signals:
    void receivedValues();
	void numberOfDataPointsChanged();

public slots:
	void readyRead();


private:
	QUdpSocket *socket;
    QVector<double> m_x, m_y, m_z;
	int16_t m_numberOfDataPoints = 0;
	QHostAddress m_senderHostAddress;
	quint16 m_port;
    QDataStream m_stream;
};

#endif // MYUDP_H
