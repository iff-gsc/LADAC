#include "myudp.h"

MyUDP::MyUDP(QObject *parent) : QObject(parent),
    // m_senderHostAddress(QHostAddress::LocalHost),
    m_senderHostAddress(QHostAddress::Any),
    // m_senderHostAddress("192.168.11.13"),
    m_port(4321)
{
	// create a QUDP socket
	socket = new QUdpSocket(this);

	// The most common way to use QUdpSocket class is to bind it to an address and port using bind()
	// bool QAbstractSocket::bind(const QHostAddress & address,
	//     quint16 port = 0, BindMode mode = DefaultForPlatform)
	socket->bind(m_senderHostAddress, m_port);

    socket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption,1250000);
	// Connect the signal of the QUdpSocket with the slot of this MyUDP class
	// If a package is available for the specified Host and Port, call the function readyRead of this class
	connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
}

void MyUDP::getPoints(QVector<double> &x, QVector<double> &y, QVector<double> &z) {
    x = m_x;
    y = m_y;
    z = m_z;
}

int16_t MyUDP::getNumberOfDataPoints() {
	return m_numberOfDataPoints;
}

void MyUDP::setHostAddressAndPort(QString hostAddress, quint16 port)
{
	m_senderHostAddress = QHostAddress(hostAddress);
	m_port = port;
	socket->close();
	socket->bind(m_senderHostAddress, m_port);

	connect(socket, SIGNAL(readyRead()), this, SLOT(readyRead()));
}

QString MyUDP::getHostAddress()
{
	return m_senderHostAddress.toString();
}

quint16 MyUDP::getHostPort()
{
	return m_port;
}

int i = 0;

/*****************************************************************************
 * This function is called, when a package is available
 *****************************************************************************/
void MyUDP::readyRead() {
	// Create a QByteArray as a buffer
	QByteArray buffer;
	buffer.resize(socket->pendingDatagramSize());

	// Create variables for the Host address and the Port of the sender
	QHostAddress sender;
	quint16 senderPort;

	// qint64 QUdpSocket::readDatagram(char * data, qint64 maxSize,
	//                 QHostAddress * address = 0, quint16 * port = 0)
	// Receives a datagram no larger than maxSize bytes and stores it in data.
	// The sender's host address and port is stored in *address and *port
	// (unless the pointers are 0).
	qint64 dataGramSize = socket->readDatagram(buffer.data(), buffer.size(),
						 &sender, &senderPort);

	// Check if receiving of data was successful
	if (dataGramSize != -1) {
		// Create QDataStream to read serialized of binary data stored in buffer
        QDataStream stream(buffer);
        // Set the QDataStream to write data in little endian, because "Simulink UDP Receive Binary" block receives only
		// little endian
        stream.setByteOrder(QDataStream::LittleEndian);

        stream.setFloatingPointPrecision(QDataStream::SinglePrecision);

        /*
        StructWithFieldnames test;
        test.setFromStream(stream);

        QVector<QString> subStructNames = test.getSubStructNames();
        StructWithFieldnames sub = test.getSubStruct(subStructNames[0]);
        */



        StructWithFieldnames aircraftStruct;
        aircraftStruct.setFromStream(stream);

        m_aircraft = Aircraft();
        m_aircraft.setAircraft(aircraftStruct);



        if (i>30) {
            i = 0;
        }
        else {
            i++;
        }

        qInfo() << i;
        // qInfo() << subStructNames[0];

	}
	else {
		std::cout << "An ERROR occures while receiving datagram!" << std::endl;
	}
    emit receivedValues();
}
