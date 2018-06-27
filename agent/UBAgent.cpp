#include "UBAgent.h"
#include "UBNetwork.h"
#include "UBPower.h"

#include "UBConfig.h"

#include <QTime>
#include <QTimer>
#include <QCommandLineParser>

#include "Vehicle.h"
#include "TCPLink.h"
#include "QGCApplication.h"

int flight_direction = 90; //0  north, 90 east
int flight_distance = 10;
float target_stabilize_time = 2.0; //in seconds
float target_wait_time = 5.0; //in seconds

UBAgent::UBAgent(QObject *parent) : QObject(parent),
    m_mav(nullptr)
{
    m_net = new UBNetwork;
    connect(m_net, SIGNAL(dataReady(quint8, QByteArray)), this, SLOT(dataReadyEvent(quint8, QByteArray)));

    m_power = new UBPower;
    connect(m_power, SIGNAL(dataReady(quint8, QByteArray)), this, SLOT(dataReadyEvent(quint8, QByteArray)));

    m_timer = new QTimer;
    connect(m_timer, SIGNAL(timeout()), this, SLOT(missionTracker()));

    startAgent();
}

void UBAgent::startAgent() {
    QCommandLineParser parser;
    parser.setSingleDashWordOptionMode(QCommandLineParser::ParseAsLongOptions);

    parser.addOptions({
        {{"I", "instance"}, "Set instance (ID) of the agent", "id"},
    });

//    parser.process(*QCoreApplication::instance());
    parser.parse(QCoreApplication::arguments());

    quint8 id = parser.value("I").toUInt();
    LinkConfiguration* link = nullptr;
    if (id) {
        quint32 port = 10 * id + STL_PORT + 3;
        TCPConfiguration* tcp = new TCPConfiguration(tr("TCP Port %1").arg(port));
        tcp->setAddress(QHostAddress::LocalHost);
        tcp->setPort(port);

        link = tcp;
    } else {
        SerialConfiguration* serial = new SerialConfiguration("Serial Port");
        serial->setBaud(BAUD_RATE);
        serial->setPortName(SERIAL_PORT);

        link = serial;
    }

    link->setDynamic();
    link->setAutoConnect();

    LinkManager* linkManager = qgcApp()->toolbox()->linkManager();
    linkManager->addConfiguration(link);
    linkManager->linkConfigurationsChanged();

    connect(qgcApp()->toolbox()->multiVehicleManager(), SIGNAL(vehicleAdded(Vehicle*)), this, SLOT(vehicleAddedEvent(Vehicle*)));
    connect(qgcApp()->toolbox()->multiVehicleManager(), SIGNAL(vehicleRemoved(Vehicle*)), this, SLOT(vehicleRemovedEvent(Vehicle*)));

    m_net->connectToHost(QHostAddress::LocalHost, 10 * id + NET_PORT);
    m_power->connectToHost(QHostAddress::LocalHost, PWR_PORT);
    m_timer->start(1000.0*MISSION_TRACK_DELAY);

    m_mission_data.reset();
}

void UBAgent::setMAV(Vehicle* mav) {
    if (m_mav) {
        disconnect(m_mav, SIGNAL(armedChanged(bool)), this, SLOT(armedChangedEvent(bool)));
        disconnect(m_mav, SIGNAL(flightModeChanged(QString)), this, SLOT(flightModeChangedEvent(QString)));
    }

    m_mav = mav;

    if (m_mav) {
        connect(m_mav, SIGNAL(armedChanged(bool)), this, SLOT(armedChangedEvent(bool)));
        connect(m_mav, SIGNAL(flightModeChanged(QString)), this, SLOT(flightModeChangedEvent(QString)));
    }
}

void UBAgent::vehicleAddedEvent(Vehicle* mav) {
    if (!mav || m_mav == mav) {
        return;
    }

    setMAV(mav);
    m_net->setID(mav->id());

    qInfo() << "New MAV connected with ID: " << m_mav->id();
}

void UBAgent::vehicleRemovedEvent(Vehicle* mav) {
    if (!mav || m_mav != mav) {
        return;
    }

    setMAV(nullptr);
    m_net->setID(0);

    qInfo() << "MAV disconnected with ID: " << mav->id();
}

void UBAgent::armedChangedEvent(bool armed) {
    if (!armed) {
        if (m_mission_state!=STATE_LAND) { //do not interfere lnding procedure.
            m_mission_state = STATE_IDLE;
        }
        return;
    }

    if (m_mav->altitudeRelative()->rawValue().toDouble() > POINT_ZONE) {
        qWarning() << "The mission can not start while the drone is airborne!";
        return;
    }

//    m_mav->setGuidedMode(true);
    if (!m_mav->guidedMode()) {
        qWarning() << "The mission can not start while the drone is not in Guided mode!";
        return;
    }

    m_mission_data.reset();
    qInfo() << "Mission starts...";

    m_mission_state = STATE_TAKEOFF;
//    m_mav->guidedModeTakeoff();
    m_mav->sendMavCommand(m_mav->defaultComponentId(),
                            MAV_CMD_NAV_TAKEOFF,
                            true, // show error
                            0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
                            TAKEOFF_ALT);
}

void UBAgent::flightModeChangedEvent(QString mode) {
    qInfo() << mode;
    // automatic arm after switching from Land to Guided. be careful!
    if (m_mission_data.previousFlightMode == "Land" && 
        mode=="Guided" && !m_mav->armed()) {
            m_mav->setArmed(true);
    }
    m_mission_data.previousFlightMode = mode;
}

void UBAgent::dataReadyEvent(quint8 srcID, QByteArray data) {
//    Q_UNUSED(data)
    qInfo() << "Data received from srcID=" << srcID << ":\n" << data;
//    This is used to arm one UAV by previous one.
//    if(srcID == m_mav->id() - 1 && !m_mav->armed()) {
//        m_mav->setArmed(true);
//    }
}

void UBAgent::missionTracker() {
    switch (m_mission_state) {
    case STATE_IDLE:
        stateIdle();
        break;
    case STATE_TAKEOFF:
        stateTakeoff();
        break;
    case STATE_MISSION:
        stateMission();
        break;
    case STATE_LAND:
        stateLand();
        break;
    default:
        break;
    }
}

void UBAgent::stateIdle() {
}

void UBAgent::stateTakeoff() {
    if (m_mission_data.stage == 0) {
        if (m_mav->altitudeRelative()->rawValue().toDouble() > TAKEOFF_ALT - POINT_ZONE) {
            m_mission_data.tick=0;
            m_mission_data.stage++;
        }
    }
    if (m_mission_data.stage == 1) {
        m_mission_data.tick++;
        if (m_mission_data.tick > (3.0 * 1.0 / MISSION_TRACK_DELAY - 0.001)) { //waiting for 3 seconds to stabilize
            m_mission_data.stage = 0;
            m_mission_state = STATE_MISSION;
            qInfo() << "Takeoff completed.";
            QGeoCoordinate dest = m_mav->coordinate().atDistanceAndAzimuth(0, flight_direction); // 0 -> North, 90 (M_PI / 2) -> East
            m_mav->guidedModeGotoLocation(dest);
        }
    }
}

void UBAgent::stateLand() {
    switch (m_mission_data.stage) {
        case (0): {
            if (m_mav->altitudeRelative()->rawValue().toDouble() < POINT_ZONE) {
                m_mission_data.stage++;
                qInfo() << "Land completed. Waiting for disarm";
            }
            break;
        }
        case (1): {
            if (!m_mav->armed()) {
                m_mission_data.tick=0;
                m_mission_data.stage++;
                qInfo() << "Motors stopped. Waiting for 3 seconds.";
            }
            break;
        }
        case (2): {
            m_mission_data.tick++;
            if (m_mission_data.tick >= (3.0 * 1.0 / MISSION_TRACK_DELAY - 0.001)) { //waiting for 3 seconds to stabilize
                m_mission_data.stage = 0;
                m_mission_state = STATE_IDLE;
            }
            break;
         }
    }
}


void UBAgent::logInfo() {
    QByteArray info;
    unsigned long int now;
    now = QDateTime::currentMSecsSinceEpoch();
    info += QByteArray::number(now/1000.0, 'f', 3);
    info += "\tLAT="+ QByteArray::number(m_mav->latitude(), 'f', 20);
    info += "\tLON="+ QByteArray::number(m_mav->longitude(), 'f', 20);
    info += "\tALT="+ QByteArray::number(m_mav->altitudeRelative()->rawValue().toDouble(), 'f', 20);
    info += "\tVEL="+ QByteArray::number(m_mav->groundSpeed()->rawValue().toDouble(), 'f', 20);
    m_power->sendData(UBPower::PWR_INFO, info);
}

void UBAgent::stateMission() {
    static QGeoCoordinate dest;

    switch (m_mission_data.stage) {

        case (0): {
            m_mission_data.tick = 0;
            m_mission_data.stage++;
            qInfo() << "Turning...";
            m_mav->sendMavCommand(m_mav->defaultComponentId(),  //fix heading
                            MAV_CMD_CONDITION_YAW,
                            true, // show error
                            (flight_direction)%360, 10.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
	    break;
	}
        // waiting while fix heading
        case (1): {
            m_mission_data.tick++;
            if (m_mission_data.tick >= (target_wait_time * 1.0 / MISSION_TRACK_DELAY - 0.001)) {
                qInfo() << "Heading forward, starting power measurement.";
                m_power->sendData(UBPower::PWR_START, QByteArray());
                m_mission_data.tick = 0;
                m_mission_data.stage++;
            }
            break;
         }
        // move
        case (2): {
            dest = m_mav->coordinate().atDistanceAndAzimuth(flight_distance, flight_direction); // 0 -> North, 90 (M_PI / 2) -> East
            m_mav->guidedModeGotoLocation(dest);
            m_mission_data.tick=0;
            m_mission_data.stage++;
            break;
        }    
        // reaching and waiting
        case (3): {
	    // sending information to the logger
            
    	    if ((m_mav->coordinate().distanceTo(dest) < POINT_ZONE) &&
                  (abs(m_mav->coordinate().altitude() - dest.altitude()) < POINT_ZONE)) {
                m_mission_data.tick++;
            }
            if (m_mission_data.tick >= (target_stabilize_time * 1.0 / MISSION_TRACK_DELAY - 0.001)) {       
                qInfo() << "Reached target, stopping measurement";
                m_power->sendData(UBPower::PWR_STOP, QByteArray());
                m_mission_data.tick=0;
                m_mission_data.stage++;
            }
            break;
        }
        // landing? or skipping it? TODO 
        case (4): {
            m_mission_data.tick=0;
            m_mission_data.stage++;
            break;
        }
        // take off? and fix heading
        case (5): {
            qInfo() << "Turning...";
            m_mav->sendMavCommand(m_mav->defaultComponentId(),
                            MAV_CMD_CONDITION_YAW,
                            true, // show error
                            (180+flight_direction)%360, 10.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            m_mission_data.stage++;
            break;
        }
        // waiting while fix heading
        case (6): {
            m_mission_data.tick++;
            if (m_mission_data.tick >= (target_wait_time * 1.0 / MISSION_TRACK_DELAY - 0.001)) {
                qInfo() << "Heading back, starting power measurement.";
                m_power->sendData(UBPower::PWR_START, QByteArray());
                m_mission_data.stage++;
            }
            break;
         }
        // move back
        case (7): {
            dest = m_mav->coordinate().atDistanceAndAzimuth(flight_distance, (180+flight_direction)%360); // 0 -> North, 90 (M_PI / 2) -> East
            m_mav->guidedModeGotoLocation(dest);
            m_mission_data.tick=0;
            m_mission_data.stage++;
            break;
        }    
        // reaching and waiting
        case (8): {
	    // sending information to the logger TODO            
    	    if ((m_mav->coordinate().distanceTo(dest) < POINT_ZONE) &&
                  (abs(m_mav->coordinate().altitude() - dest.altitude()) < POINT_ZONE)) {
                m_mission_data.tick++;
            }
            if (m_mission_data.tick >= (target_stabilize_time * 1.0 / MISSION_TRACK_DELAY - 0.001)) {       
                qInfo() << "Reached target, stopping measurement.";
                m_power->sendData(UBPower::PWR_STOP, QByteArray());
                qInfo() << "Landing....";
                m_mission_data.stage=0;
                m_mission_state = STATE_LAND;
                m_mav->guidedModeLand();
            }
            break;
        }
//        //  arming next uav
//        case (2): {
//            m_net->sendData(m_mav->id() + 1, QByteArray(1, MAV_CMD_NAV_TAKEOFF));
//            m_mission_data.stage++;
//        }
    }
    logInfo();
}
