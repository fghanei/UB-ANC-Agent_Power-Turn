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

int dest_index=0;
int flight_direction = 180; //0  north, 90 east
float flight_distance = 50;
float flight_speed = 5.0;
float target_wait_time = 1.0; //in seconds
QGeoCoordinate dest[17], start_point, mid_point, turn0_point, turn45_point, turn90_point, turn135_point;

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


// hardcode west_north waypoints:
start_point    = QGeoCoordinate(43.0088501, -78.7897001, 5.0);
mid_point      = QGeoCoordinate(43.0088501, -78.7903151, 5.0);
turn0_point    = QGeoCoordinate(43.0088501, -78.7909301, 5.0);
turn45_point   = QGeoCoordinate(43.0091681, -78.7907501, 5.0);
turn90_point   = QGeoCoordinate(43.0093001, -78.7903151, 5.0);
turn135_point  = QGeoCoordinate(43.0091681, -78.7898801, 5.0);

/*
// hardcode south_west waypoints:
start_point    = QGeoCoordinate(43.0093001, -78.7897001, 5.0);
mid_point      = QGeoCoordinate(43.0088501, -78.7897001, 5.0);
turn0_point    = QGeoCoordinate(43.0084011, -78.7897001, 5.0);
turn45_point   = QGeoCoordinate(43.0085321, -78.7901351, 5.0);
turn90_point   = QGeoCoordinate(43.0088501, -78.7903151, 5.0);
turn135_point  = QGeoCoordinate(43.0091681, -78.7901351, 5.0);
*/

// generate waypoints:
/*
start_point   = QGeoCoordinate(43.0093001, -78.7897001, 5.0);
mid_point     = start_point.atDistanceAndAzimuth(flight_distance, flight_direction);
turn0_point   = mid_point.atDistanceAndAzimuth(flight_distance, flight_direction);
turn45_point  = mid_point.atDistanceAndAzimuth(flight_distance, (flight_direction + 45)  % 360);
turn90_point  = mid_point.atDistanceAndAzimuth(flight_distance, (flight_direction + 90)  % 360);
turn135_point = mid_point.atDistanceAndAzimuth(flight_distance, (flight_direction + 135) % 360);


    qInfo() << "========";
    qInfo() << "start:   " << 43.00f << 1000000.0 * (start_point.latitude()-43.0)   << -78 << -1000000.0 * (start_point.longitude()+78.0);
    qInfo() << "mid:     " << 43.00f << 1000000.0 * (mid_point.latitude()-43.0)     << -78 << -1000000.0 * (mid_point.longitude()+78.0);
    qInfo() << "turn0:   " << 43.00f << 1000000.0 * (turn0_point.latitude()-43.0)   << -78 << -1000000.0 * (turn0_point.longitude()+78.0);
    qInfo() << "turn45:  " << 43.00f << 1000000.0 * (turn45_point.latitude()-43.0)  << -78 << -1000000.0 * (turn45_point.longitude()+78.0);
    qInfo() << "turn90:  " << 43.00f << 1000000.0 * (turn90_point.latitude()-43.0)  << -78 << -1000000.0 * (turn90_point.longitude()+78.0);
    qInfo() << "turn135: " << 43.00f << 1000000.0 * (turn135_point.latitude()-43.0) << -78 << -1000000.0 * (turn135_point.longitude()+78.0);

*/

            dest[0]  = start_point;
            dest[1]  = mid_point; 
            dest[2]  = turn0_point;
            dest[3]  = mid_point; 
            dest[4]  = start_point;
            dest[5]  = mid_point;
            dest[6]  = turn45_point;
            dest[7]  = mid_point;
            dest[8]  = start_point;
            dest[9]  = mid_point;
            dest[10] = turn90_point;
            dest[11] = mid_point;
            dest[12] = start_point;
            dest[13] = mid_point;
            dest[14] = turn135_point;
            dest[15] = mid_point;
            dest[16] = start_point;

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
    dest_index=0;
    m_power->sendData(UBPower::PWR_STOP, QByteArray());
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
            qInfo() << "Takeoff completed.";

    	    m_mav->sendMavCommand(m_mav->defaultComponentId(),  //fix flight speed
                    MAV_CMD_DO_CHANGE_SPEED,
                    true, // show error
                    1, flight_speed, -1, 0, 0, 0, 0);            

//            m_mav->guidedModeGotoLocation(start_point);

            m_mission_data.stage = 0;
            m_mission_state = STATE_MISSION;
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
                qInfo() << "You may now restart the mission by changing to LAND and then GUIDED mode.";
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
//    qInfo() << "dest_index: " << dest_index << " / stage: " << m_mission_data.stage;
    switch (m_mission_data.stage) {
        // waiting till target
        case (0): {
            if (dest_index%2!=0) {
                qInfo() << "WRONG STATE";
            }
            if (m_mav->coordinate().distanceTo(dest[dest_index]) < 5*POINT_ZONE) {

                qInfo() << "Getting close to the target, stopping power measurement.";
                m_power->sendData(UBPower::PWR_STOP, QByteArray());
                m_mission_data.stage++;
                m_mission_data.tick = 0;
            }
            break;
        }
        // waiting till getting ready for heading again
        case (1): {
            if (dest_index%2!=0) {
                qInfo() << "WRONG STATE";
            }
            m_mission_data.tick++;
            if (m_mission_data.tick > (target_wait_time * 1.0 / MISSION_TRACK_DELAY - 0.001)) {
                //enough waiting, we should be at the point, now we'll wait to leave again.
                m_mission_data.stage++;
                m_mission_data.tick = 0;
                dest_index++;
            }
            break;
        }
        // start measuring as heading
        case (2): {
            if (dest_index%2!=1) {
                qInfo() << "WRONG STATE";
            }
            if (m_mav->coordinate().distanceTo(dest[dest_index-1]) > 10*POINT_ZONE) {
                qInfo() << "Getting away from heading point, starting power measurement.";
                m_power->sendData(UBPower::PWR_STOP, QByteArray());//stop any unfinished measurements, which should be ignored
                m_power->sendData(UBPower::PWR_START, QByteArray());
                m_mission_data.stage++;
            }
            break;
        }
        // waiting till mipoint for EVENT
        case (3): {
            if (dest_index%2!=1) {
                qInfo() << "WRONG STATE";
            }
            if (m_mav->coordinate().distanceTo(dest[dest_index]) < 5*POINT_ZONE) {
                qInfo() << "Getting close to the mid_point, sending EVENT.";
                m_power->sendData(UBPower::PWR_EVENT, QByteArray());
                m_mission_data.stage=0;
                dest_index++;
            }
            break;
        }
    }

    // sending information to the logger
    logInfo();
}
