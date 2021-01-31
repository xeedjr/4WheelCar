//.$file${.::RosserialQM.h} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//
// Model: RosserialQM.qm
// File:  ${.::RosserialQM.h}
//
// This code has been generated by QM 5.1.0 <www.state-machine.com/qm/>.
// DO NOT EDIT THIS FILE MANUALLY. All your changes will be lost.
//
// This program is open source software: you can redistribute it and/or
// modify it under the terms of the GNU General Public License as published
// by the Free Software Foundation.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
// for more details.
//
//.$endhead${.::RosserialQM.h} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#include "qpcpp.hpp" // QP/C++ framework API
#include <functional>

namespace ros_serial {

#define VIRTUAL_FUNCTIONS \
    virtual bool initialize(const QP::QEvt *e) = 0; \
    virtual bool process_in_data(const QP::QEvt *e) = 0; \
    virtual bool timer1(const QP::QEvt *e) = 0; \


enum Signals {
    TIMEOUT_SIG = QP::Q_USER_SIG, // time event timeout
    TIMER1_SIG,

    INITIALIZE_SIG,

    /// External
    RECEIVED_BYTE_SIG,

    MAX_SIG         // the last signal
};

};

// ask QM to declare the Blinky class ----------------------------------------
//.$declare${ros_serial::RosserialQM} vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
namespace ros_serial {

//.${ros_serial::RosserialQM} ................................................
class RosserialQM : public QP::QActive {
private:
      VIRTUAL_FUNCTIONS;
    QP::QTimeEvt spin_timeEvt{this, RECEIVED_BYTE_SIG, 0U};
    QP::QTimeEvt timer1_timeEvt{this, TIMER1_SIG, 0U};

public:
    RosserialQM();

protected:
    Q_STATE_DECL(initial);
    Q_STATE_DECL(initialize);
    Q_STATE_DECL(Ready);
};

} // namespace ros_serial
//.$enddecl${ros_serial::RosserialQM} ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
