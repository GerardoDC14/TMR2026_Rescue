/****************************************************************************
** Meta object code from reading C++ file 'dashboard_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../shared/gui_ws/src/gui/include/gui/dashboard_panel.hpp"
#include <QtGui/qtextcursor.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'dashboard_panel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_DashboardPanel_t {
    const uint offsetsAndSize[80];
    char stringdata0[504];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_DashboardPanel_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_DashboardPanel_t qt_meta_stringdata_DashboardPanel = {
    {
QT_MOC_LITERAL(0, 14), // "DashboardPanel"
QT_MOC_LITERAL(15, 21), // "resetSourcesRequested"
QT_MOC_LITERAL(37, 0), // ""
QT_MOC_LITERAL(38, 17), // "settingsRequested"
QT_MOC_LITERAL(56, 19), // "magnetometerUpdated"
QT_MOC_LITERAL(76, 1), // "x"
QT_MOC_LITERAL(78, 1), // "y"
QT_MOC_LITERAL(80, 1), // "z"
QT_MOC_LITERAL(82, 10), // "gasUpdated"
QT_MOC_LITERAL(93, 5), // "value"
QT_MOC_LITERAL(99, 10), // "imuUpdated"
QT_MOC_LITERAL(110, 3), // "yaw"
QT_MOC_LITERAL(114, 5), // "pitch"
QT_MOC_LITERAL(120, 4), // "roll"
QT_MOC_LITERAL(125, 17), // "telemetryReceived"
QT_MOC_LITERAL(143, 13), // "uptimeUpdated"
QT_MOC_LITERAL(157, 8), // "uptime_s"
QT_MOC_LITERAL(166, 14), // "hwEstopChanged"
QT_MOC_LITERAL(181, 6), // "active"
QT_MOC_LITERAL(188, 18), // "solverModeReceived"
QT_MOC_LITERAL(207, 4), // "mode"
QT_MOC_LITERAL(212, 14), // "onEstopToggled"
QT_MOC_LITERAL(227, 7), // "checked"
QT_MOC_LITERAL(235, 14), // "onAudioToggled"
QT_MOC_LITERAL(250, 21), // "onMagnetometerUpdated"
QT_MOC_LITERAL(272, 12), // "onGasUpdated"
QT_MOC_LITERAL(285, 12), // "onImuUpdated"
QT_MOC_LITERAL(298, 22), // "onTranscriptionUpdated"
QT_MOC_LITERAL(321, 4), // "text"
QT_MOC_LITERAL(326, 19), // "onTelemetryReceived"
QT_MOC_LITERAL(346, 16), // "onHeartbeatCheck"
QT_MOC_LITERAL(363, 15), // "onUptimeUpdated"
QT_MOC_LITERAL(379, 10), // "onClearAll"
QT_MOC_LITERAL(390, 17), // "publishEstopState"
QT_MOC_LITERAL(408, 15), // "onSensorToggled"
QT_MOC_LITERAL(424, 16), // "onHwEstopChanged"
QT_MOC_LITERAL(441, 15), // "onSolverToggled"
QT_MOC_LITERAL(457, 20), // "onSolverModeReceived"
QT_MOC_LITERAL(478, 17), // "setThermalEnabled"
QT_MOC_LITERAL(496, 7) // "enabled"

    },
    "DashboardPanel\0resetSourcesRequested\0"
    "\0settingsRequested\0magnetometerUpdated\0"
    "x\0y\0z\0gasUpdated\0value\0imuUpdated\0yaw\0"
    "pitch\0roll\0telemetryReceived\0uptimeUpdated\0"
    "uptime_s\0hwEstopChanged\0active\0"
    "solverModeReceived\0mode\0onEstopToggled\0"
    "checked\0onAudioToggled\0onMagnetometerUpdated\0"
    "onGasUpdated\0onImuUpdated\0"
    "onTranscriptionUpdated\0text\0"
    "onTelemetryReceived\0onHeartbeatCheck\0"
    "onUptimeUpdated\0onClearAll\0publishEstopState\0"
    "onSensorToggled\0onHwEstopChanged\0"
    "onSolverToggled\0onSolverModeReceived\0"
    "setThermalEnabled\0enabled"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DashboardPanel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      25,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       9,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,  164,    2, 0x06,    1 /* Public */,
       3,    0,  165,    2, 0x06,    2 /* Public */,
       4,    3,  166,    2, 0x06,    3 /* Public */,
       8,    1,  173,    2, 0x06,    7 /* Public */,
      10,    3,  176,    2, 0x06,    9 /* Public */,
      14,    0,  183,    2, 0x06,   13 /* Public */,
      15,    1,  184,    2, 0x06,   14 /* Public */,
      17,    1,  187,    2, 0x06,   16 /* Public */,
      19,    1,  190,    2, 0x06,   18 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      21,    1,  193,    2, 0x08,   20 /* Private */,
      23,    1,  196,    2, 0x08,   22 /* Private */,
      24,    3,  199,    2, 0x08,   24 /* Private */,
      25,    1,  206,    2, 0x08,   28 /* Private */,
      26,    3,  209,    2, 0x08,   30 /* Private */,
      27,    1,  216,    2, 0x08,   34 /* Private */,
      29,    0,  219,    2, 0x08,   36 /* Private */,
      30,    0,  220,    2, 0x08,   37 /* Private */,
      31,    1,  221,    2, 0x08,   38 /* Private */,
      32,    0,  224,    2, 0x08,   40 /* Private */,
      33,    0,  225,    2, 0x08,   41 /* Private */,
      34,    0,  226,    2, 0x08,   42 /* Private */,
      35,    1,  227,    2, 0x08,   43 /* Private */,
      36,    1,  230,    2, 0x08,   45 /* Private */,
      37,    1,  233,    2, 0x08,   47 /* Private */,
      38,    1,  236,    2, 0x0a,   49 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,
    QMetaType::Void, QMetaType::Double,    9,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   11,   12,   13,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Float,   16,
    QMetaType::Void, QMetaType::Bool,   18,
    QMetaType::Void, QMetaType::QString,   20,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,   22,
    QMetaType::Void, QMetaType::Bool,   22,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,
    QMetaType::Void, QMetaType::Double,    9,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   11,   12,   13,
    QMetaType::Void, QMetaType::QString,   28,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Float,   16,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,   18,
    QMetaType::Void, QMetaType::Bool,   22,
    QMetaType::Void, QMetaType::QString,   20,
    QMetaType::Void, QMetaType::Bool,   39,

       0        // eod
};

void DashboardPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<DashboardPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->resetSourcesRequested(); break;
        case 1: _t->settingsRequested(); break;
        case 2: _t->magnetometerUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 3: _t->gasUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1]))); break;
        case 4: _t->imuUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 5: _t->telemetryReceived(); break;
        case 6: _t->uptimeUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1]))); break;
        case 7: _t->hwEstopChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 8: _t->solverModeReceived((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 9: _t->onEstopToggled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 10: _t->onAudioToggled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 11: _t->onMagnetometerUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 12: _t->onGasUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1]))); break;
        case 13: _t->onImuUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 14: _t->onTranscriptionUpdated((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 15: _t->onTelemetryReceived(); break;
        case 16: _t->onHeartbeatCheck(); break;
        case 17: _t->onUptimeUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1]))); break;
        case 18: _t->onClearAll(); break;
        case 19: _t->publishEstopState(); break;
        case 20: _t->onSensorToggled(); break;
        case 21: _t->onHwEstopChanged((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 22: _t->onSolverToggled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 23: _t->onSolverModeReceived((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 24: _t->setThermalEnabled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (DashboardPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::resetSourcesRequested)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::settingsRequested)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)(double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::magnetometerUpdated)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::gasUpdated)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)(double , double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::imuUpdated)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::telemetryReceived)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)(float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::uptimeUpdated)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)(bool );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::hwEstopChanged)) {
                *result = 7;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::solverModeReceived)) {
                *result = 8;
                return;
            }
        }
    }
}

const QMetaObject DashboardPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_DashboardPanel.offsetsAndSize,
    qt_meta_data_DashboardPanel,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_DashboardPanel_t
, QtPrivate::TypeAndForceComplete<DashboardPanel, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>


>,
    nullptr
} };


const QMetaObject *DashboardPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *DashboardPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_DashboardPanel.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int DashboardPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 25)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 25;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 25)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 25;
    }
    return _id;
}

// SIGNAL 0
void DashboardPanel::resetSourcesRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}

// SIGNAL 1
void DashboardPanel::settingsRequested()
{
    QMetaObject::activate(this, &staticMetaObject, 1, nullptr);
}

// SIGNAL 2
void DashboardPanel::magnetometerUpdated(double _t1, double _t2, double _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void DashboardPanel::gasUpdated(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void DashboardPanel::imuUpdated(double _t1, double _t2, double _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void DashboardPanel::telemetryReceived()
{
    QMetaObject::activate(this, &staticMetaObject, 5, nullptr);
}

// SIGNAL 6
void DashboardPanel::uptimeUpdated(float _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void DashboardPanel::hwEstopChanged(bool _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}

// SIGNAL 8
void DashboardPanel::solverModeReceived(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 8, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
