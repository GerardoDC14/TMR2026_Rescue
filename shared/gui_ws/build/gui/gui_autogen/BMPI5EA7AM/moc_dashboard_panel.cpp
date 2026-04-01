/****************************************************************************
** Meta object code from reading C++ file 'dashboard_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/gui/include/gui/dashboard_panel.hpp"
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
    const uint offsetsAndSize[62];
    char stringdata0[378];
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
QT_MOC_LITERAL(166, 14), // "onEstopToggled"
QT_MOC_LITERAL(181, 7), // "checked"
QT_MOC_LITERAL(189, 14), // "onAudioToggled"
QT_MOC_LITERAL(204, 21), // "onMagnetometerUpdated"
QT_MOC_LITERAL(226, 12), // "onGasUpdated"
QT_MOC_LITERAL(239, 12), // "onImuUpdated"
QT_MOC_LITERAL(252, 22), // "onTranscriptionUpdated"
QT_MOC_LITERAL(275, 4), // "text"
QT_MOC_LITERAL(280, 19), // "onTelemetryReceived"
QT_MOC_LITERAL(300, 16), // "onHeartbeatCheck"
QT_MOC_LITERAL(317, 15), // "onUptimeUpdated"
QT_MOC_LITERAL(333, 10), // "onClearAll"
QT_MOC_LITERAL(344, 17), // "publishEstopState"
QT_MOC_LITERAL(362, 15) // "onSensorToggled"

    },
    "DashboardPanel\0resetSourcesRequested\0"
    "\0settingsRequested\0magnetometerUpdated\0"
    "x\0y\0z\0gasUpdated\0value\0imuUpdated\0yaw\0"
    "pitch\0roll\0telemetryReceived\0uptimeUpdated\0"
    "uptime_s\0onEstopToggled\0checked\0"
    "onAudioToggled\0onMagnetometerUpdated\0"
    "onGasUpdated\0onImuUpdated\0"
    "onTranscriptionUpdated\0text\0"
    "onTelemetryReceived\0onHeartbeatCheck\0"
    "onUptimeUpdated\0onClearAll\0publishEstopState\0"
    "onSensorToggled"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DashboardPanel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      19,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       7,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,  128,    2, 0x06,    1 /* Public */,
       3,    0,  129,    2, 0x06,    2 /* Public */,
       4,    3,  130,    2, 0x06,    3 /* Public */,
       8,    1,  137,    2, 0x06,    7 /* Public */,
      10,    3,  140,    2, 0x06,    9 /* Public */,
      14,    0,  147,    2, 0x06,   13 /* Public */,
      15,    1,  148,    2, 0x06,   14 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      17,    1,  151,    2, 0x08,   16 /* Private */,
      19,    1,  154,    2, 0x08,   18 /* Private */,
      20,    3,  157,    2, 0x08,   20 /* Private */,
      21,    1,  164,    2, 0x08,   24 /* Private */,
      22,    3,  167,    2, 0x08,   26 /* Private */,
      23,    1,  174,    2, 0x08,   30 /* Private */,
      25,    0,  177,    2, 0x08,   32 /* Private */,
      26,    0,  178,    2, 0x08,   33 /* Private */,
      27,    1,  179,    2, 0x08,   34 /* Private */,
      28,    0,  182,    2, 0x08,   36 /* Private */,
      29,    0,  183,    2, 0x08,   37 /* Private */,
      30,    0,  184,    2, 0x08,   38 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,
    QMetaType::Void, QMetaType::Double,    9,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   11,   12,   13,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Float,   16,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,   18,
    QMetaType::Void, QMetaType::Bool,   18,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,
    QMetaType::Void, QMetaType::Double,    9,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,   11,   12,   13,
    QMetaType::Void, QMetaType::QString,   24,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Float,   16,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

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
        case 7: _t->onEstopToggled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 8: _t->onAudioToggled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 9: _t->onMagnetometerUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 10: _t->onGasUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1]))); break;
        case 11: _t->onImuUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 12: _t->onTranscriptionUpdated((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 13: _t->onTelemetryReceived(); break;
        case 14: _t->onHeartbeatCheck(); break;
        case 15: _t->onUptimeUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1]))); break;
        case 16: _t->onClearAll(); break;
        case 17: _t->publishEstopState(); break;
        case 18: _t->onSensorToggled(); break;
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
    }
}

const QMetaObject DashboardPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_DashboardPanel.offsetsAndSize,
    qt_meta_data_DashboardPanel,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_DashboardPanel_t
, QtPrivate::TypeAndForceComplete<DashboardPanel, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


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
        if (_id < 19)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 19;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 19)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 19;
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
