/****************************************************************************
** Meta object code from reading C++ file 'odometry_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../shared/gui_ws/src/gui/include/gui/odometry_panel.hpp"
#include <QtGui/qtextcursor.h>
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'odometry_panel.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 68
#error "This file was generated using the moc from 6.2.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_OdometryPanel_t {
    const uint offsetsAndSize[86];
    char stringdata0[460];
};
#define QT_MOC_LITERAL(ofs, len) \
    uint(offsetof(qt_meta_stringdata_OdometryPanel_t, stringdata0) + ofs), len 
static const qt_meta_stringdata_OdometryPanel_t qt_meta_stringdata_OdometryPanel = {
    {
QT_MOC_LITERAL(0, 13), // "OdometryPanel"
QT_MOC_LITERAL(14, 16), // "telemetryUpdated"
QT_MOC_LITERAL(31, 0), // ""
QT_MOC_LITERAL(32, 5), // "spd_l"
QT_MOC_LITERAL(38, 5), // "spd_r"
QT_MOC_LITERAL(44, 10), // "flip_angle"
QT_MOC_LITERAL(55, 6), // "uptime"
QT_MOC_LITERAL(62, 17), // "flipperExtUpdated"
QT_MOC_LITERAL(80, 2), // "fl"
QT_MOC_LITERAL(83, 2), // "fr"
QT_MOC_LITERAL(86, 2), // "rl"
QT_MOC_LITERAL(89, 2), // "rr"
QT_MOC_LITERAL(92, 11), // "modeUpdated"
QT_MOC_LITERAL(104, 4), // "mode"
QT_MOC_LITERAL(109, 12), // "flagsUpdated"
QT_MOC_LITERAL(122, 5), // "flags"
QT_MOC_LITERAL(128, 13), // "tracksUpdated"
QT_MOC_LITERAL(142, 8), // "left_rpm"
QT_MOC_LITERAL(151, 9), // "right_rpm"
QT_MOC_LITERAL(161, 17), // "vescStatusUpdated"
QT_MOC_LITERAL(179, 2), // "id"
QT_MOC_LITERAL(182, 4), // "erpm"
QT_MOC_LITERAL(187, 7), // "current"
QT_MOC_LITERAL(195, 4), // "duty"
QT_MOC_LITERAL(200, 8), // "temp_fet"
QT_MOC_LITERAL(209, 10), // "temp_motor"
QT_MOC_LITERAL(220, 7), // "voltage"
QT_MOC_LITERAL(228, 16), // "mainMotorUpdated"
QT_MOC_LITERAL(245, 9), // "left_duty"
QT_MOC_LITERAL(255, 10), // "right_duty"
QT_MOC_LITERAL(266, 12), // "flipper_duty"
QT_MOC_LITERAL(279, 16), // "odriveArmUpdated"
QT_MOC_LITERAL(296, 9), // "joint_idx"
QT_MOC_LITERAL(306, 4), // "iq_a"
QT_MOC_LITERAL(311, 6), // "vbus_v"
QT_MOC_LITERAL(318, 18), // "onTelemetryUpdated"
QT_MOC_LITERAL(337, 19), // "onFlipperExtUpdated"
QT_MOC_LITERAL(357, 13), // "onModeUpdated"
QT_MOC_LITERAL(371, 14), // "onFlagsUpdated"
QT_MOC_LITERAL(386, 15), // "onTracksUpdated"
QT_MOC_LITERAL(402, 19), // "onVescStatusUpdated"
QT_MOC_LITERAL(422, 18), // "onMainMotorUpdated"
QT_MOC_LITERAL(441, 18) // "onOdriveArmUpdated"

    },
    "OdometryPanel\0telemetryUpdated\0\0spd_l\0"
    "spd_r\0flip_angle\0uptime\0flipperExtUpdated\0"
    "fl\0fr\0rl\0rr\0modeUpdated\0mode\0flagsUpdated\0"
    "flags\0tracksUpdated\0left_rpm\0right_rpm\0"
    "vescStatusUpdated\0id\0erpm\0current\0"
    "duty\0temp_fet\0temp_motor\0voltage\0"
    "mainMotorUpdated\0left_duty\0right_duty\0"
    "flipper_duty\0odriveArmUpdated\0joint_idx\0"
    "iq_a\0vbus_v\0onTelemetryUpdated\0"
    "onFlipperExtUpdated\0onModeUpdated\0"
    "onFlagsUpdated\0onTracksUpdated\0"
    "onVescStatusUpdated\0onMainMotorUpdated\0"
    "onOdriveArmUpdated"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OdometryPanel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       8,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    4,  110,    2, 0x06,    1 /* Public */,
       7,    4,  119,    2, 0x06,    6 /* Public */,
      12,    1,  128,    2, 0x06,   11 /* Public */,
      14,    1,  131,    2, 0x06,   13 /* Public */,
      16,    2,  134,    2, 0x06,   15 /* Public */,
      19,    7,  139,    2, 0x06,   18 /* Public */,
      27,    3,  154,    2, 0x06,   26 /* Public */,
      31,    3,  161,    2, 0x06,   30 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      35,    4,  168,    2, 0x08,   34 /* Private */,
      36,    4,  177,    2, 0x08,   39 /* Private */,
      37,    1,  186,    2, 0x08,   44 /* Private */,
      38,    1,  189,    2, 0x08,   46 /* Private */,
      39,    2,  192,    2, 0x08,   48 /* Private */,
      40,    7,  197,    2, 0x08,   51 /* Private */,
      41,    3,  212,    2, 0x08,   59 /* Private */,
      42,    3,  219,    2, 0x08,   63 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,    6,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    8,    9,   10,   11,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   17,   18,
    QMetaType::Void, QMetaType::Int, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,   20,   21,   22,   23,   24,   25,   26,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float,   28,   29,   30,
    QMetaType::Void, QMetaType::Int, QMetaType::Float, QMetaType::Float,   32,   33,   34,

 // slots: parameters
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,    6,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    8,    9,   10,   11,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   17,   18,
    QMetaType::Void, QMetaType::Int, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,   20,   21,   22,   23,   24,   25,   26,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float,   28,   29,   30,
    QMetaType::Void, QMetaType::Int, QMetaType::Float, QMetaType::Float,   32,   33,   34,

       0        // eod
};

void OdometryPanel::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<OdometryPanel *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->telemetryUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[4]))); break;
        case 1: _t->flipperExtUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[4]))); break;
        case 2: _t->modeUpdated((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 3: _t->flagsUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 4: _t->tracksUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2]))); break;
        case 5: _t->vescStatusUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[5])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[6])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[7]))); break;
        case 6: _t->mainMotorUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3]))); break;
        case 7: _t->odriveArmUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3]))); break;
        case 8: _t->onTelemetryUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[4]))); break;
        case 9: _t->onFlipperExtUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[4]))); break;
        case 10: _t->onModeUpdated((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 11: _t->onFlagsUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 12: _t->onTracksUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2]))); break;
        case 13: _t->onVescStatusUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[4])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[5])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[6])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[7]))); break;
        case 14: _t->onMainMotorUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3]))); break;
        case 15: _t->onOdriveArmUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (OdometryPanel::*)(float , float , float , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OdometryPanel::telemetryUpdated)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (OdometryPanel::*)(float , float , float , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OdometryPanel::flipperExtUpdated)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (OdometryPanel::*)(const QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OdometryPanel::modeUpdated)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (OdometryPanel::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OdometryPanel::flagsUpdated)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (OdometryPanel::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OdometryPanel::tracksUpdated)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (OdometryPanel::*)(int , float , float , float , float , float , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OdometryPanel::vescStatusUpdated)) {
                *result = 5;
                return;
            }
        }
        {
            using _t = void (OdometryPanel::*)(float , float , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OdometryPanel::mainMotorUpdated)) {
                *result = 6;
                return;
            }
        }
        {
            using _t = void (OdometryPanel::*)(int , float , float );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&OdometryPanel::odriveArmUpdated)) {
                *result = 7;
                return;
            }
        }
    }
}

const QMetaObject OdometryPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_OdometryPanel.offsetsAndSize,
    qt_meta_data_OdometryPanel,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_OdometryPanel_t
, QtPrivate::TypeAndForceComplete<OdometryPanel, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>


>,
    nullptr
} };


const QMetaObject *OdometryPanel::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OdometryPanel::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_OdometryPanel.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int OdometryPanel::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void OdometryPanel::telemetryUpdated(float _t1, float _t2, float _t3, float _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void OdometryPanel::flipperExtUpdated(float _t1, float _t2, float _t3, float _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void OdometryPanel::modeUpdated(const QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void OdometryPanel::flagsUpdated(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void OdometryPanel::tracksUpdated(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void OdometryPanel::vescStatusUpdated(int _t1, float _t2, float _t3, float _t4, float _t5, float _t6, float _t7)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t5))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t6))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t7))) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}

// SIGNAL 6
void OdometryPanel::mainMotorUpdated(float _t1, float _t2, float _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 6, _a);
}

// SIGNAL 7
void OdometryPanel::odriveArmUpdated(int _t1, float _t2, float _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 7, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
