/****************************************************************************
** Meta object code from reading C++ file 'odometry_panel.hpp'
**
** Created by: The Qt Meta Object Compiler version 68 (Qt 6.2.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/gui/include/gui/odometry_panel.hpp"
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
    const uint offsetsAndSize[48];
    char stringdata0[245];
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
QT_MOC_LITERAL(161, 18), // "onTelemetryUpdated"
QT_MOC_LITERAL(180, 19), // "onFlipperExtUpdated"
QT_MOC_LITERAL(200, 13), // "onModeUpdated"
QT_MOC_LITERAL(214, 14), // "onFlagsUpdated"
QT_MOC_LITERAL(229, 15) // "onTracksUpdated"

    },
    "OdometryPanel\0telemetryUpdated\0\0spd_l\0"
    "spd_r\0flip_angle\0uptime\0flipperExtUpdated\0"
    "fl\0fr\0rl\0rr\0modeUpdated\0mode\0flagsUpdated\0"
    "flags\0tracksUpdated\0left_rpm\0right_rpm\0"
    "onTelemetryUpdated\0onFlipperExtUpdated\0"
    "onModeUpdated\0onFlagsUpdated\0"
    "onTracksUpdated"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OdometryPanel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    4,   74,    2, 0x06,    1 /* Public */,
       7,    4,   83,    2, 0x06,    6 /* Public */,
      12,    1,   92,    2, 0x06,   11 /* Public */,
      14,    1,   95,    2, 0x06,   13 /* Public */,
      16,    2,   98,    2, 0x06,   15 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      19,    4,  103,    2, 0x08,   18 /* Private */,
      20,    4,  112,    2, 0x08,   23 /* Private */,
      21,    1,  121,    2, 0x08,   28 /* Private */,
      22,    1,  124,    2, 0x08,   30 /* Private */,
      23,    2,  127,    2, 0x08,   32 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,    6,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    8,    9,   10,   11,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   17,   18,

 // slots: parameters
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    3,    4,    5,    6,
    QMetaType::Void, QMetaType::Float, QMetaType::Float, QMetaType::Float, QMetaType::Float,    8,    9,   10,   11,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, QMetaType::Int,   15,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   17,   18,

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
        case 5: _t->onTelemetryUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[4]))); break;
        case 6: _t->onFlipperExtUpdated((*reinterpret_cast< std::add_pointer_t<float>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[3])),(*reinterpret_cast< std::add_pointer_t<float>>(_a[4]))); break;
        case 7: _t->onModeUpdated((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 8: _t->onFlagsUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 9: _t->onTracksUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2]))); break;
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
    }
}

const QMetaObject OdometryPanel::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_OdometryPanel.offsetsAndSize,
    qt_meta_data_OdometryPanel,
    qt_static_metacall,
    nullptr,
qt_incomplete_metaTypeArray<qt_meta_stringdata_OdometryPanel_t
, QtPrivate::TypeAndForceComplete<OdometryPanel, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<float, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>


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
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 10;
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
QT_WARNING_POP
QT_END_MOC_NAMESPACE
