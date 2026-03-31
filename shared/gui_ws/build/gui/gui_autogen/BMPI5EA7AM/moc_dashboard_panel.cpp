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
    const uint offsetsAndSize[44];
    char stringdata0[284];
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
QT_MOC_LITERAL(99, 17), // "heartbeatReceived"
QT_MOC_LITERAL(117, 14), // "onEstopToggled"
QT_MOC_LITERAL(132, 7), // "checked"
QT_MOC_LITERAL(140, 14), // "onAudioToggled"
QT_MOC_LITERAL(155, 21), // "onMagnetometerUpdated"
QT_MOC_LITERAL(177, 12), // "onGasUpdated"
QT_MOC_LITERAL(190, 22), // "onTranscriptionUpdated"
QT_MOC_LITERAL(213, 4), // "text"
QT_MOC_LITERAL(218, 19), // "onHeartbeatReceived"
QT_MOC_LITERAL(238, 16), // "onHeartbeatCheck"
QT_MOC_LITERAL(255, 10), // "onClearAll"
QT_MOC_LITERAL(266, 17) // "publishEstopState"

    },
    "DashboardPanel\0resetSourcesRequested\0"
    "\0settingsRequested\0magnetometerUpdated\0"
    "x\0y\0z\0gasUpdated\0value\0heartbeatReceived\0"
    "onEstopToggled\0checked\0onAudioToggled\0"
    "onMagnetometerUpdated\0onGasUpdated\0"
    "onTranscriptionUpdated\0text\0"
    "onHeartbeatReceived\0onHeartbeatCheck\0"
    "onClearAll\0publishEstopState"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_DashboardPanel[] = {

 // content:
      10,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags, initial metatype offsets
       1,    0,   98,    2, 0x06,    1 /* Public */,
       3,    0,   99,    2, 0x06,    2 /* Public */,
       4,    3,  100,    2, 0x06,    3 /* Public */,
       8,    1,  107,    2, 0x06,    7 /* Public */,
      10,    0,  110,    2, 0x06,    9 /* Public */,

 // slots: name, argc, parameters, tag, flags, initial metatype offsets
      11,    1,  111,    2, 0x08,   10 /* Private */,
      13,    1,  114,    2, 0x08,   12 /* Private */,
      14,    3,  117,    2, 0x08,   14 /* Private */,
      15,    1,  124,    2, 0x08,   18 /* Private */,
      16,    1,  127,    2, 0x08,   20 /* Private */,
      18,    0,  130,    2, 0x08,   22 /* Private */,
      19,    0,  131,    2, 0x08,   23 /* Private */,
      20,    0,  132,    2, 0x08,   24 /* Private */,
      21,    0,  133,    2, 0x08,   25 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Bool,   12,
    QMetaType::Void, QMetaType::Double, QMetaType::Double, QMetaType::Double,    5,    6,    7,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, QMetaType::QString,   17,
    QMetaType::Void,
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
        case 3: _t->gasUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 4: _t->heartbeatReceived(); break;
        case 5: _t->onEstopToggled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 6: _t->onAudioToggled((*reinterpret_cast< std::add_pointer_t<bool>>(_a[1]))); break;
        case 7: _t->onMagnetometerUpdated((*reinterpret_cast< std::add_pointer_t<double>>(_a[1])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[2])),(*reinterpret_cast< std::add_pointer_t<double>>(_a[3]))); break;
        case 8: _t->onGasUpdated((*reinterpret_cast< std::add_pointer_t<int>>(_a[1]))); break;
        case 9: _t->onTranscriptionUpdated((*reinterpret_cast< std::add_pointer_t<QString>>(_a[1]))); break;
        case 10: _t->onHeartbeatReceived(); break;
        case 11: _t->onHeartbeatCheck(); break;
        case 12: _t->onClearAll(); break;
        case 13: _t->publishEstopState(); break;
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
            using _t = void (DashboardPanel::*)(int );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::gasUpdated)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (DashboardPanel::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&DashboardPanel::heartbeatReceived)) {
                *result = 4;
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
, QtPrivate::TypeAndForceComplete<DashboardPanel, std::true_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>
, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<bool, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<double, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<int, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<const QString &, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>, QtPrivate::TypeAndForceComplete<void, std::false_type>


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
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 14;
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
void DashboardPanel::gasUpdated(int _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void DashboardPanel::heartbeatReceived()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
