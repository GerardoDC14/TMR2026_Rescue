"""
Program test environment
Pyhone version:3.4.1
Firmware version:2.8.28
Dependent files(MacOSX):libGinkgo_Driver.dylib,libusb-0.1.4.dylib,libusb-1.0.0.dylib
Dependent files(Windows):Ginkgo_Driver.dll
Dependent files(Linux):libGinkgo_Driver.so,libusb-1.0.so
More product infomation:
http://www.viewtool.com/index.php/en/14-2016-07-26-07-18-35/57-2016-07-27-07-19-58

Data structure and API functionality definition detail please download: 
Ginkgo USB-CAN Interface API Library Instruction
http://www.viewtool.com/demo/Ginkgo/Documents/Ginkgo_USB-CAN_Interface_API_Library_Instruction_v1.2.zip
"""
from ctypes import *
import os
import platform
import struct

# Compatible with other CAN adapter datatype
class VCI_BOARD_INFO(Structure):  
    _fields_ = [
        ("hw_Version", c_ushort),    # hardware version, in hex format, for example: 0x0100 present version is 1.00
        ("fw_Version", c_ushort),    # firmware version in hex format
        ("dr_Version", c_ushort),    # driver version, in hex format
        ("in_Version",c_ushort),    # interface library version, in hex format
        ("irq_Num",c_ushort),        # interrupt number used by board
        ("can_Num",c_ubyte),        # CAN channel number
        ("str_Serial_Num",c_ubyte*20),    # CAN board serial number
        ("str_hw_Type",c_ubyte*40),    # string for hardware type,for example:"USBCAN V1.00\0"(note:include string null end'\0').
        ("Reserved",c_ushort*4)
    ];
	
# Board info datatype
class VCI_BOARD_INFO_EX(Structure):  
    _fields_ = [
        ("ProductName",     c_ubyte*64),    # hardware name,for example: "Ginkgo-CAN-Adaptor\0"(note: include string null end'\0')
        ("FirmwareVersion", c_ubyte*4),     # firmware version
        ("HardwareVersion", c_ubyte*4),     # hardware version
        ("SerialNumber",    c_ubyte*12),    # adatper serial number
        ("DllVersion",      c_ubyte*12),    # dll version.        
    ];
	
# Definition of CAN frame
class VCI_CAN_OBJ(Structure):  
    _fields_ = [("ID", c_uint),    # Frame ID
        ("TimeStamp", c_uint),    # timestamp of the frame arriving,started from initialization of CAN controller
        ("TimeFlag", c_byte),    # if using timestamp. 1: use TimeStamp, 0:not use. TimeFlag and TimeStamp is available when the frame is recived frame
        ("SendType",c_byte),    # send frame type. 0: normal send,1: single send,2: self send/receive,3: single self send/receive
        ("RemoteFlag",c_byte),    # remote frame flag
        ("ExternFlag",c_byte),    # extended frame flag
        ("DataLen",c_byte),     # Data length(<=8),how many uint8_ts of data
        ("Data",c_ubyte*8),      # text data
        ("Reserved",c_byte*3)    # reserved
    ];
# definition of CAN controller status
class VCI_CAN_STATUS(Structure):  
    _fields_ = [("ErrInterrupt", c_byte),    # interrupt record,will be cleared while reading
        ("regMode", c_byte),        # CAN controller mode register
        ("regStatus", c_byte),        # CAN controller status register
        ("regALCapture",c_byte),    # CAN controller arbitrator lost register
        ("regECCapture",c_byte),    # CAN controller error register
        ("regEWLimit",c_byte),        # CAN controller error alarm limitation register
        ("regRECounter",c_byte),    # CAN controller receive error register
        ("regTECounter",c_byte),    # CAN controller send error register
        ("regESR",c_uint),              # CAN controller status register
        ("regTSR",c_uint),              # CAN controller status register
        ("BufferSize",c_uint),        # CAN controller receive buffer size
        ("Reserved",c_uint),        # 
    ];

# definition of error data type
class VCI_ERR_INFO(Structure):
    _fields_ = [("ErrCode",c_uint),            # error code
        ("Passive_ErrData",c_ubyte*3),    # error identification data when error has passive error
        ("ArLost_ErrData",c_ubyte)    # error identification data when error has arbitration lost error
    ];
	
# definition of CAN initialization data type
class VCI_INIT_CONFIG(Structure):
    _fields_ = [("AccCode",c_uint),            # ACC code (for verification)
        ("AccMask",c_uint),        # Mask code
        ("Reserved",c_uint),        # 
        ("Filter",c_byte),        # filter type.0: double filter,1: single filter
        ("Timing0",c_byte),        # Timer 0
        ("Timing1",c_byte),        # Timer 1
        ("Mode",c_byte)
        ];
		
# Definition of CAN initialization data type
"""
CAN baudrate = 36MHz/(CAN_BRP)/(CAN_SJW+CAN_BS1+CAN_BS2)
baudrate（bps） CAN_BRP CAN_SJW CAN_BS1 CAN_BS2 
1M                9        1       2       1 
900K              5        1       5       2 
800K              3        1      10      4 
600K             12       1       3       1 
666K             6         1       7       1 
500K             12       1       4       1 
400K             10       1       7       1 
300K             15       1       6       1 
250K             18       1       6       1 
200K             10       1       15     2 
150K             30       1       6       1 
125K             36       1       6       1 
100K             45       1       6       1 
90K               50       1       6       1 
80K               75       1       4       1 
60K               75       1       6       1 
50K               90       1       6       1 
40K               100     1       7       1 
30K               150     1       6       1 
20K               225     1       6       1 

more baudrate parameters and detail please download:
Ginkgo USB-CAN Interface STM32 baud rate calculation software:
http://www.viewtool.com/demo/download/Ginkgo2/Ginkgo_USB-CAN_Baudrate_Windows.rar
"""
class VCI_INIT_CONFIG_EX(Structure):  
    _fields_ = [("CAN_BRP", c_uint),# range: 1~1024, 
        ("CAN_SJW", c_byte),    # range: 1~4  
        ("CAN_BS1", c_byte),    # range: 1~16      
        ("CAN_BS2",c_byte),     # range: 1~8
        ("CAN_Mode",c_byte),    # CAN working mode. 0: normal,1: loopback,2: silent,3: silent loopback
        ("CAN_ABOM",c_byte),    # auto off line management. 0: prohibit,1: enable
        ("CAN_NART",c_byte),    # text repeat send management. 0: enable text repeat sending,1: disable text repeat sending
        ("CAN_RFLM",c_byte),    # FIFO lock management. 0: new text overwrite old, 1: ignore new text
        ("CAN_TXFP",c_byte),    # send priority management, 0: by ID, 1: by order
        ("CAN_RELAY",c_byte),    # relay feature enable. 0x00: close relay function,0x10: relay from CAN1 to CAN2,0x01: relay from CAN2 to CAN1, 0x11: bidirectionaly relay
        ("Reserved",c_uint)    # reserved
        ];
# definition of CAN filter setting

class VCI_FILTER_CONFIG(Structure):  
    _fields_ = [("Enable", c_byte),            # filter enable. 1: enable,0: disable
        ("FilterIndex", c_byte),    # filter index. range: 0~13
        ("FilterMode", c_byte),            # filter mode.0: mask bit,1: id list
        ("ExtFrame",c_byte),        # filter frame flag. 1: the frame to be filtered is extended frame,0: the frame to be filtered is standard frame
        ("ID_Std_Ext",c_uint),        # verification code ID
        ("ID_IDE",c_uint),        # verification code IDE
        ("ID_RTR",c_uint),        # verification code RTR
        ("MASK_Std_Ext",c_uint),    # Mask code ID,only available when filter mode set to mask bit mode
        ("MASK_IDE",c_uint),        # Mask code IDE,only available when filter mode set to mask bit mode
        ("MASK_RTR",c_uint),        # Mask code RTR,only available when filter mode set to mask bit mode
        ("Reserved",c_uint),        # reserved
        ];

# definition of CAN receive callback
if(platform.system()=="Windows"):
    PVCI_RECEIVE_CALLBACK = WINFUNCTYPE(c_void_p,c_uint,c_uint,c_uint)
else:
    PVCI_RECEIVE_CALLBACK = CFUNCTYPE(c_void_p,c_uint,c_uint,c_uint)

# CAN type definition
VCI_USBCAN1    =    3
VCI_USBCAN2    =    4

#working mode
WORKING_MODE_NORMAL             = 0
WORKING_MODE_LOOP_BACK          = 1
WORKING_MODE_SILENT             = 2
WORKING_MODE_SILENT_LOOP_BACK   = 3

# receiving mode
RX_MODE_NORMAL      = 0
RX_MODE_CALLBACK    = 1

# CAN error code
ERR_CAN_OVERFLOW        =    0x0001    # CAN controller internal FIFO overflow
ERR_CAN_ERRALARM        =    0x0002    # CAN controller error alarm
ERR_CAN_PASSIVE            =    0x0004    # CAN controller passive error
ERR_CAN_LOSE            =    0x0008    # CAN controller arbitration lost
ERR_CAN_BUSERR            =    0x0010    # CAN arbitration bus error
ERR_CAN_BUSOFF            =    0x0020    # CAN arbitration bus off

# Generic error code
ERR_DEVICEOPENED        =    0x0100    # Device is opened
ERR_DEVICEOPEN            =    0x0200    # Device open failed
ERR_DEVICENOTOPEN        =    0x0400    # Device not open
ERR_BUFFEROVERFLOW        =    0x0800    # Buffer overflow
ERR_DEVICENOTEXIST        =    0x1000    # Device is not exist
ERR_LOADKERNELDLL        =    0x2000    # Load dll failed
ERR_CMDFAILED            =    0x4000    # Command execution failed
ERR_BUFFERCREATE        =    0x8000    # Memory is not enough

# Function return status
# STATUS_OK                =    0x01
# STATUS_ERR                =    0x00

_MODULE_DIR = os.path.dirname(os.path.abspath(__file__))
_MACHINE_MAP = {
    0x014C: "x86",
    0x8664: "x64",
    0xAA64: "arm64",
}
_GINKGO_DLL_DIR_HANDLE = None


def _pe_machine_type(path):
    try:
        with open(path, "rb") as f:
            if f.read(2) != b"MZ":
                return None
            f.seek(0x3C)
            pe_offset = struct.unpack("<I", f.read(4))[0]
            f.seek(pe_offset)
            if f.read(4) != b"PE\x00\x00":
                return None
            return struct.unpack("<H", f.read(2))[0]
    except (IOError, OSError, struct.error):
        return None


def _machine_name(machine_type):
    if machine_type is None:
        return "unknown"
    return _MACHINE_MAP.get(machine_type, "0x{0:04X}".format(machine_type))


def _load_windows_library():
    global _GINKGO_DLL_DIR_HANDLE

    python_bits = 64 if sizeof(c_void_p) == 8 else 32
    bit_dir = "64bit" if python_bits == 64 else "32bit"
    dll_dir = os.path.join(_MODULE_DIR, "lib", "windows", bit_dir)
    dll_path = os.path.join(dll_dir, "Ginkgo_Driver.dll")
    libusb_path = os.path.join(dll_dir, "libusb0.dll")

    if hasattr(os, "add_dll_directory"):
        # Keep the handle alive so dependency resolution stays active.
        _GINKGO_DLL_DIR_HANDLE = os.add_dll_directory(dll_dir)

    try:
        return windll.LoadLibrary(dll_path)
    except OSError as err:
        if getattr(err, "winerror", None) == 193:
            msg = (
                "Cannot load {0}. Python is {1}-bit, Ginkgo_Driver.dll is {2}, "
                "libusb0.dll is {3}. WinError 193 means there is a 32/64-bit mismatch.\n"
                "Directory checked: {4}\n"
                "Use matching binaries (x64 Python + x64 DLLs, or x86 Python + x86 DLLs)."
            ).format(
                dll_path,
                python_bits,
                _machine_name(_pe_machine_type(dll_path)),
                _machine_name(_pe_machine_type(libusb_path)) if os.path.exists(libusb_path) else "missing",
                dll_dir,
            )
            raise OSError(msg) from err
        raise


def _preload_library_if_exists(path):
    if not os.path.exists(path):
        return
    if hasattr(os, "RTLD_GLOBAL"):
        CDLL(path, mode=os.RTLD_GLOBAL)
    else:
        CDLL(path)


def _load_linux_library():
    bit_dir = "64bit" if sizeof(c_void_p) == 8 else "32bit"
    lib_dir = os.path.join(_MODULE_DIR, "lib", "linux", bit_dir)
    _preload_library_if_exists(os.path.join(lib_dir, "libusb-1.0.so"))
    _preload_library_if_exists(os.path.join(lib_dir, "libusb.so"))
    return cdll.LoadLibrary(os.path.join(lib_dir, "libGinkgo_Driver.so"))


def _load_macos_library():
    lib_dir = os.path.join(_MODULE_DIR, "lib", "macos")
    _preload_library_if_exists(os.path.join(lib_dir, "libusb-1.0.0.dylib"))
    _preload_library_if_exists(os.path.join(lib_dir, "libusb-0.1.4.dylib"))
    return cdll.LoadLibrary(os.path.join(lib_dir, "libGinkgo_Driver.dylib"))


# Import library
if platform.system() == "Windows":
    GinkgoLib = _load_windows_library()
elif platform.system() == "Darwin":
    GinkgoLib = _load_macos_library()
elif platform.system() == "Linux":
    GinkgoLib = _load_linux_library()
else:
    raise OSError("Unknown system: {0}".format(platform.system()))

# Scan device
def VCI_ScanDevice(NeedInit=1):
    return GinkgoLib.VCI_ScanDevice(c_ubyte(NeedInit))

# Open device
def VCI_OpenDevice(DevType, DevIndex, Reserved):
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_OpenDevice(c_uint(DevType), c_uint(DevIndex), c_uint(Reserved))

# Close device
def VCI_CloseDevice( DevType, DevIndex):
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_CloseDevice( c_uint(DevType), c_uint(DevIndex))
# Initialize device
def VCI_InitCAN(DevType, DevIndex, CANIndex, pInitConfig):
    'Initialize device'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_InitCAN(c_uint(DevType), c_uint(DevIndex), c_uint(CANIndex), pInitConfig)
# Initialize device extend
def VCI_InitCANEx(DevType, DevIndex, CANIndex, pInitConfig):
    'Initialize device extend'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_InitCANEx(c_uint(DevType), c_uint(DevIndex), c_uint(CANIndex), pInitConfig)
# Get board infomation
def VCI_ReadBoardInfoEx(DevIndex, pInfo):
    'Get board infomation'
    return GinkgoLib.VCI_ReadBoardInfoEx(c_uint(DevIndex), pInfo)
# Get CAN status
def VCI_ReadCANStatus(DevType,DevIndex,CANIndex,pCANStatus):
    'Get CAN status'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_ReadCANStatus(c_uint(DevType),c_uint(DevIndex),c_uint(CANIndex),pCANStatus)
# Set CAN filter
def VCI_SetFilter(DevType,DevIndex,CANIndex,pFilter):
    'Set CAN filter'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_SetFilter(c_uint(DevType),c_uint(DevIndex),c_uint(CANIndex),pFilter)
# Get CAN number from buffer
def VCI_GetReceiveNum(DevType,DevIndex,CANIndex):
    'Get CAN number from buffer'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_GetReceiveNum(c_uint(DevType),c_uint(DevIndex),c_uint(CANIndex))
# Clear CAN buffer
def VCI_ClearBuffer(DevType,DevIndex,CANIndex):
    'Clear CAN buffer'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_ClearBuffer(c_uint(DevType),c_uint(DevIndex),c_uint(CANIndex))
# Register receive callback function
def VCI_RegisterReceiveCallback(DevIndex, pReceiveCallBack):
    'Register receive callback function'
    return GinkgoLib.VCI_RegisterReceiveCallback(c_uint(DevIndex),pReceiveCallBack)
# Logout receive callback function
def VCI_LogoutReceiveCallback(DevIndex):
    'Logout receive callback function'
    return GinkgoLib.VCI_LogoutReceiveCallback(c_uint(DevIndex))
# Start receive CAN
def VCI_StartCAN(DevType,DevIndex,CANIndex):
    'Start receive CAN'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_StartCAN(c_uint(DevType),c_uint(DevIndex),c_uint(CANIndex))
# Stop and reset CAN
def VCI_ResetCAN(DevType, DevIndex, CANIndex):
    'Reset CAN'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_ResetCAN(c_uint(DevType),c_uint(DevIndex),c_uint(CANIndex))
# Transmit CAN data
def VCI_Transmit(DevType, DevIndex, CANIndex, pSend, Len):
    'Transmit CAN data'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_Transmit(c_uint(DevType),c_uint(DevIndex),c_uint(CANIndex),pSend,c_uint(Len))
	
# Read CAN data from buffer
def VCI_Receive(DevType,DevIndex,CANIndex,pReceive,Len,WaitTime):
    'Read CAN data from buffer'
    try:
        assert DevType == VCI_USBCAN2, 'Device type error!'
        assert 0 <= CANIndex <= 1, 'CAN index error!'
    except AssertionError as args:
        print('%s: %s'%(args.__class__.__name__, args))
        exit()
    return GinkgoLib.VCI_Receive(c_uint(DevType),c_uint(DevIndex),c_uint(CANIndex),pReceive,c_uint(Len),c_uint(WaitTime))




