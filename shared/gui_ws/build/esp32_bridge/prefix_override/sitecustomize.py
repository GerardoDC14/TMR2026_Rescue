import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arepo/TMR2026_Rescue/shared/gui_ws/install/esp32_bridge'
