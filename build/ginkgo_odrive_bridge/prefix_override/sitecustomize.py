import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gerardo/TMR2026_Rescue/install/ginkgo_odrive_bridge'
