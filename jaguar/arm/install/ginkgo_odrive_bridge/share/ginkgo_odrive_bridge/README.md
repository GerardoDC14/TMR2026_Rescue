# Ginkgo USB-CAN Python Tools

Cleaned and organized scripts for the Ginkgo USB-CAN adapter, with support for Windows and Linux.

## What was cleaned

- Added shared CAN helpers in `Python_USB_CAN_Test_64bits/ginkgo_can/`.
- Refactored active scripts to use one code path for open/config/send/receive.
- Moved old vendor-style test harness files into `Python_USB_CAN_Test_64bits/legacy/`.
- Added root `requirements.txt` and `.gitignore`.
- Improved native library loading in `ControlCAN.py` for Linux/macOS dependency preloading.

## Project layout

```text
Python_USB_CAN_Test_64bits/
  ControlCAN.py                # ctypes binding + native library loader
  ErrorType.py                 # vendor error codes
  ginkgo_can/
    bus.py                     # shared bus/session utilities
    odrive.py                  # ODrive and servo payload helpers
  can_sniffer.py               # bus sniffer
  can_get_pos.py               # read ODrive encoder estimate
  can_send_pos.py              # send ODrive Set_Input_Pos
  send_Read_pos.py             # move, wait for target, then return to zero
  sendcan_odrive_servos.py     # read ODrive + send one servo command
  sencan_odrive_pca.py         # send ODrive + two servo commands
  legacy/                      # old scripts and logs kept for reference
  lib/                         # vendor native libraries
tools/
  windows/
    zadig-2.9_mod.exe          # bundled Zadig installer used for driver setup
ginkgo_odrive_bridge/
  joint_state_bridge.py        # ROS 2 node: /joint_states -> ODrive CAN
config/
  joint_state_bridge.yaml      # default ROS 2 parameters
launch/
  joint_state_bridge.launch.py # ROS 2 launch entrypoint
package.xml                    # ROS 2 package manifest
setup.py                       # ROS 2 Python package installer
```

## Requirements

- Python 3.10+
- Ginkgo USB-CAN adapter
- Matching driver/native binaries for your OS/architecture

Python package dependencies: none (standard library only).

ROS 2 package dependencies:

- `rclpy`
- `sensor_msgs`
- `ament_index_python`
- `launch_ros`

## OS notes

### Windows

- Use a Python architecture that matches your DLLs (`x64` Python with `lib/windows/64bit` DLLs, or `x86` with `32bit`).
- Install the USB driver for the adapter before running scripts.
- Bundled installer: `tools/windows/zadig-2.9_mod.exe`

#### Windows driver setup (what this project uses)

1. Run `tools/windows/zadig-2.9_mod.exe` as Administrator.
2. In Zadig: `Options -> List All Devices`.
3. Select the Ginkgo USB-CAN device.
4. Choose driver: `libusb-win32`.
5. Click `Install Driver` (or `Replace Driver`).
6. Replug the adapter and run a script (for example `python can_sniffer.py`).

### Linux

- Scripts require USB access. Run with `sudo` or configure udev permissions.
- `ControlCAN.py` now preloads local `libusb` libraries from `lib/linux/<32bit|64bit>/`.

#### Recommended Linux setup for ROS 2

Do not run the ROS 2 bridge with `sudo` unless you absolutely have to.
On Linux, running the bridge as `root` while the rest of the ROS graph runs as
your normal user can result in topic discovery succeeding while message data is
not delivered.

Install the provided udev rule for the Ginkgo adapter instead:

```bash
sudo cp udev/99-ginkgo-usb-can.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG plugdev $USER
```

Then unplug/replug the Ginkgo adapter and log out/in once.

After that, run the bridge as your normal user:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ginkgo_odrive_bridge joint_state_bridge.launch.py
```

## Quick start

From repo root:

```bash
cd Python_USB_CAN_Test_64bits
python can_sniffer.py
```

Linux example:

```bash
cd Python_USB_CAN_Test_64bits
sudo python3 can_sniffer.py 0 500
```

## ROS 2 package

This repo now includes a ROS 2 Python package named `ginkgo_odrive_bridge`.
The bridge subscribes to `/joint_states` and sends native ODrive CAN frames
through the Ginkgo adapter.

Default behavior:

- `Joint1 -> node_id 16 (0x10)`
- `Joint2 -> node_id 17 (0x11)`
- The startup encoder estimate of every configured joint is treated as that
  joint's zero reference on the ODrive side.
- `/joint_states` values are used directly as the command input in radians.
- `/joint_states` uses `sensor_data` QoS by default to match common ROS 2
  publishers using best-effort delivery.

Build inside a ROS 2 workspace:

```bash
cd /path/to/ros2_ws
colcon build --packages-select ginkgo_odrive_bridge --symlink-install
source install/setup.bash
```

Run with the default parameter file:

```bash
ros2 launch ginkgo_odrive_bridge joint_state_bridge.launch.py
```

Run with verbose bridge logging:

```bash
ros2 launch ginkgo_odrive_bridge joint_state_bridge.launch.py verbose:=true verbose_period_s:=0.5
```

Run directly:

```bash
ros2 run ginkgo_odrive_bridge joint_state_bridge --ros-args --params-file \
  $(ros2 pkg prefix ginkgo_odrive_bridge)/share/ginkgo_odrive_bridge/config/joint_state_bridge.yaml
```

Adjust CAN and joint mapping in `config/joint_state_bridge.yaml`.
The array parameters must stay aligned:

- `joint_names`
- `node_ids`
- `gear_ratios`
- `directions`
- `use_startup_offsets`
- `command_modes`

Supported `command_modes`:

- `direct`: signed position command
- `abs`: positive absolute command, like `abs(turns)`
- `neg_abs`: negative absolute command, like `-abs(turns)`

## Script usage

```bash
python can_sniffer.py [channel] [kbps]
python can_get_pos.py [channel] [kbps] [node_id] [--timeout 1.0]
python can_send_pos.py [channel] [kbps] [node_id] [--target 5.0] [--hold 5.0] [--no-return]
python send_Read_pos.py [channel] [kbps] [node_id] [--target 5.0] [--threshold 0.01] [--timeout 10]
python sendcan_odrive_servos.py <channel> <kbps> <odrive_node> <servo_channel> <angle_deg>
python sencan_odrive_pca.py <channel> <kbps> <odrive_node> <servo0_ch> <servo0_angle> <servo1_ch> <servo1_angle>
```

## Notes

- CAN bitrates currently supported in scripts: `100, 125, 250, 500, 800, 1000` kbps.
- Legacy files were not deleted; they were moved to `legacy/` so nothing is lost.
- On Linux, ROS 2 runs still need USB permissions for the Ginkgo adapter. Use
  `sudo` or configure udev rules.

                                 ,,,,,,,,,,,,,,,,,,,,,
           M                  , '                     ',
         {|  M            , '                           ',
        { |    M      , '                                 ',
       {./       >,,'                             ;         ;,
 ======;;;;;    __>                               ;         ; ',
=====,'   @    (__                                ;         ;   ',
___ /         .../                                ;         ;
\,/                  ',         ',               ;         ;
 (  ^     , '''''',,,,,',         ',            ,;        ;',
  \//_, '         ;     ;',        ;,,,,,,,'''';  ;      ;   ',
                 ;     ;   ',      ;      ;    ;   ;     ;     ',
                ;    ;       ;     ;     ;    ;    ;    ;        '
               ;    ;        ;    ;     ;    ;     ;    ;
              ;    ;         ;    ;    (/(/(/      ;    ;
             (/(/(/          ;    ;                (/(/(/
                             (/(/(/
