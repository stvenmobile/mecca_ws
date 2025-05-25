#!/bin/bash

# USB Device Diagnostic Script for Mecca Robot
# Checks USB devices, permissions, and stable symlinks

echo "=== Mecca Robot USB Device Diagnostic ==="
echo "Date: $(date)"
echo "User: ${USER}"
echo ""

# Check if user is in dialout group
echo "=== User Group Membership ==="
if groups "${USER}" | grep -q dialout; then
    echo "✅ User ${USER} is in dialout group"
else
    echo "❌ User ${USER} is NOT in dialout group"
    echo "   Run: sudo usermod -a -G dialout ${USER}"
fi
echo ""

# Check USB serial devices
echo "=== Available USB Serial Devices ==="
if ls /dev/ttyUSB* >/dev/null 2>&1; then
    ls -la /dev/ttyUSB*
else
    echo "❌ No /dev/ttyUSB* devices found"
fi

if ls /dev/ttyACM* >/dev/null 2>&1; then
    ls -la /dev/ttyACM*
else
    echo "No /dev/ttyACM* devices found"
fi
echo ""

# Check stable symlinks
echo "=== Stable Device Symlinks ==="
for device in stm32_serial lidar_serial stm32_alt lidar_alt; do
    if [ -e "/dev/${device}" ]; then
        echo "✅ /dev/${device} -> $(readlink /dev/${device})"
    else
        echo "❌ /dev/${device} not found"
    fi
done
echo ""

# Check udev rules
echo "=== udev Rules Status ==="
RULES_FILE="/etc/udev/rules.d/99-mecca-robot.rules"
if [ -f "${RULES_FILE}" ]; then
    echo "✅ udev rules file exists: ${RULES_FILE}"
    echo "   Modified: $(stat -c %y "${RULES_FILE}")"
else
    echo "❌ udev rules file not found: ${RULES_FILE}"
    echo "   Run the install_udev_rules.sh script"
fi
echo ""

# USB device details
echo "=== USB Device Details ==="
for device in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$device" ]; then
        echo "--- $device ---"
        udevadm info --name="$device" --attribute-walk | grep -E "(ATTRS{product}|ATTRS{manufacturer}|ATTRS{serial}|DRIVERS)" | head -4
        echo ""
    fi
done

# USB topology
echo "=== USB Hub Topology ==="
lsusb -t
echo ""

# Test device access
echo "=== Device Access Test ==="
for device in /dev/stm32_serial /dev/lidar_serial /dev/ttyUSB0 /dev/ttyUSB1; do
    if [ -e "$device" ]; then
        if [ -r "$device" ] && [ -w "$device" ]; then
            echo "✅ ${device} - Read/Write access OK"
        else
            echo "❌ ${device} - No read/write access"
        fi
    fi
done
echo ""

echo "=== Recommendations ==="
if ! groups "${USER}" | grep -q dialout; then
    echo "1. Add user to dialout group: sudo usermod -a -G dialout ${USER}"
fi

if [ ! -f "/etc/udev/rules.d/99-mecca-robot.rules" ]; then
    echo "2. Install udev rules: ./install_udev_rules.sh"
fi

if ! ls /dev/stm32_serial >/dev/null 2>&1 || ! ls /dev/lidar_serial >/dev/null 2>&1; then
    echo "3. Reboot system to apply udev rules: sudo reboot"
fi

echo ""
echo "For support, share this output when reporting issues."
