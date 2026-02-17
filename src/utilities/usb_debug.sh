#!/bin/bash

echo "=== USB Device Debug Information ==="
echo "Date: $(date)"
echo

echo "=== Available USB Serial Devices ==="
ls -la /dev/ttyUSB* 2>/dev/null || echo "No /dev/ttyUSB* devices found"
ls -la /dev/ttyACM* 2>/dev/null || echo "No /dev/ttyACM* devices found"
echo

echo "=== Device Permissions ==="
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
echo

echo "=== USB Device Details ==="
for device in /dev/ttyUSB* /dev/ttyACM*; do
    if [ -e "$device" ]; then
        echo "--- $device ---"
        udevadm info --name="$device" --attribute-walk | grep -E "(ATTRS{product}|ATTRS{manufacturer}|ATTRS{serial})" | head -3
        echo
    fi
done

echo "=== USB Hub Power Status ==="
lsusb -t
echo

echo "=== dmesg USB Messages (last 20 lines) ==="
dmesg | grep -i usb | tail -20
