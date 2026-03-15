#!/bin/bash

# SSD 디바이스 이름 찾기
SSD_DEVICE=$(lsblk -ndo NAME,TYPE | grep -w "disk" | grep -E "sd|nvme" | head -n 1 | awk '{print "/dev/" $1}')

# 디바이스 이름이 존재하는지 확인
if [ -z "$SSD_DEVICE" ]; then
    echo "No SSD device found"
    exit 1
fi

# 시리얼 넘버 추출
SERIAL=$(udevadm info --query=all --name=$SSD_DEVICE | grep "ID_SERIAL_SHORT" | awk -F= '{print $2}')

# 시리얼 넘버가 존재하는지 확인
if [ -z "$SERIAL" ]; then
    echo "No serial number found for device $SSD_DEVICE"
    exit 1
fi

# 시리얼 넘버 출력
echo "$SERIAL"