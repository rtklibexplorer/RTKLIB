#!/bin/sh
# A0A20001 – Start sequence and payload length (1 byte)
# 94 – Payload
# 0094B0B3
/bin/echo -e "\xA0\xA2\x00\x01\x94\x00\x94\xB0\xB3 und normaler Text"  >/dev/ttyUSB0
sleep 3
cat GSD4e_4.1.2_P1_RPATCH_10.pd2 > /dev/ttyUSB0




