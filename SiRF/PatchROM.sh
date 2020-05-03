#!/bin/sh
# A0A20001 – Start sequence and payload length (1 byte)
# 94 – Payload
# 0094B0B3
/bin/echo -ne "\xA0\xA2\x00\x01\x94\x00\x94\xB0\xB3"  >/dev/ttyUSB0
sleep 3
cat GSD4e_4.1.2_P1_RPATCH_10.pd2 > /dev/ttyUSB0
sleep 2
/bin/echo -ne "\xA0\xA2\x00\x02\x84\x00\x00\x84\xb0\xb3" >/dev/ttyUSB0




