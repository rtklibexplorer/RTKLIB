# $PSRF101,0,0,0,0,0,0,12,4*10
!WAIT 300
$PSRF100,0,115200,8,1,0*04
# Enable CW controller
#!HEX a0 a2 00 03 dc 01 00 00 dd b0 b3

# Initalize GPS/DR Navigation + Debug Output
!HEX a0 a2 00 1c ac 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0c 36 00 ef b0 b3
!WAIT 200

# Set Elevetaion mask 10 Degrees
!HEX a0 a2 00 05 8b 00 32 00 64 01 21 b0 b3
!WAIT 100

# Set Power Mask 32 dbHz
!HEX a0 a2 00 03 8c 1c 20 00 c8 b0 b3
!WAIT 100

# Disable DOP Mask
!HEX a0 a2 00 05 89 04 00 00 00 00 8d b0 b3

#; Disable SGEE, enable CGEE
#!HEX a0 a2 00 04 e8 20 01 00 01 09 b0 b3
#; Disable SGEE+CGEE
!HEX a0 a2 00 04 e8 20 01 01 01 0a b0 b3
!WAIT 100

#; Disable CGEE(+prediction) and SGEE at all
!HEX a0 a2 00 06 e8 fe ff ff ff ff 05 e2 b0 b3

#; Disable DGPS
!HEX a0 a2 00 07 85 00 00 00 00 00 00 00 85 b0 b3

# Full power mode ??
!HEX a0 a2 00 02 c8 00 00 c8 b0 b2

#; Enable navlib 
!HEX a0 a2 00 19 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 0c 30 00 bc b0 b3
!WAIT 100

#; Disable Messages #51 unknown, #91 HW Control Output & AGC; #92 CW Controller Output, MID 9+255
!HEX a0 a2 00 04 a6 00 33 00 00 00 00 00 00 d9 b0 b3
!WAIT 200
!HEX a0 a2 00 04 a6 00 5b 2e 00 00 00 00 01 2f b0 b3
!WAIT 200
!HEX a0 a2 00 04 a6 00 5c 00 00 00 00 00 01 02 b0 b3
!WAIT 200
!HEX a0 a2 00 08 a6 04 00 00 00 00 00 00 00 aa b0 b3

#; MID 166 Enable NAVLIB Messages 7, 28, 29, 30, 31 1Hz + 1 PPS + NAV Bit Aiding + MID 64
!HEX a0 a2 00 08 a6 00 05 00 01 00 00 00 00 ac b0 b3
!WAIT 100
!HEX a0 a2 00 08 a6 00 34 05 00 00 00 00 00 df b0 b3
!WAIT 100
!HEX a0 a2 00 x8 a6 00 20 01 00 00 00 00 00 c7 b0 b3
!WAIT 100
!HEX a0 a2 00 0a 49 04 ff ff 06 ff ff ff ff 03 06 50 b0 b3
!WAIT 100

#; 84 - Poll software version, 144 - clock, 147 - ephemeris, 146 - almanac
!HEX a0 a2 00 02 84 00 00 84 b0 b3
!WAIT 100
!HEX a0 a2 00 02 90 00 00 90 b0 b3
!WAIT 100
!HEX a0 a2 00 03 93 00 00 00 93 b0 b3
!WAIT 100
!HEX a0 a2 00 02 92 00 00 92 b0 b3
!WAIT 100

#;  MID 136 Freeze Clock drift, Full Power, disable fast time sync, no altitude hold=all fixes 3D, disable DR, raw measurement, softw Tracking disable
# Enable fast time sync?
#!HEX a0 a2 00 0e 88 00 00 03 14 00 00 00 02 00 00 00 00 04 00 a5 b0 b3
#!HEX a0 a2 00 0e 88 00 00 04 04 00 00 00 02 00 00 00 00 04 00 96 b0 b3
!HEX a0 a2 00 0e 88 00 00 04 00 00 00 00 02 00 00 00 00 04 00 92 b0 b3

!WAIT 100

# Set OnTime  112233445566778899 1000 = 100% +200ms on
#!HEX a0a20009 97000000c8000000c8 0227b0b3 
# 5Hz ... naja
!HEX a0 a2 00 09 97 00 00 03 e8 00 00 00 c8 02 4a b0 b3 
!WAIT 100

# Disable DGPS / SBAS
!HEX a0 a2 00 03 8a 02 00 00 8c b0 b3
!WAIT 100

# Switch to RTCM
#!HEX a0a20002 8704 0091b0b3
# Time Accuracy Status Request
!HEX a0 a2 00 d4 0402 00 06 b0 b3

#; Static Navigaton
!HEX a0 a2 00 02 8f 01 00 90 b0 b3

#; Query Power Mode - should return A0 A2 00 02 5A 00 00 5A B0 B3 on Full Power Mode
#!HEX a0 a2 00 02 da 00 00 da b0 b3

#; Query Navigation Parameters
#!HEX a0 a2 00 02 98 00 00 98 b0 b3
