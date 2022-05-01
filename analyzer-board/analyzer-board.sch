EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "TeensyLogicAnalyzer main board"
Date "2022-04-30"
Rev "1.0"
Comp "Copyright (c) 2022 Jason R. Thorpe.  See LICENSE."
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L jrt-Arduino:Teensy4_1_GPIO U1
U 1 1 6265D8F8
P 1850 3300
F 0 "U1" H 1875 5565 50  0000 C CNN
F 1 "Teensy4_1_GPIO" H 1875 5474 50  0000 C CNN
F 2 "Package_DIP:DIP-48_W15.24mm_Socket" H 1900 3200 50  0001 C CNN
F 3 "https://www.pjrc.com/store/teensy41.html" H 1850 4250 50  0001 C CNN
	1    1850 3300
	1    0    0    -1  
$EndComp
NoConn ~ 1200 1700
$Comp
L power:GND #PWR0101
U 1 1 62662BAA
P 1200 2700
F 0 "#PWR0101" H 1200 2450 50  0001 C CNN
F 1 "GND" H 1205 2527 50  0000 C CNN
F 2 "" H 1200 2700 50  0001 C CNN
F 3 "" H 1200 2700 50  0001 C CNN
	1    1200 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2500 1200 2600
Wire Wire Line
	1200 2600 1200 2700
Connection ~ 1200 2600
Connection ~ 1200 2700
Text GLabel 2550 1300 2    50   Input ~ 0
CA0
Text GLabel 2550 1400 2    50   Input ~ 0
CA1
Text GLabel 2550 2700 2    50   Input ~ 0
CA2
Text GLabel 2550 2800 2    50   Input ~ 0
CA3
Text GLabel 2550 2900 2    50   Input ~ 0
CA4
Text GLabel 2550 3000 2    50   Input ~ 0
CA5
Text GLabel 2550 3100 2    50   Input ~ 0
CA6
Text GLabel 2550 3200 2    50   Input ~ 0
CA7
Text GLabel 2550 3300 2    50   Input ~ 0
CA8
Text GLabel 2550 3400 2    50   Input ~ 0
CA9
Text GLabel 2550 3500 2    50   Input ~ 0
CA10
Text GLabel 2550 3600 2    50   Input ~ 0
CA11
Text GLabel 2550 3700 2    50   Input ~ 0
CA12
Text GLabel 2550 3800 2    50   Input ~ 0
CA13
Text GLabel 2550 3900 2    50   Input ~ 0
CA14
Text GLabel 2550 4000 2    50   Input ~ 0
CA15
Text GLabel 2550 1900 2    50   Input ~ 0
CD0
Text GLabel 2550 2000 2    50   Input ~ 0
CD1
Text GLabel 2550 2100 2    50   Input ~ 0
CD2
Text GLabel 2550 2200 2    50   Input ~ 0
CD3
Text GLabel 2550 2300 2    50   Input ~ 0
CD4
Text GLabel 2550 2400 2    50   Input ~ 0
CD5
Text GLabel 2550 2500 2    50   Input ~ 0
CD6
Text GLabel 2550 4500 2    50   Input ~ 0
CD7
Text GLabel 2550 1500 2    50   Input ~ 0
CC0
Text GLabel 2550 1600 2    50   Input ~ 0
CC1
Text GLabel 2550 1700 2    50   Input ~ 0
CC2
Text GLabel 2550 1800 2    50   Input ~ 0
CC5
Text GLabel 2550 4200 2    50   Input ~ 0
CC4
Text GLabel 2550 4600 2    50   Input ~ 0
CC5
Text GLabel 2550 5100 2    50   Input ~ 0
CC6
Text GLabel 2550 4700 2    50   Input ~ 0
CC7
Text GLabel 2550 4800 2    50   Input ~ 0
CC8
Text GLabel 2550 4900 2    50   Input ~ 0
CC9
Text GLabel 2550 5000 2    50   Input ~ 0
CC10
Text GLabel 2550 5200 2    50   Input ~ 0
CC11
Text GLabel 2550 5300 2    50   Input ~ 0
CC12
Text GLabel 2550 5400 2    50   Input ~ 0
CC13
$Comp
L power:+3V3 #PWR0102
U 1 1 626687EF
P 1200 2150
F 0 "#PWR0102" H 1200 2000 50  0001 C CNN
F 1 "+3V3" V 1215 2278 50  0000 L CNN
F 2 "" H 1200 2150 50  0001 C CNN
F 3 "" H 1200 2150 50  0001 C CNN
	1    1200 2150
	0    -1   -1   0   
$EndComp
NoConn ~ 2550 2600
$Comp
L 74xx:74LS245 U2
U 1 1 6266A207
P 3800 2000
F 0 "U2" H 4050 2800 50  0000 C CNN
F 1 "74LVC245" H 4050 2700 50  0000 C CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 3800 2000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 3800 2000 50  0001 C CNN
	1    3800 2000
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0103
U 1 1 6266BBB1
P 3800 1200
F 0 "#PWR0103" H 3800 1050 50  0001 C CNN
F 1 "+3V3" H 3815 1373 50  0000 C CNN
F 2 "" H 3800 1200 50  0001 C CNN
F 3 "" H 3800 1200 50  0001 C CNN
	1    3800 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 6266BE5C
P 3800 2800
F 0 "#PWR0104" H 3800 2550 50  0001 C CNN
F 1 "GND" H 3805 2627 50  0000 C CNN
F 2 "" H 3800 2800 50  0001 C CNN
F 3 "" H 3800 2800 50  0001 C CNN
	1    3800 2800
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 6266C969
P 2950 3950
F 0 "R1" H 3020 3996 50  0000 L CNN
F 1 "3K3" H 3020 3905 50  0000 L CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P7.62mm_Horizontal" V 2880 3950 50  0001 C CNN
F 3 "~" H 2950 3950 50  0001 C CNN
	1    2950 3950
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0105
U 1 1 6266D896
P 2950 3800
F 0 "#PWR0105" H 2950 3650 50  0001 C CNN
F 1 "+3V3" H 2965 3973 50  0000 C CNN
F 2 "" H 2950 3800 50  0001 C CNN
F 3 "" H 2950 3800 50  0001 C CNN
	1    2950 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 4100 2950 4100
$Comp
L 74xx:74LS245 U3
U 1 1 62672896
P 5400 2000
F 0 "U3" H 5650 2800 50  0000 C CNN
F 1 "74LVC245" H 5650 2700 50  0000 C CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 5400 2000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 5400 2000 50  0001 C CNN
	1    5400 2000
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0106
U 1 1 6267289C
P 5400 1200
F 0 "#PWR0106" H 5400 1050 50  0001 C CNN
F 1 "+3V3" H 5415 1373 50  0000 C CNN
F 2 "" H 5400 1200 50  0001 C CNN
F 3 "" H 5400 1200 50  0001 C CNN
	1    5400 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 626728A2
P 5400 2800
F 0 "#PWR0107" H 5400 2550 50  0001 C CNN
F 1 "GND" H 5405 2627 50  0000 C CNN
F 2 "" H 5400 2800 50  0001 C CNN
F 3 "" H 5400 2800 50  0001 C CNN
	1    5400 2800
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS245 U4
U 1 1 62673D6B
P 7100 2000
F 0 "U4" H 7350 2800 50  0000 C CNN
F 1 "74LVC245" H 7350 2700 50  0000 C CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 7100 2000 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 7100 2000 50  0001 C CNN
	1    7100 2000
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0108
U 1 1 62673D71
P 7100 1200
F 0 "#PWR0108" H 7100 1050 50  0001 C CNN
F 1 "+3V3" H 7115 1373 50  0000 C CNN
F 2 "" H 7100 1200 50  0001 C CNN
F 3 "" H 7100 1200 50  0001 C CNN
	1    7100 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 62673D77
P 7100 2800
F 0 "#PWR0109" H 7100 2550 50  0001 C CNN
F 1 "GND" H 7105 2627 50  0000 C CNN
F 2 "" H 7100 2800 50  0001 C CNN
F 3 "" H 7100 2800 50  0001 C CNN
	1    7100 2800
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS245 U5
U 1 1 6267ADC1
P 4200 4100
F 0 "U5" H 4450 4900 50  0000 C CNN
F 1 "74LVC245" H 4450 4800 50  0000 C CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 4200 4100 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 4200 4100 50  0001 C CNN
	1    4200 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0110
U 1 1 6267ADC7
P 4200 3300
F 0 "#PWR0110" H 4200 3150 50  0001 C CNN
F 1 "+3V3" H 4215 3473 50  0000 C CNN
F 2 "" H 4200 3300 50  0001 C CNN
F 3 "" H 4200 3300 50  0001 C CNN
	1    4200 3300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0111
U 1 1 6267ADCD
P 4200 4900
F 0 "#PWR0111" H 4200 4650 50  0001 C CNN
F 1 "GND" H 4205 4727 50  0000 C CNN
F 2 "" H 4200 4900 50  0001 C CNN
F 3 "" H 4200 4900 50  0001 C CNN
	1    4200 4900
	1    0    0    -1  
$EndComp
$Comp
L 74xx:74LS245 U6
U 1 1 6267ADD7
P 5800 4100
F 0 "U6" H 6050 4900 50  0000 C CNN
F 1 "74LVC245" H 6050 4800 50  0000 C CNN
F 2 "Package_DIP:DIP-20_W7.62mm_Socket" H 5800 4100 50  0001 C CNN
F 3 "http://www.ti.com/lit/gpn/sn74LS245" H 5800 4100 50  0001 C CNN
	1    5800 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR0112
U 1 1 6267ADDD
P 5800 3300
F 0 "#PWR0112" H 5800 3150 50  0001 C CNN
F 1 "+3V3" H 5815 3473 50  0000 C CNN
F 2 "" H 5800 3300 50  0001 C CNN
F 3 "" H 5800 3300 50  0001 C CNN
	1    5800 3300
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0113
U 1 1 6267ADE3
P 5800 4900
F 0 "#PWR0113" H 5800 4650 50  0001 C CNN
F 1 "GND" H 5805 4727 50  0000 C CNN
F 2 "" H 5800 4900 50  0001 C CNN
F 3 "" H 5800 4900 50  0001 C CNN
	1    5800 4900
	1    0    0    -1  
$EndComp
Text GLabel 3300 1500 0    50   Output ~ 0
CA0
Text GLabel 3300 1600 0    50   Output ~ 0
CA1
Text GLabel 3300 1700 0    50   Output ~ 0
CA2
Text GLabel 3300 1800 0    50   Output ~ 0
CA3
Text GLabel 3300 1900 0    50   Output ~ 0
CA4
Text GLabel 3300 2000 0    50   Output ~ 0
CA5
Text GLabel 3300 2100 0    50   Output ~ 0
CA6
Text GLabel 3300 2200 0    50   Output ~ 0
CA7
Text GLabel 4900 1500 0    50   Output ~ 0
CA8
Text GLabel 4900 1600 0    50   Output ~ 0
CA9
Text GLabel 4900 1700 0    50   Output ~ 0
CA10
Text GLabel 4900 1800 0    50   Output ~ 0
CA11
Text GLabel 4900 1900 0    50   Output ~ 0
CA12
Text GLabel 4900 2000 0    50   Output ~ 0
CA13
Text GLabel 4900 2100 0    50   Output ~ 0
CA14
Text GLabel 4900 2200 0    50   Output ~ 0
CA15
Text GLabel 6600 1500 0    50   Output ~ 0
CD0
Text GLabel 6600 1600 0    50   Output ~ 0
CD1
Text GLabel 6600 1700 0    50   Output ~ 0
CD2
Text GLabel 6600 1800 0    50   Output ~ 0
CD3
Text GLabel 6600 1900 0    50   Output ~ 0
CD4
Text GLabel 6600 2000 0    50   Output ~ 0
CD5
Text GLabel 6600 2100 0    50   Output ~ 0
CD6
Text GLabel 6600 2200 0    50   Output ~ 0
CD7
Text GLabel 3700 3600 0    50   Output ~ 0
CC0
Text GLabel 3700 3700 0    50   Output ~ 0
CC1
Text GLabel 3700 3800 0    50   Output ~ 0
CC2
Text GLabel 3700 3900 0    50   Output ~ 0
CC3
Text GLabel 3700 4000 0    50   Output ~ 0
CC4
Text GLabel 3700 4100 0    50   Output ~ 0
CC5
Text GLabel 3700 4200 0    50   Output ~ 0
CC6
Text GLabel 3700 4300 0    50   Output ~ 0
CC7
Text GLabel 5300 3600 0    50   Output ~ 0
CC8
Text GLabel 5300 3700 0    50   Output ~ 0
CC9
Text GLabel 5300 3800 0    50   Output ~ 0
CC10
Text GLabel 5300 3900 0    50   Output ~ 0
CC11
Text GLabel 5300 4000 0    50   Output ~ 0
CC12
Text GLabel 5300 4100 0    50   Output ~ 0
CC13
NoConn ~ 5300 4200
NoConn ~ 5300 4300
NoConn ~ 6300 4300
NoConn ~ 6300 4200
Text GLabel 4300 1500 2    50   Input ~ 0
A0
Text GLabel 4300 1600 2    50   Input ~ 0
A1
Text GLabel 4300 1700 2    50   Input ~ 0
A2
Text GLabel 4300 1800 2    50   Input ~ 0
A3
Text GLabel 4300 1900 2    50   Input ~ 0
A4
Text GLabel 4300 2000 2    50   Input ~ 0
A5
Text GLabel 4300 2100 2    50   Input ~ 0
A6
Text GLabel 4300 2200 2    50   Input ~ 0
A7
Text GLabel 5900 1500 2    50   Input ~ 0
A8
Text GLabel 5900 1600 2    50   Input ~ 0
A9
Text GLabel 5900 2100 2    50   Input ~ 0
A14
Text GLabel 5900 1700 2    50   Input ~ 0
A10
Text GLabel 5900 1800 2    50   Input ~ 0
A11
Text GLabel 5900 1900 2    50   Input ~ 0
A12
Text GLabel 5900 2000 2    50   Input ~ 0
A13
Text GLabel 5900 2200 2    50   Input ~ 0
A15
Text GLabel 7600 1500 2    50   Input ~ 0
D0
Text GLabel 7600 1600 2    50   Input ~ 0
D1
Text GLabel 7600 1700 2    50   Input ~ 0
D2
Text GLabel 7600 1800 2    50   Input ~ 0
D3
Text GLabel 7600 1900 2    50   Input ~ 0
D4
Text GLabel 7600 2000 2    50   Input ~ 0
D5
Text GLabel 7600 2100 2    50   Input ~ 0
D6
Text GLabel 7600 2200 2    50   Input ~ 0
D7
Text GLabel 4700 3600 2    50   Input ~ 0
C0
Text GLabel 4700 3700 2    50   Input ~ 0
C1
Text GLabel 4700 3800 2    50   Input ~ 0
C2
Text GLabel 4700 3900 2    50   Input ~ 0
C3
Text GLabel 4700 4000 2    50   Input ~ 0
C4
Text GLabel 4700 4100 2    50   Input ~ 0
C5
Text GLabel 4700 4200 2    50   Input ~ 0
C6
Text GLabel 4700 4300 2    50   Input ~ 0
C7
Text GLabel 6300 3600 2    50   Input ~ 0
C8
Text GLabel 6300 3700 2    50   Input ~ 0
C9
Text GLabel 6300 3800 2    50   Input ~ 0
C10
Text GLabel 6300 3900 2    50   Input ~ 0
C11
Text GLabel 6300 4000 2    50   Input ~ 0
C12
Text GLabel 6300 4100 2    50   Input ~ 0
C13
$Comp
L Device:C C1
U 1 1 6269B782
P 1100 6350
F 0 "C1" H 1215 6396 50  0000 L CNN
F 1 "0.1µF" H 1215 6305 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 1138 6200 50  0001 C CNN
F 3 "~" H 1100 6350 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 1100 6350 50  0001 C CNN "Mouser"
	1    1100 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C2
U 1 1 626B9713
P 1550 6350
F 0 "C2" H 1665 6396 50  0000 L CNN
F 1 "0.1µF" H 1665 6305 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 1588 6200 50  0001 C CNN
F 3 "~" H 1550 6350 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 1550 6350 50  0001 C CNN "Mouser"
	1    1550 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C3
U 1 1 626B989A
P 2000 6350
F 0 "C3" H 2115 6396 50  0000 L CNN
F 1 "0.1µF" H 2115 6305 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 2038 6200 50  0001 C CNN
F 3 "~" H 2000 6350 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 2000 6350 50  0001 C CNN "Mouser"
	1    2000 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C4
U 1 1 626B99E5
P 2450 6350
F 0 "C4" H 2565 6396 50  0000 L CNN
F 1 "0.1µF" H 2565 6305 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 2488 6200 50  0001 C CNN
F 3 "~" H 2450 6350 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 2450 6350 50  0001 C CNN "Mouser"
	1    2450 6350
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 626B9B30
P 2900 6350
F 0 "C5" H 3015 6396 50  0000 L CNN
F 1 "0.1µF" H 3015 6305 50  0000 L CNN
F 2 "Capacitor_THT:C_Radial_D6.3mm_H5.0mm_P2.50mm" H 2938 6200 50  0001 C CNN
F 3 "~" H 2900 6350 50  0001 C CNN
F 4 "80-C320C104K2R5TA" H 2900 6350 50  0001 C CNN "Mouser"
	1    2900 6350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0121
U 1 1 626BA442
P 2900 6500
F 0 "#PWR0121" H 2900 6250 50  0001 C CNN
F 1 "GND" H 2905 6327 50  0000 C CNN
F 2 "" H 2900 6500 50  0001 C CNN
F 3 "" H 2900 6500 50  0001 C CNN
	1    2900 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 626BA4AB
P 2450 6500
F 0 "#PWR0122" H 2450 6250 50  0001 C CNN
F 1 "GND" H 2455 6327 50  0000 C CNN
F 2 "" H 2450 6500 50  0001 C CNN
F 3 "" H 2450 6500 50  0001 C CNN
	1    2450 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0123
U 1 1 626BA5AC
P 2000 6500
F 0 "#PWR0123" H 2000 6250 50  0001 C CNN
F 1 "GND" H 2005 6327 50  0000 C CNN
F 2 "" H 2000 6500 50  0001 C CNN
F 3 "" H 2000 6500 50  0001 C CNN
	1    2000 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 626BA6AD
P 1550 6500
F 0 "#PWR0124" H 1550 6250 50  0001 C CNN
F 1 "GND" H 1555 6327 50  0000 C CNN
F 2 "" H 1550 6500 50  0001 C CNN
F 3 "" H 1550 6500 50  0001 C CNN
	1    1550 6500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0125
U 1 1 626BA7AE
P 1100 6500
F 0 "#PWR0125" H 1100 6250 50  0001 C CNN
F 1 "GND" H 1105 6327 50  0000 C CNN
F 2 "" H 1100 6500 50  0001 C CNN
F 3 "" H 1100 6500 50  0001 C CNN
	1    1100 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	2900 6200 2450 6200
Wire Wire Line
	2450 6200 2000 6200
Connection ~ 2450 6200
Wire Wire Line
	2000 6200 1550 6200
Connection ~ 2000 6200
Wire Wire Line
	1550 6200 1100 6200
Connection ~ 1550 6200
$Comp
L power:+3V3 #PWR0126
U 1 1 626BD0CD
P 1100 6200
F 0 "#PWR0126" H 1100 6050 50  0001 C CNN
F 1 "+3V3" H 1115 6373 50  0000 C CNN
F 2 "" H 1100 6200 50  0001 C CNN
F 3 "" H 1100 6200 50  0001 C CNN
	1    1100 6200
	1    0    0    -1  
$EndComp
Connection ~ 1100 6200
$Comp
L Switch:SW_Push SW1
U 1 1 626ACC39
P 3000 4550
F 0 "SW1" H 3000 4835 50  0000 C CNN
F 1 "SW_Push" H 3000 4744 50  0000 C CNN
F 2 "Button_Switch_THT:SW_TH_Tactile_Omron_B3F-10xx" H 3000 4750 50  0001 C CNN
F 3 "~" H 3000 4750 50  0001 C CNN
	1    3000 4550
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 626AE34B
P 3200 4550
F 0 "#PWR0127" H 3200 4300 50  0001 C CNN
F 1 "GND" H 3205 4377 50  0000 C CNN
F 2 "" H 3200 4550 50  0001 C CNN
F 3 "" H 3200 4550 50  0001 C CNN
	1    3200 4550
	1    0    0    -1  
$EndComp
Wire Wire Line
	2800 4400 2800 4550
NoConn ~ 2550 4300
$Comp
L power:GND #PWR0129
U 1 1 626CDC81
P 3300 2400
F 0 "#PWR0129" H 3300 2150 50  0001 C CNN
F 1 "GND" V 3305 2272 50  0000 R CNN
F 2 "" H 3300 2400 50  0001 C CNN
F 3 "" H 3300 2400 50  0001 C CNN
	1    3300 2400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0130
U 1 1 626CDD48
P 4900 2400
F 0 "#PWR0130" H 4900 2150 50  0001 C CNN
F 1 "GND" V 4905 2272 50  0000 R CNN
F 2 "" H 4900 2400 50  0001 C CNN
F 3 "" H 4900 2400 50  0001 C CNN
	1    4900 2400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0131
U 1 1 626CF545
P 6600 2400
F 0 "#PWR0131" H 6600 2150 50  0001 C CNN
F 1 "GND" V 6605 2272 50  0000 R CNN
F 2 "" H 6600 2400 50  0001 C CNN
F 3 "" H 6600 2400 50  0001 C CNN
	1    6600 2400
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0132
U 1 1 626D4A96
P 3700 4500
F 0 "#PWR0132" H 3700 4250 50  0001 C CNN
F 1 "GND" V 3705 4372 50  0000 R CNN
F 2 "" H 3700 4500 50  0001 C CNN
F 3 "" H 3700 4500 50  0001 C CNN
	1    3700 4500
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0133
U 1 1 626D4B31
P 5300 4500
F 0 "#PWR0133" H 5300 4250 50  0001 C CNN
F 1 "GND" V 5305 4372 50  0000 R CNN
F 2 "" H 5300 4500 50  0001 C CNN
F 3 "" H 5300 4500 50  0001 C CNN
	1    5300 4500
	0    1    1    0   
$EndComp
Wire Wire Line
	2950 4100 3300 4100
Connection ~ 2950 4100
Wire Wire Line
	3300 4100 3300 3050
Wire Wire Line
	3300 3050 4900 3050
Wire Wire Line
	4900 3050 4900 2500
Connection ~ 3300 3050
Wire Wire Line
	3300 3050 3300 2500
Wire Wire Line
	3300 4100 3300 4600
Wire Wire Line
	3300 4600 3700 4600
Connection ~ 3300 4100
Wire Wire Line
	2550 4400 2800 4400
Wire Wire Line
	4900 4600 5300 4600
$Comp
L Connector_Generic:Conn_02x20_Odd_Even J1
U 1 1 62778E6D
P 8750 3900
F 0 "J1" H 8800 5017 50  0000 C CNN
F 1 "Conn_02x20" H 8800 4926 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical" H 8750 3900 50  0001 C CNN
F 3 "~" H 8750 3900 50  0001 C CNN
	1    8750 3900
	-1   0    0    1   
$EndComp
Text GLabel 8450 3200 0    50   Input ~ 0
C0
Text GLabel 8450 3100 0    50   Input ~ 0
C2
Text GLabel 8950 3200 2    50   Input ~ 0
C1
Text GLabel 8450 3000 0    50   Input ~ 0
C4
Text GLabel 8450 2900 0    50   Input ~ 0
C6
Text GLabel 8950 3100 2    50   Input ~ 0
C3
Text GLabel 8950 3000 2    50   Input ~ 0
C5
Text GLabel 8950 2900 2    50   Input ~ 0
C7
Text GLabel 8450 3600 0    50   Input ~ 0
D0
Text GLabel 8450 3500 0    50   Input ~ 0
D2
Text GLabel 8450 3400 0    50   Input ~ 0
D4
Text GLabel 8450 3300 0    50   Input ~ 0
D6
Text GLabel 8950 3600 2    50   Input ~ 0
D1
Text GLabel 8950 3500 2    50   Input ~ 0
D3
Text GLabel 8950 3400 2    50   Input ~ 0
D5
Text GLabel 8950 3300 2    50   Input ~ 0
D7
Text GLabel 8450 4500 0    50   Input ~ 0
A0
Text GLabel 8450 4600 0    50   Input ~ 0
A2
Text GLabel 8450 4700 0    50   Input ~ 0
A4
Text GLabel 8450 4800 0    50   Input ~ 0
A6
Text GLabel 8950 4500 2    50   Input ~ 0
A1
Text GLabel 8950 4600 2    50   Input ~ 0
A3
Text GLabel 8950 4700 2    50   Input ~ 0
A5
Text GLabel 8950 4800 2    50   Input ~ 0
A7
Text GLabel 8450 4000 0    50   Input ~ 0
A8
Text GLabel 8450 3900 0    50   Input ~ 0
A10
Text GLabel 8450 3800 0    50   Input ~ 0
A12
Text GLabel 8450 3700 0    50   Input ~ 0
A14
Text GLabel 8950 4000 2    50   Input ~ 0
A9
Text GLabel 8950 3900 2    50   Input ~ 0
A11
Text GLabel 8950 3800 2    50   Input ~ 0
A13
Text GLabel 8950 3700 2    50   Input ~ 0
A15
Text GLabel 8450 4100 0    50   Input ~ 0
C8
Text GLabel 8450 4200 0    50   Input ~ 0
C10
Text GLabel 8450 4300 0    50   Input ~ 0
C12
Text GLabel 8950 4100 2    50   Input ~ 0
C9
Text GLabel 8950 4200 2    50   Input ~ 0
C11
Text GLabel 8950 4300 2    50   Input ~ 0
C13
Wire Wire Line
	3300 4600 3300 5150
Wire Wire Line
	3300 5150 4900 5150
Wire Wire Line
	4900 5150 4900 4600
Connection ~ 3300 4600
Wire Wire Line
	4900 3050 6600 3050
Wire Wire Line
	6600 3050 6600 2500
Connection ~ 4900 3050
$Comp
L power:GND #PWR0128
U 1 1 626C23AC
P 3450 6500
F 0 "#PWR0128" H 3450 6250 50  0001 C CNN
F 1 "GND" H 3455 6327 50  0000 C CNN
F 2 "" H 3450 6500 50  0001 C CNN
F 3 "" H 3450 6500 50  0001 C CNN
	1    3450 6500
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x01 J2
U 1 1 626C0E32
P 3450 6300
F 0 "J2" V 3414 6212 50  0000 R CNN
F 1 "Conn_01x01" V 3323 6212 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x01_P2.54mm_Vertical" H 3450 6300 50  0001 C CNN
F 3 "~" H 3450 6300 50  0001 C CNN
	1    3450 6300
	0    -1   -1   0   
$EndComp
NoConn ~ 1200 2050
NoConn ~ 11500 500 
$Comp
L power:GND #PWR?
U 1 1 628983B4
P 8950 4400
F 0 "#PWR?" H 8950 4150 50  0001 C CNN
F 1 "GND" V 8955 4272 50  0000 R CNN
F 2 "" H 8950 4400 50  0001 C CNN
F 3 "" H 8950 4400 50  0001 C CNN
	1    8950 4400
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 62898A39
P 8450 4400
F 0 "#PWR?" H 8450 4150 50  0001 C CNN
F 1 "GND" V 8455 4272 50  0000 R CNN
F 2 "" H 8450 4400 50  0001 C CNN
F 3 "" H 8450 4400 50  0001 C CNN
	1    8450 4400
	0    1    1    0   
$EndComp
$EndSCHEMATC
