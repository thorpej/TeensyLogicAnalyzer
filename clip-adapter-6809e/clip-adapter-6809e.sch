EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A 11000 8500
encoding utf-8
Sheet 1 1
Title "TeensyLogicAnalyzer 6809E clip adapter"
Date "2022-04-30"
Rev "1.0"
Comp "Copyright (c) 2022 Jason R. Thorpe.  See LICENSE."
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_02x20_Odd_Even J1
U 1 1 62778E6D
P 3100 2400
F 0 "J1" H 3150 3517 50  0000 C CNN
F 1 "Conn_02x20" H 3150 3426 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_2x20_P2.54mm_Vertical" H 3100 2400 50  0001 C CNN
F 3 "~" H 3100 2400 50  0001 C CNN
	1    3100 2400
	-1   0    0    1   
$EndComp
Text GLabel 2800 1700 0    50   Input ~ 0
C0
Text GLabel 2800 1600 0    50   Input ~ 0
C2
Text GLabel 3300 1700 2    50   Input ~ 0
C1
Text GLabel 2800 1500 0    50   Input ~ 0
C4
Text GLabel 2800 1400 0    50   Input ~ 0
C6
Text GLabel 3300 1600 2    50   Input ~ 0
C3
Text GLabel 3300 1500 2    50   Input ~ 0
C5
Text GLabel 3300 1400 2    50   Input ~ 0
C7
Text GLabel 2800 2100 0    50   Input ~ 0
D0
Text GLabel 2800 2000 0    50   Input ~ 0
D2
Text GLabel 2800 1900 0    50   Input ~ 0
D4
Text GLabel 2800 1800 0    50   Input ~ 0
D6
Text GLabel 3300 2100 2    50   Input ~ 0
D1
Text GLabel 3300 2000 2    50   Input ~ 0
D3
Text GLabel 3300 1900 2    50   Input ~ 0
D5
Text GLabel 3300 1800 2    50   Input ~ 0
D7
Text GLabel 2800 3000 0    50   Input ~ 0
A0
Text GLabel 2800 3100 0    50   Input ~ 0
A2
Text GLabel 2800 3200 0    50   Input ~ 0
A4
Text GLabel 2800 3300 0    50   Input ~ 0
A6
Text GLabel 3300 3000 2    50   Input ~ 0
A1
Text GLabel 3300 3100 2    50   Input ~ 0
A3
Text GLabel 3300 3200 2    50   Input ~ 0
A5
Text GLabel 3300 3300 2    50   Input ~ 0
A7
Text GLabel 2800 2500 0    50   Input ~ 0
A8
Text GLabel 2800 2400 0    50   Input ~ 0
A10
Text GLabel 2800 2300 0    50   Input ~ 0
A12
Text GLabel 2800 2200 0    50   Input ~ 0
A14
Text GLabel 3300 2500 2    50   Input ~ 0
A9
Text GLabel 3300 2400 2    50   Input ~ 0
A11
Text GLabel 3300 2300 2    50   Input ~ 0
A13
Text GLabel 3300 2200 2    50   Input ~ 0
A15
Text GLabel 2800 2600 0    50   Input ~ 0
C8
Text GLabel 2800 2700 0    50   Input ~ 0
C10
Text GLabel 2800 2800 0    50   Input ~ 0
C12
Text GLabel 3300 2700 2    50   Input ~ 0
C11
Text GLabel 3300 2800 2    50   Input ~ 0
C13
NoConn ~ 11500 500 
$Comp
L Connector_Generic:Conn_01x20 J2
U 1 1 6287A8D2
P 5950 2350
F 0 "J2" H 6030 2342 50  0000 L CNN
F 1 "Conn_01x20" H 6030 2251 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 5950 2350 50  0001 C CNN
F 3 "~" H 5950 2350 50  0001 C CNN
	1    5950 2350
	1    0    0    -1  
$EndComp
Text Notes 5850 1350 0    50   ~ 0
Pin 1
Text Notes 5850 3500 0    50   ~ 0
Pin 20
$Comp
L Connector_Generic:Conn_01x20 J4
U 1 1 6288332E
P 5950 4700
F 0 "J4" H 6030 4692 50  0000 L CNN
F 1 "Conn_01x20" H 6030 4601 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 5950 4700 50  0001 C CNN
F 3 "~" H 5950 4700 50  0001 C CNN
	1    5950 4700
	1    0    0    -1  
$EndComp
Text Notes 5850 3700 0    50   ~ 0
Pin 1
Text Notes 5850 5850 0    50   ~ 0
Pin 20
Wire Wire Line
	5750 5700 5700 5700
Wire Wire Line
	5700 5700 5700 3350
Wire Wire Line
	5700 3350 5750 3350
Wire Wire Line
	5750 5600 5650 5600
Wire Wire Line
	5650 5600 5650 3250
Wire Wire Line
	5650 3250 5750 3250
Wire Wire Line
	5750 5500 5600 5500
Wire Wire Line
	5600 5500 5600 3150
Wire Wire Line
	5600 3150 5750 3150
Wire Wire Line
	5750 5400 5550 5400
Wire Wire Line
	5550 5400 5550 3050
Wire Wire Line
	5550 3050 5750 3050
Wire Wire Line
	5750 5300 5500 5300
Wire Wire Line
	5500 5300 5500 2950
Wire Wire Line
	5500 2950 5750 2950
Wire Wire Line
	5750 5200 5450 5200
Wire Wire Line
	5450 5200 5450 2850
Wire Wire Line
	5450 2850 5750 2850
Wire Wire Line
	5750 5100 5400 5100
Wire Wire Line
	5400 5100 5400 2750
Wire Wire Line
	5400 2750 5750 2750
Wire Wire Line
	5750 5000 5350 5000
Wire Wire Line
	5350 5000 5350 2650
Wire Wire Line
	5350 2650 5750 2650
Wire Wire Line
	5750 4900 5300 4900
Wire Wire Line
	5300 4900 5300 2550
Wire Wire Line
	5300 2550 5750 2550
Wire Wire Line
	5750 4800 5250 4800
Wire Wire Line
	5250 4800 5250 2450
Wire Wire Line
	5250 2450 5750 2450
Wire Wire Line
	5750 4700 5200 4700
Wire Wire Line
	5200 4700 5200 2350
Wire Wire Line
	5200 2350 5750 2350
Wire Wire Line
	5750 4600 5150 4600
Wire Wire Line
	5150 4600 5150 2250
Wire Wire Line
	5150 2250 5750 2250
Wire Wire Line
	5750 4500 5100 4500
Wire Wire Line
	5100 4500 5100 2150
Wire Wire Line
	5100 2150 5750 2150
Wire Wire Line
	5750 4400 5050 4400
Wire Wire Line
	5050 4400 5050 2050
Wire Wire Line
	5050 2050 5750 2050
Wire Wire Line
	5750 4300 5000 4300
Wire Wire Line
	5000 4300 5000 1950
Wire Wire Line
	5000 1950 5750 1950
Wire Wire Line
	5750 4200 4950 4200
Wire Wire Line
	4950 4200 4950 1850
Wire Wire Line
	4950 1850 5750 1850
Wire Wire Line
	5750 4100 4900 4100
Wire Wire Line
	4900 4100 4900 1750
Wire Wire Line
	4900 1750 5750 1750
Wire Wire Line
	5750 4000 4850 4000
Wire Wire Line
	4850 4000 4850 1650
Wire Wire Line
	4850 1650 5750 1650
Wire Wire Line
	5750 3900 4800 3900
Wire Wire Line
	4800 3900 4800 1550
Wire Wire Line
	4800 1550 5750 1550
Wire Wire Line
	5750 3800 4750 3800
Wire Wire Line
	4750 3800 4750 1450
Wire Wire Line
	4750 1450 5750 1450
Text Notes 7250 3500 0    50   ~ 0
Pin 21
Text Notes 7250 1350 0    50   ~ 0
Pin 40
Text GLabel 7550 1750 2    50   Output ~ 0
C3
Text GLabel 7550 2050 2    50   Output ~ 0
C0
Text GLabel 7550 2250 2    50   Output ~ 0
C2
Text GLabel 7550 2350 2    50   Output ~ 0
D0
Text GLabel 7550 2450 2    50   Output ~ 0
D1
Text GLabel 7550 2550 2    50   Output ~ 0
D2
Text GLabel 7550 2650 2    50   Output ~ 0
D3
Text GLabel 7550 2750 2    50   Output ~ 0
D4
Text GLabel 7550 2850 2    50   Output ~ 0
D5
Text GLabel 7550 2950 2    50   Output ~ 0
D6
Text GLabel 7550 3050 2    50   Output ~ 0
D7
Text GLabel 7550 3150 2    50   Output ~ 0
A15
Text GLabel 7550 3250 2    50   Output ~ 0
A14
Text GLabel 7550 3350 2    50   Output ~ 0
A13
$Comp
L power:GND #PWR0102
U 1 1 628A4484
P 4700 1450
F 0 "#PWR0102" H 4700 1200 50  0001 C CNN
F 1 "GND" V 4705 1322 50  0000 R CNN
F 2 "" H 4700 1450 50  0001 C CNN
F 3 "" H 4700 1450 50  0001 C CNN
	1    4700 1450
	0    1    1    0   
$EndComp
Connection ~ 4750 1450
Text GLabel 4650 1550 0    50   Output ~ 0
C5
Text GLabel 4650 1650 0    50   Output ~ 0
C4
Text GLabel 4650 1750 0    50   Output ~ 0
C6
Text GLabel 4650 2150 0    50   Output ~ 0
A0
Text GLabel 4650 2250 0    50   Output ~ 0
A1
Text GLabel 4650 2350 0    50   Output ~ 0
A2
Text GLabel 4650 2450 0    50   Output ~ 0
A3
Text GLabel 4650 2550 0    50   Output ~ 0
A4
Text GLabel 4650 2650 0    50   Output ~ 0
A5
Text GLabel 4650 2750 0    50   Output ~ 0
A6
Text GLabel 4650 2850 0    50   Output ~ 0
A7
Text GLabel 4650 2950 0    50   Output ~ 0
A8
Text GLabel 4650 3050 0    50   Output ~ 0
A9
Text GLabel 4650 3150 0    50   Output ~ 0
A10
Text GLabel 4650 3250 0    50   Output ~ 0
A11
Wire Wire Line
	4700 1450 4750 1450
Wire Wire Line
	4650 1550 4800 1550
Connection ~ 4800 1550
Wire Wire Line
	4650 1650 4850 1650
Connection ~ 4850 1650
Wire Wire Line
	4650 1750 4900 1750
Connection ~ 4900 1750
Wire Wire Line
	4650 1950 5000 1950
Connection ~ 5000 1950
Wire Wire Line
	4650 2250 5150 2250
Connection ~ 5150 2250
Wire Wire Line
	4650 2350 5200 2350
Connection ~ 5200 2350
Wire Wire Line
	4650 2450 5250 2450
Connection ~ 5250 2450
Wire Wire Line
	4650 2550 5300 2550
Connection ~ 5300 2550
Wire Wire Line
	4650 2650 5350 2650
Connection ~ 5350 2650
Wire Wire Line
	4650 2750 5400 2750
Connection ~ 5400 2750
Wire Wire Line
	4650 2850 5450 2850
Connection ~ 5450 2850
Wire Wire Line
	4650 2950 5500 2950
Connection ~ 5500 2950
Wire Wire Line
	4650 3050 5550 3050
Connection ~ 5550 3050
Wire Wire Line
	4650 3150 5600 3150
Connection ~ 5600 3150
Wire Wire Line
	4650 3250 5650 3250
Connection ~ 5650 3250
Wire Wire Line
	4650 3350 5700 3350
Connection ~ 5700 3350
$Comp
L power:GND #PWR0103
U 1 1 628D211A
P 3300 2900
F 0 "#PWR0103" H 3300 2650 50  0001 C CNN
F 1 "GND" V 3305 2772 50  0000 R CNN
F 2 "" H 3300 2900 50  0001 C CNN
F 3 "" H 3300 2900 50  0001 C CNN
	1    3300 2900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 628D2534
P 2800 2900
F 0 "#PWR0104" H 2800 2650 50  0001 C CNN
F 1 "GND" V 2805 2772 50  0000 R CNN
F 2 "" H 2800 2900 50  0001 C CNN
F 3 "" H 2800 2900 50  0001 C CNN
	1    2800 2900
	0    1    1    0   
$EndComp
Text Notes 2450 3750 0    50   ~ 0
6809E uses all of control signals.
Text GLabel 3300 2600 2    50   Input ~ 0
C9
Text GLabel 4650 1850 0    50   Output ~ 0
C9
Wire Wire Line
	4650 1850 4950 1850
Connection ~ 4950 1850
Text GLabel 7550 1450 2    50   Output ~ 0
C13
Text GLabel 4650 1950 0    50   Output ~ 0
C8
Text GLabel 7550 1850 2    50   Output ~ 0
C11
Text GLabel 4650 3350 0    50   Output ~ 0
A12
Text GLabel 7550 2150 2    50   Output ~ 0
C12
Text GLabel 7550 1950 2    50   Output ~ 0
C1
Wire Wire Line
	4650 2150 5100 2150
Connection ~ 5100 2150
$Comp
L Connector_Generic:Conn_01x20 J3
U 1 1 62883555
P 7350 2350
F 0 "J3" H 7450 2350 50  0000 C CNN
F 1 "Conn_01x20" H 7650 2250 50  0000 C CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x20_P2.54mm_Vertical" H 7350 2350 50  0001 C CNN
F 3 "~" H 7350 2350 50  0001 C CNN
	1    7350 2350
	-1   0    0    -1  
$EndComp
Text GLabel 7550 1550 2    50   Output ~ 0
C10
Text GLabel 7550 1650 2    50   Output ~ 0
C7
$EndSCHEMATC
