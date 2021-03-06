EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title "RosalynFC"
Date "2021-01-03"
Rev "v0.2"
Comp "www.2-0.dk"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5FF86628
P 3350 2800
AR Path="/5FF86628" Ref="J?"  Part="1" 
AR Path="/5FF7D209/5FF86628" Ref="J11"  Part="1" 
F 0 "J11" H 3430 2792 50  0000 L CNN
F 1 "Camera" H 3430 2701 50  0000 L CNN
F 2 "rosalyn-fc:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal" H 3350 2800 50  0001 C CNN
F 3 "~" H 3350 2800 50  0001 C CNN
	1    3350 2800
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FF8662E
P 3150 2700
AR Path="/5FF8662E" Ref="#PWR?"  Part="1" 
AR Path="/5FF7D209/5FF8662E" Ref="#PWR070"  Part="1" 
F 0 "#PWR070" H 3150 2450 50  0001 C CNN
F 1 "GND" V 3155 2572 50  0000 R CNN
F 2 "" H 3150 2700 50  0001 C CNN
F 3 "" H 3150 2700 50  0001 C CNN
	1    3150 2700
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5FF86634
P 3150 2800
AR Path="/5FF86634" Ref="#PWR?"  Part="1" 
AR Path="/5FF7D209/5FF86634" Ref="#PWR071"  Part="1" 
F 0 "#PWR071" H 3150 2650 50  0001 C CNN
F 1 "+5V" V 3165 2928 50  0000 L CNN
F 2 "" H 3150 2800 50  0001 C CNN
F 3 "" H 3150 2800 50  0001 C CNN
	1    3150 2800
	0    -1   -1   0   
$EndComp
Text GLabel 3150 2900 0    50   Output ~ 0
vin
Text GLabel 3150 3000 0    50   Input ~ 0
ocd
$Comp
L rosalyn-fc:AT7456E U3
U 1 1 5FF868D3
P 5700 3750
F 0 "U3" H 5700 4565 50  0000 C CNN
F 1 "AT7456E" H 5700 4474 50  0000 C CNN
F 2 "Package_SO:TSSOP-28-1EP_4.4x9.7mm_P0.65mm" H 5700 3750 50  0001 C CNN
F 3 "https://datasheets.maximintegrated.com/en/ds/MAX7456.pdf" H 5700 3750 50  0001 C CNN
	1    5700 3750
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR081
U 1 1 5FF87556
P 5850 4600
F 0 "#PWR081" H 5850 4350 50  0001 C CNN
F 1 "GND" H 5855 4427 50  0000 C CNN
F 2 "" H 5850 4600 50  0001 C CNN
F 3 "" H 5850 4600 50  0001 C CNN
	1    5850 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 4500 5700 4550
Wire Wire Line
	5700 4550 5800 4550
Wire Wire Line
	6000 4550 6000 4500
Wire Wire Line
	5900 4500 5900 4550
Connection ~ 5900 4550
Wire Wire Line
	5900 4550 6000 4550
Wire Wire Line
	5800 4500 5800 4550
Connection ~ 5800 4550
Wire Wire Line
	5800 4550 5850 4550
Wire Wire Line
	5850 4600 5850 4550
Connection ~ 5850 4550
Wire Wire Line
	5850 4550 5900 4550
$Comp
L Device:C C24
U 1 1 5FF8807D
P 5100 4650
F 0 "C24" V 4848 4650 50  0000 C CNN
F 1 "100n" V 4939 4650 50  0000 C CNN
F 2 "" H 5138 4500 50  0001 C CNN
F 3 "~" H 5100 4650 50  0001 C CNN
	1    5100 4650
	0    1    1    0   
$EndComp
$Comp
L Device:C C25
U 1 1 5FF888B5
P 5100 5050
F 0 "C25" V 4848 5050 50  0000 C CNN
F 1 "100n" V 4939 5050 50  0000 C CNN
F 2 "" H 5138 4900 50  0001 C CNN
F 3 "~" H 5100 5050 50  0001 C CNN
	1    5100 5050
	0    1    1    0   
$EndComp
$Comp
L Device:C C26
U 1 1 5FF8900E
P 5100 5450
F 0 "C26" V 4848 5450 50  0000 C CNN
F 1 "100n" V 4939 5450 50  0000 C CNN
F 2 "" H 5138 5300 50  0001 C CNN
F 3 "~" H 5100 5450 50  0001 C CNN
	1    5100 5450
	0    1    1    0   
$EndComp
$Comp
L Device:C C27
U 1 1 5FF893C4
P 5100 5850
F 0 "C27" V 4848 5850 50  0000 C CNN
F 1 "10u" V 4939 5850 50  0000 C CNN
F 2 "" H 5138 5700 50  0001 C CNN
F 3 "~" H 5100 5850 50  0001 C CNN
	1    5100 5850
	0    1    1    0   
$EndComp
Wire Wire Line
	5400 4500 5400 4650
Wire Wire Line
	5400 4650 5250 4650
Wire Wire Line
	5500 4500 5500 5050
Wire Wire Line
	5500 5050 5250 5050
Wire Wire Line
	5600 4500 5600 5450
Wire Wire Line
	5600 5450 5250 5450
Wire Wire Line
	5400 4650 5400 5850
Wire Wire Line
	5400 5850 5250 5850
Connection ~ 5400 4650
Wire Wire Line
	5500 5050 5500 5850
Wire Wire Line
	5500 5850 5400 5850
Connection ~ 5500 5050
Connection ~ 5400 5850
Wire Wire Line
	5600 5450 5600 5850
Wire Wire Line
	5600 5850 5500 5850
Connection ~ 5600 5450
Connection ~ 5500 5850
$Comp
L power:GND #PWR075
U 1 1 5FF8B0FC
P 4950 4650
F 0 "#PWR075" H 4950 4400 50  0001 C CNN
F 1 "GND" V 4955 4522 50  0000 R CNN
F 2 "" H 4950 4650 50  0001 C CNN
F 3 "" H 4950 4650 50  0001 C CNN
	1    4950 4650
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR076
U 1 1 5FF8BAD3
P 4950 5050
F 0 "#PWR076" H 4950 4800 50  0001 C CNN
F 1 "GND" V 4955 4922 50  0000 R CNN
F 2 "" H 4950 5050 50  0001 C CNN
F 3 "" H 4950 5050 50  0001 C CNN
	1    4950 5050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR077
U 1 1 5FF8C1F1
P 4950 5450
F 0 "#PWR077" H 4950 5200 50  0001 C CNN
F 1 "GND" V 4955 5322 50  0000 R CNN
F 2 "" H 4950 5450 50  0001 C CNN
F 3 "" H 4950 5450 50  0001 C CNN
	1    4950 5450
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR078
U 1 1 5FF8C7E6
P 4950 5850
F 0 "#PWR078" H 4950 5600 50  0001 C CNN
F 1 "GND" V 4955 5722 50  0000 R CNN
F 2 "" H 4950 5850 50  0001 C CNN
F 3 "" H 4950 5850 50  0001 C CNN
	1    4950 5850
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR080
U 1 1 5FF8D102
P 5250 5950
F 0 "#PWR080" H 5250 5800 50  0001 C CNN
F 1 "+3.3V" H 5265 6123 50  0000 C CNN
F 2 "" H 5250 5950 50  0001 C CNN
F 3 "" H 5250 5950 50  0001 C CNN
	1    5250 5950
	-1   0    0    1   
$EndComp
Wire Wire Line
	5250 5950 5250 5850
Connection ~ 5250 5850
NoConn ~ 5200 3750
NoConn ~ 5200 3850
NoConn ~ 5200 3950
NoConn ~ 6200 4050
$Comp
L power:+3.3V #PWR079
U 1 1 5FF8F6E5
P 5200 4050
F 0 "#PWR079" H 5200 3900 50  0001 C CNN
F 1 "+3.3V" V 5215 4178 50  0000 L CNN
F 2 "" H 5200 4050 50  0001 C CNN
F 3 "" H 5200 4050 50  0001 C CNN
	1    5200 4050
	0    -1   -1   0   
$EndComp
Text GLabel 6200 3250 2    50   Input ~ 0
ocd_cs
Text GLabel 6200 3350 2    50   Input ~ 0
spi2_mosi
Text GLabel 6200 3450 2    50   Input ~ 0
spi2_sck
Text GLabel 6200 3550 2    50   3State ~ 0
spi2_miso
$Comp
L Device:Crystal_GND24 Y2
U 1 1 5FF96385
P 6800 3900
F 0 "Y2" V 6650 3950 50  0000 L CNN
F 1 "27MHz" V 6950 3950 50  0000 L CNN
F 2 "" H 6800 3900 50  0001 C CNN
F 3 "~" H 6800 3900 50  0001 C CNN
	1    6800 3900
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR083
U 1 1 5FF98D98
P 7000 3900
F 0 "#PWR083" H 7000 3650 50  0001 C CNN
F 1 "GND" V 7005 3772 50  0000 R CNN
F 2 "" H 7000 3900 50  0001 C CNN
F 3 "" H 7000 3900 50  0001 C CNN
	1    7000 3900
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR082
U 1 1 5FF994EA
P 6600 3900
F 0 "#PWR082" H 6600 3650 50  0001 C CNN
F 1 "GND" V 6605 3772 50  0000 R CNN
F 2 "" H 6600 3900 50  0001 C CNN
F 3 "" H 6600 3900 50  0001 C CNN
	1    6600 3900
	0    1    1    0   
$EndComp
Wire Wire Line
	6200 3950 6300 3950
Wire Wire Line
	6200 3850 6300 3850
Text GLabel 4300 2450 0    50   Input ~ 0
vin
Text GLabel 4300 3550 0    50   Output ~ 0
vout
$Comp
L Connector_Generic:Conn_01x04 J?
U 1 1 5FFAAD46
P 3350 3400
AR Path="/5FFAAD46" Ref="J?"  Part="1" 
AR Path="/5FF7D209/5FFAAD46" Ref="J12"  Part="1" 
F 0 "J12" H 3430 3392 50  0000 L CNN
F 1 "VTX" H 3430 3301 50  0000 L CNN
F 2 "rosalyn-fc:JST_SH_SM04B-SRSS-TB_1x04-1MP_P1.00mm_Horizontal" H 3350 3400 50  0001 C CNN
F 3 "~" H 3350 3400 50  0001 C CNN
	1    3350 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FFAAD4C
P 3150 3300
AR Path="/5FFAAD4C" Ref="#PWR?"  Part="1" 
AR Path="/5FF7D209/5FFAAD4C" Ref="#PWR072"  Part="1" 
F 0 "#PWR072" H 3150 3050 50  0001 C CNN
F 1 "GND" V 3155 3172 50  0000 R CNN
F 2 "" H 3150 3300 50  0001 C CNN
F 3 "" H 3150 3300 50  0001 C CNN
	1    3150 3300
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5FFAAD52
P 3150 3400
AR Path="/5FFAAD52" Ref="#PWR?"  Part="1" 
AR Path="/5FF7D209/5FFAAD52" Ref="#PWR073"  Part="1" 
F 0 "#PWR073" H 3150 3250 50  0001 C CNN
F 1 "+5V" V 3165 3528 50  0000 L CNN
F 2 "" H 3150 3400 50  0001 C CNN
F 3 "" H 3150 3400 50  0001 C CNN
	1    3150 3400
	0    -1   -1   0   
$EndComp
Text GLabel 3150 3500 0    50   Input ~ 0
vout
Text GLabel 3150 3600 0    50   Input ~ 0
uart5_tx
$Comp
L Device:CP C23
U 1 1 5FFABED9
P 4800 3550
F 0 "C23" V 4545 3550 50  0000 C CNN
F 1 "47u" V 4636 3550 50  0000 C CNN
F 2 "" H 4838 3400 50  0001 C CNN
F 3 "~" H 4800 3550 50  0001 C CNN
	1    4800 3550
	0    1    1    0   
$EndComp
$Comp
L Device:CP C22
U 1 1 5FFACE1E
P 4800 3150
F 0 "C22" V 4545 3150 50  0000 C CNN
F 1 "47u" V 4636 3150 50  0000 C CNN
F 2 "" H 4838 3000 50  0001 C CNN
F 3 "~" H 4800 3150 50  0001 C CNN
	1    4800 3150
	0    1    1    0   
$EndComp
$Comp
L Device:C C21
U 1 1 5FFAED95
P 4800 2450
F 0 "C21" V 4548 2450 50  0000 C CNN
F 1 "100n" V 4639 2450 50  0000 C CNN
F 2 "" H 4838 2300 50  0001 C CNN
F 3 "~" H 4800 2450 50  0001 C CNN
	1    4800 2450
	0    1    1    0   
$EndComp
Wire Wire Line
	4950 2450 5100 2450
$Comp
L power:GND #PWR074
U 1 1 5FFC3E11
P 4450 2750
F 0 "#PWR074" H 4450 2500 50  0001 C CNN
F 1 "GND" H 4455 2577 50  0000 C CNN
F 2 "" H 4450 2750 50  0001 C CNN
F 3 "" H 4450 2750 50  0001 C CNN
	1    4450 2750
	1    0    0    -1  
$EndComp
$Comp
L Device:R R4
U 1 1 5FFD6522
P 4450 3550
F 0 "R4" V 4657 3550 50  0000 C CNN
F 1 "75" V 4566 3550 50  0000 C CNN
F 2 "" V 4380 3550 50  0001 C CNN
F 3 "~" H 4450 3550 50  0001 C CNN
	1    4450 3550
	0    1    -1   0   
$EndComp
$Comp
L Device:R R3
U 1 1 5FFE9FB4
P 4450 2600
F 0 "R3" H 4520 2646 50  0000 L CNN
F 1 "75" H 4520 2555 50  0000 L CNN
F 2 "" V 4380 2600 50  0001 C CNN
F 3 "~" H 4450 2600 50  0001 C CNN
	1    4450 2600
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 3550 4650 3550
Wire Wire Line
	4300 2450 4450 2450
Wire Wire Line
	4650 3150 4650 3550
Connection ~ 4650 3550
Wire Wire Line
	5000 3450 5000 3150
Wire Wire Line
	5000 3150 4950 3150
Wire Wire Line
	5100 3350 5200 3350
Wire Wire Line
	5000 3450 5200 3450
Wire Wire Line
	4950 3550 5200 3550
Wire Wire Line
	5100 2450 5100 3350
Connection ~ 4450 2450
Wire Wire Line
	4450 2450 4650 2450
Wire Wire Line
	6300 3750 6300 3850
Wire Wire Line
	6300 3950 6300 4050
Wire Wire Line
	6300 3750 6800 3750
Wire Wire Line
	6300 4050 6800 4050
$EndSCHEMATC
