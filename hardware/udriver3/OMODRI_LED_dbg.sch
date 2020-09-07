EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 17 17
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 5500 3500 0    50   Input ~ 0
VDD_3V3
$Comp
L power:GND #PWR?
U 1 1 5FA05E5D
P 6075 4500
AR Path="/5F310311/5FA05E5D" Ref="#PWR?"  Part="1" 
AR Path="/5F3255E8/5FA05E5D" Ref="#PWR?"  Part="1" 
AR Path="/5F387075/5FA05E5D" Ref="#PWR?"  Part="1" 
AR Path="/5FA05E5D" Ref="#PWR?"  Part="1" 
AR Path="/5F9FCB8C/5FA05E5D" Ref="#PWR0142"  Part="1" 
F 0 "#PWR0142" H 6075 4250 50  0001 C CNN
F 1 "GND" H 6080 4327 50  0000 C CNN
F 2 "" H 6075 4500 50  0001 C CNN
F 3 "" H 6075 4500 50  0001 C CNN
	1    6075 4500
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5FA05E63
P 5600 3900
AR Path="/5F3A3F16/5FA05E63" Ref="#PWR?"  Part="1" 
AR Path="/5F7A5BAE/5FA05E63" Ref="#PWR?"  Part="1" 
AR Path="/5F68F8CB/5FA05E63" Ref="#PWR?"  Part="1" 
AR Path="/5F4C38EC/5FA05E63" Ref="#PWR?"  Part="1" 
AR Path="/5FA05E63" Ref="#PWR?"  Part="1" 
AR Path="/5F9FCB8C/5FA05E63" Ref="#PWR0141"  Part="1" 
F 0 "#PWR0141" H 5600 3650 50  0001 C CNN
F 1 "GND" H 5605 3727 50  0000 C CNN
F 2 "" H 5600 3900 50  0001 C CNN
F 3 "" H 5600 3900 50  0001 C CNN
	1    5600 3900
	1    0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5FA05E6F
P 5600 3650
AR Path="/5F3A3F16/5FA05E6F" Ref="C?"  Part="1" 
AR Path="/5F7A5BAE/5FA05E6F" Ref="C?"  Part="1" 
AR Path="/5F68F8CB/5FA05E6F" Ref="C?"  Part="1" 
AR Path="/5F4C38EC/5FA05E6F" Ref="C?"  Part="1" 
AR Path="/5FA05E6F" Ref="C?"  Part="1" 
AR Path="/5F9FCB8C/5FA05E6F" Ref="C113"  Part="1" 
F 0 "C113" H 5715 3696 50  0000 L CNN
F 1 "100nF" H 5715 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0201_0603Metric" H 5638 3500 50  0001 C CNN
F 3 "https://www.murata.com/en-eu/products/productdetail?partno=GRM033C71A104KE14%23" H 5600 3650 50  0001 C CNN
F 4 "GRM033C71A104KE14D" H 5600 3650 50  0001 C CNN "Part No"
F 5 "10V" H 5600 3650 50  0001 C CNN "Rated Voltage"
F 6 "2990681" H 5600 3650 50  0001 C CNN "Farnell"
F 7 "185-1751" H 5600 3650 50  0001 C CNN "RS"
F 8 "81-GRM033C71A104KE4D" H 5600 3650 50  0001 C CNN "Mouser"
F 9 "490-14450-1-ND" H 5600 3650 50  0001 C CNN "DigiKey"
	1    5600 3650
	1    0    0    -1  
$EndComp
$Comp
L LED:WS2812B D?
U 1 1 5FA05E75
P 6075 4100
AR Path="/60EE22FD/5FA05E75" Ref="D?"  Part="1" 
AR Path="/5FA05E75" Ref="D?"  Part="1" 
AR Path="/5F9FCB8C/5FA05E75" Ref="D5"  Part="1" 
F 0 "D5" H 5875 3850 50  0000 L CNN
F 1 "WS2812B" H 5675 4350 50  0000 L CNN
F 2 "LED_SMD:LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm" H 6125 3800 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf" H 6175 3725 50  0001 L TNN
	1    6075 4100
	-1   0    0    -1  
$EndComp
Wire Wire Line
	6075 4500 6075 4400
NoConn ~ 5775 4100
Wire Wire Line
	5500 3500 5600 3500
Wire Wire Line
	6075 3500 6075 3800
Connection ~ 5600 3500
Wire Wire Line
	5600 3900 5600 3800
Wire Wire Line
	6375 4100 6525 4100
Wire Wire Line
	5600 3500 6075 3500
Text HLabel 6525 4100 2    50   Input ~ 0
WS2812B_CMD
$EndSCHEMATC
