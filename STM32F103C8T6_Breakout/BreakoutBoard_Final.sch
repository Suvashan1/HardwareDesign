EESchema Schematic File Version 4
LIBS:BreakoutBoard_Final-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "STM32F103C8T6_Breakout_Board"
Date ""
Rev ""
Comp "Suvashan Pillay"
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_ST_STM32F1:STM32F103C8Tx U3
U 1 1 5F7F83BE
P 5415 3990
F 0 "U3" H 5865 2510 50  0000 C CNN
F 1 "STM32F103C8Tx" H 5970 2435 50  0000 C CNN
F 2 "Package_QFP:LQFP-48_7x7mm_P0.5mm" H 4815 2590 50  0001 R CNN
F 3 "http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00161566.pdf" H 5415 3990 50  0001 C CNN
F 4 "C8734" H 0   0   50  0001 C CNN "LCSC Part #"
	1    5415 3990
	1    0    0    -1  
$EndComp
Text GLabel 6015 5090 2    50   Input ~ 0
SWD_IO
Text GLabel 1475 5030 0    50   Input ~ 0
USB_-
Text GLabel 6015 4990 2    50   Input ~ 0
USB_+
Text GLabel 6015 5190 2    50   Input ~ 0
SW_CLK
Text GLabel 8195 2445 3    50   Input ~ 0
I2C_SDA
Text GLabel 4715 4390 0    50   Input ~ 0
I2C_SCL
Text GLabel 4715 4090 0    50   Input ~ 0
SWO
Text GLabel 6015 4290 2    50   Input ~ 0
SPI_SCK
Text GLabel 6015 4390 2    50   Input ~ 0
SPI_MISO
Text GLabel 6015 4490 2    50   Input ~ 0
SPI_MOSI
Text GLabel 6015 4790 2    50   Input ~ 0
USART1_RX
Text GLabel 6015 4690 2    50   Input ~ 0
USART1_TX
$Comp
L Device:D_Schottky D1
U 1 1 5F808B6D
P 1560 1290
F 0 "D1" H 1560 1074 50  0000 C CNN
F 1 "B5819W" H 1560 1165 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 1560 1290 50  0001 C CNN
F 3 "~" H 1560 1290 50  0001 C CNN
F 4 "C8598" H 1560 1290 50  0001 C CNN "LCSC Part #"
	1    1560 1290
	-1   0    0    1   
$EndComp
$Comp
L Regulator_Linear:NCP1117-3.3_SOT223 U2
U 1 1 5F80935E
P 2630 1290
F 0 "U2" H 2680 1425 50  0000 C CNN
F 1 "NCP1117" H 2710 1505 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-223-3_TabPin2" H 2630 1490 50  0001 C CNN
F 3 "http://www.onsemi.com/pub_link/Collateral/NCP1117-D.PDF" H 2730 1040 50  0001 C CNN
F 4 "C6186" H 0   0   50  0001 C CNN "LCSC Part #"
	1    2630 1290
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C4
U 1 1 5F809C21
P 2930 1390
F 0 "C4" H 3030 1425 50  0000 L CNN
F 1 "10u" H 3010 1345 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2930 1390 50  0001 C CNN
F 3 "~" H 2930 1390 50  0001 C CNN
F 4 "C15850" H 0   0   50  0001 C CNN "LCSC Part #"
	1    2930 1390
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1590 2630 1590
Wire Wire Line
	2930 1490 2930 1590
Wire Wire Line
	2930 1590 2630 1590
Connection ~ 2630 1590
$Comp
L power:GND #PWR07
U 1 1 5F809D38
P 2630 1590
F 0 "#PWR07" H 2630 1340 50  0001 C CNN
F 1 "GND" H 2635 1417 50  0000 C CNN
F 2 "" H 2630 1590 50  0001 C CNN
F 3 "" H 2630 1590 50  0001 C CNN
	1    2630 1590
	1    0    0    -1  
$EndComp
Wire Wire Line
	2930 1290 3205 1290
Connection ~ 2930 1290
Wire Wire Line
	1175 1290 1410 1290
$Comp
L power:+3.3V #PWR08
U 1 1 5F80A24A
P 3205 1290
F 0 "#PWR08" H 3205 1140 50  0001 C CNN
F 1 "+3.3V" H 3220 1463 50  0000 C CNN
F 2 "" H 3205 1290 50  0001 C CNN
F 3 "" H 3205 1290 50  0001 C CNN
	1    3205 1290
	1    0    0    -1  
$EndComp
Wire Notes Line
	1100 980  1100 1800
Text Notes 1095 930  0    50   ~ 0
Power supply and filtering\n
$Comp
L power:+3.3V #PWR012
U 1 1 5F80A632
P 5215 2435
F 0 "#PWR012" H 5215 2285 50  0001 C CNN
F 1 "+3.3V" H 5230 2608 50  0000 C CNN
F 2 "" H 5215 2435 50  0001 C CNN
F 3 "" H 5215 2435 50  0001 C CNN
	1    5215 2435
	1    0    0    -1  
$EndComp
Wire Wire Line
	5215 2490 5215 2465
Wire Wire Line
	5315 2490 5315 2465
Wire Wire Line
	5315 2465 5215 2465
Connection ~ 5215 2465
Wire Wire Line
	5215 2465 5215 2435
Wire Wire Line
	5415 2490 5415 2465
Wire Wire Line
	5415 2465 5315 2465
Connection ~ 5315 2465
Wire Wire Line
	5515 2490 5515 2465
Wire Wire Line
	5515 2465 5415 2465
Connection ~ 5415 2465
$Comp
L Device:C_Small C6
U 1 1 5F80AE3A
P 4075 1385
F 0 "C6" H 4167 1431 50  0000 L CNN
F 1 "100n" H 4167 1340 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4075 1385 50  0001 C CNN
F 3 "~" H 4075 1385 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    4075 1385
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C8
U 1 1 5F80AE97
P 4590 1380
F 0 "C8" H 4682 1426 50  0000 L CNN
F 1 "100n" H 4682 1335 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 4590 1380 50  0001 C CNN
F 3 "~" H 4590 1380 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    4590 1380
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C10
U 1 1 5F80AFB7
P 5090 1370
F 0 "C10" H 5182 1416 50  0000 L CNN
F 1 "100n" H 5182 1325 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5090 1370 50  0001 C CNN
F 3 "~" H 5090 1370 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    5090 1370
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C11
U 1 1 5F80B10A
P 5600 1385
F 0 "C11" H 5692 1431 50  0000 L CNN
F 1 "100n" H 5692 1340 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 5600 1385 50  0001 C CNN
F 3 "~" H 5600 1385 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    5600 1385
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C12
U 1 1 5F80B232
P 6070 1385
F 0 "C12" H 6162 1431 50  0000 L CNN
F 1 "100n" H 6162 1340 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6070 1385 50  0001 C CNN
F 3 "~" H 6070 1385 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    6070 1385
	1    0    0    -1  
$EndComp
Wire Wire Line
	4075 1485 4075 1540
Wire Wire Line
	4590 1540 4590 1480
Wire Wire Line
	5090 1470 5090 1540
Wire Wire Line
	4075 1540 4590 1540
Connection ~ 4590 1540
Wire Wire Line
	4590 1540 5090 1540
Wire Wire Line
	5600 1485 5600 1540
Wire Wire Line
	5600 1540 5090 1540
Connection ~ 5090 1540
Wire Wire Line
	6070 1485 6070 1540
Connection ~ 5600 1540
Wire Wire Line
	5600 1540 6070 1540
Wire Wire Line
	6070 1285 6070 1230
Wire Wire Line
	5600 1285 5600 1230
Wire Wire Line
	5600 1230 6070 1230
Wire Wire Line
	5090 1270 5090 1230
Wire Wire Line
	5090 1230 5600 1230
Connection ~ 5600 1230
Wire Wire Line
	4590 1280 4590 1230
Wire Wire Line
	4590 1230 5090 1230
Connection ~ 5090 1230
Wire Wire Line
	4075 1285 4075 1230
Wire Wire Line
	4075 1230 4590 1230
Connection ~ 4590 1230
Wire Wire Line
	3735 1285 3735 1230
Wire Wire Line
	3735 1230 4075 1230
Connection ~ 4075 1230
Wire Wire Line
	3735 1485 3735 1540
Wire Wire Line
	3735 1540 4075 1540
Connection ~ 4075 1540
$Comp
L power:+3.3VA #PWR014
U 1 1 5F810095
P 5615 2480
F 0 "#PWR014" H 5615 2330 50  0001 C CNN
F 1 "+3.3VA" H 5630 2653 50  0000 C CNN
F 2 "" H 5615 2480 50  0001 C CNN
F 3 "" H 5615 2480 50  0001 C CNN
	1    5615 2480
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR015
U 1 1 5F810C8C
P 8360 1235
F 0 "#PWR015" H 8360 1085 50  0001 C CNN
F 1 "+3.3V" H 8375 1408 50  0000 C CNN
F 2 "" H 8360 1235 50  0001 C CNN
F 3 "" H 8360 1235 50  0001 C CNN
	1    8360 1235
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3VA #PWR021
U 1 1 5F814053
P 8945 1235
F 0 "#PWR021" H 8945 1085 50  0001 C CNN
F 1 "+3.3VA" H 8960 1408 50  0000 C CNN
F 2 "" H 8945 1235 50  0001 C CNN
F 3 "" H 8945 1235 50  0001 C CNN
	1    8945 1235
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5F814931
P 3735 1230
F 0 "#PWR09" H 3735 1080 50  0001 C CNN
F 1 "+3.3V" H 3750 1403 50  0000 C CNN
F 2 "" H 3735 1230 50  0001 C CNN
F 3 "" H 3735 1230 50  0001 C CNN
	1    3735 1230
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5F815297
P 3735 1540
F 0 "#PWR010" H 3735 1290 50  0001 C CNN
F 1 "GND" H 3740 1367 50  0000 C CNN
F 2 "" H 3735 1540 50  0001 C CNN
F 3 "" H 3735 1540 50  0001 C CNN
	1    3735 1540
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR020
U 1 1 5F815D1E
P 8865 1490
F 0 "#PWR020" H 8865 1240 50  0001 C CNN
F 1 "GND" H 8870 1317 50  0000 C CNN
F 2 "" H 8865 1490 50  0001 C CNN
F 3 "" H 8865 1490 50  0001 C CNN
	1    8865 1490
	1    0    0    -1  
$EndComp
Connection ~ 3735 1230
Connection ~ 3735 1540
Wire Notes Line
	3405 970  3405 980 
Wire Notes Line
	1100 1800 7635 1800
$Comp
L power:GND #PWR013
U 1 1 5F8183D7
P 5215 5530
F 0 "#PWR013" H 5215 5280 50  0001 C CNN
F 1 "GND" H 5220 5357 50  0000 C CNN
F 2 "" H 5215 5530 50  0001 C CNN
F 3 "" H 5215 5530 50  0001 C CNN
	1    5215 5530
	1    0    0    -1  
$EndComp
Wire Wire Line
	5215 5490 5215 5510
Wire Wire Line
	5315 5490 5315 5510
Wire Wire Line
	5315 5510 5215 5510
Connection ~ 5215 5510
Wire Wire Line
	5215 5510 5215 5530
Wire Wire Line
	5415 5490 5415 5510
Wire Wire Line
	5415 5510 5315 5510
Connection ~ 5315 5510
Wire Wire Line
	5515 5490 5515 5510
Wire Wire Line
	5515 5510 5415 5510
Connection ~ 5415 5510
$Comp
L Device:Crystal_GND24_Small Y1
U 1 1 5F81EA20
P 2260 6850
F 0 "Y1" H 2030 7145 50  0000 R CNN
F 1 "16MHz" H 2140 7070 50  0000 R CNN
F 2 "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm" H 2260 6850 50  0001 C CNN
F 3 "~" H 2260 6850 50  0001 C CNN
F 4 "C13738" H 0   0   50  0001 C CNN "LCSC Part #"
	1    2260 6850
	1    0    0    -1  
$EndComp
Text GLabel 4715 3090 0    50   Input ~ 0
HSE_IN
Text GLabel 4715 3190 0    50   Input ~ 0
HSE_OUT
Wire Wire Line
	2160 6850 2015 6850
Text GLabel 1920 6850 0    50   Input ~ 0
HSE_IN
Wire Wire Line
	2360 6850 2455 6850
Text GLabel 2805 6850 2    50   Input ~ 0
HSE_OUT
Text Notes 2725 7425 2    50   ~ 0
CL = 2 * (Cr - Cparasitic) = 12pF\nCr = 9pF\n
Wire Wire Line
	2015 6900 2015 6850
Connection ~ 2015 6850
Wire Wire Line
	2015 6850 1920 6850
$Comp
L Device:C_Small C3
U 1 1 5F82C8B9
P 2455 7005
F 0 "C3" H 2610 7030 50  0000 L CNN
F 1 "12p" H 2590 6935 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2455 7005 50  0001 C CNN
F 3 "~" H 2455 7005 50  0001 C CNN
F 4 "C1792" H 0   0   50  0001 C CNN "LCSC Part #"
	1    2455 7005
	1    0    0    -1  
$EndComp
Wire Wire Line
	2455 6905 2455 6850
Connection ~ 2455 6850
Wire Wire Line
	2455 6850 2530 6850
Wire Wire Line
	2455 7105 2455 7115
Wire Wire Line
	2455 7115 2365 7115
$Comp
L Device:R_Small R1
U 1 1 5F83264E
P 2630 6850
F 0 "R1" V 2434 6850 50  0000 C CNN
F 1 "390" V 2525 6850 50  0000 C CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 2630 6850 50  0001 C CNN
F 3 "~" H 2630 6850 50  0001 C CNN
F 4 "C17655" H 0   0   50  0001 C CNN "LCSC Part #"
	1    2630 6850
	0    1    1    0   
$EndComp
Wire Wire Line
	2730 6850 2805 6850
Wire Wire Line
	2260 6725 2260 6685
Wire Wire Line
	2260 6685 2365 6685
Wire Wire Line
	2365 6685 2365 7115
Connection ~ 2365 7115
$Comp
L power:GND #PWR02
U 1 1 5F83ABE1
P 1895 4185
F 0 "#PWR02" H 1895 3935 50  0001 C CNN
F 1 "GND" H 1900 4012 50  0000 C CNN
F 2 "" H 1895 4185 50  0001 C CNN
F 3 "" H 1895 4185 50  0001 C CNN
	1    1895 4185
	1    0    0    -1  
$EndComp
$Comp
L Power_Protection:USBLC6-2SC6 U1
U 1 1 5F83C459
P 1975 5130
F 0 "U1" H 1310 5640 50  0000 C CNN
F 1 "USBLC6-2SC6" H 1330 5540 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:SOT-23-6" H 1225 5530 50  0001 C CNN
F 3 "http://www2.st.com/resource/en/datasheet/CD00050750.pdf" H 2175 5480 50  0001 C CNN
F 4 "C7519" H 0   0   50  0001 C CNN "LCSC Part #"
	1    1975 5130
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D2
U 1 1 5F847FF1
P 1995 1440
F 0 "D2" V 2125 1520 50  0000 C CNN
F 1 "B5819W" V 2190 1535 50  0000 C CNN
F 2 "Diode_SMD:D_SOD-123" H 1995 1440 50  0001 C CNN
F 3 "~" H 1995 1440 50  0001 C CNN
F 4 "C8598" H 0   0   50  0001 C CNN "LCSC Part #"
	1    1995 1440
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR05
U 1 1 5F854CB5
P 1975 4630
F 0 "#PWR05" H 1975 4480 50  0001 C CNN
F 1 "+5V" H 2040 4770 50  0000 C CNN
F 2 "" H 1975 4630 50  0001 C CNN
F 3 "" H 1975 4630 50  0001 C CNN
	1    1975 4630
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5F85525D
P 1975 5630
F 0 "#PWR06" H 1975 5380 50  0001 C CNN
F 1 "GND" H 1980 5457 50  0000 C CNN
F 2 "" H 1975 5630 50  0001 C CNN
F 3 "" H 1975 5630 50  0001 C CNN
	1    1975 5630
	1    0    0    -1  
$EndComp
Text GLabel 1450 5230 0    50   Input ~ 0
USB_CONN-
Text GLabel 6015 4890 2    50   Input ~ 0
USB_-
Text GLabel 2475 5030 2    50   Input ~ 0
USB_+
Text GLabel 2475 5230 2    50   Input ~ 0
USB_CONN+
$Comp
L power:+5V #PWR04
U 1 1 5F865054
P 1945 3465
F 0 "#PWR04" H 1945 3315 50  0001 C CNN
F 1 "+5V" H 2030 3575 50  0000 C CNN
F 2 "" H 1945 3465 50  0001 C CNN
F 3 "" H 1945 3465 50  0001 C CNN
	1    1945 3465
	1    0    0    -1  
$EndComp
Text GLabel 8435 2445 3    50   Input ~ 0
I2C_SCL
$Comp
L Device:R_Small R4
U 1 1 5F86DDF0
P 8435 2345
F 0 "R4" H 8494 2391 50  0000 L CNN
F 1 "4.7k" H 8494 2300 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 8435 2345 50  0001 C CNN
F 3 "~" H 8435 2345 50  0001 C CNN
F 4 "C17936" H 0   0   50  0001 C CNN "LCSC Part #"
	1    8435 2345
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR029
U 1 1 5F86DED3
P 8435 2245
F 0 "#PWR029" H 8435 2095 50  0001 C CNN
F 1 "+3.3V" H 8450 2418 50  0000 C CNN
F 2 "" H 8435 2245 50  0001 C CNN
F 3 "" H 8435 2245 50  0001 C CNN
	1    8435 2245
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R3
U 1 1 5F8730A9
P 8195 2345
F 0 "R3" H 8030 2390 50  0000 L CNN
F 1 "4.7k" H 7995 2315 50  0000 L CNN
F 2 "Resistor_SMD:R_1206_3216Metric" H 8195 2345 50  0001 C CNN
F 3 "~" H 8195 2345 50  0001 C CNN
F 4 "C17936" H 0   0   50  0001 C CNN "LCSC Part #"
	1    8195 2345
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR026
U 1 1 5F8730B0
P 8195 2245
F 0 "#PWR026" H 8195 2095 50  0001 C CNN
F 1 "+3.3V" H 8210 2418 50  0000 C CNN
F 2 "" H 8195 2245 50  0001 C CNN
F 3 "" H 8195 2245 50  0001 C CNN
	1    8195 2245
	1    0    0    -1  
$EndComp
Text GLabel 4715 4490 0    50   Input ~ 0
I2C_SDA
$Comp
L Device:R_Small R2
U 1 1 5F87B9D4
P 4615 3790
F 0 "R2" V 4735 3930 50  0000 L CNN
F 1 "33ohm" V 4685 3855 50  0000 L CNN
F 2 "Resistor_SMD:R_0805_2012Metric" H 4615 3790 50  0001 C CNN
F 3 "~" H 4615 3790 50  0001 C CNN
F 4 "C17634" H 0   0   50  0001 C CNN "LCSC Part #"
	1    4615 3790
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4515 3790 3975 3790
Wire Wire Line
	3975 3790 3975 3850
$Comp
L Device:LED_Small D3
U 1 1 5F8825FF
P 3975 3950
F 0 "D3" V 4090 4105 50  0000 R CNN
F 1 "Status" V 4025 4235 50  0000 R CNN
F 2 "LED_SMD:LED_0603_1608Metric" V 3975 3950 50  0001 C CNN
F 3 "~" V 3975 3950 50  0001 C CNN
F 4 "C72041" H 0   0   50  0001 C CNN "LCSC Part #"
	1    3975 3950
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR011
U 1 1 5F882A5C
P 3975 4050
F 0 "#PWR011" H 3975 3800 50  0001 C CNN
F 1 "GND" H 3980 3877 50  0000 C CNN
F 2 "" H 3975 4050 50  0001 C CNN
F 3 "" H 3975 4050 50  0001 C CNN
	1    3975 4050
	1    0    0    -1  
$EndComp
Text GLabel 6015 4190 2    50   Input ~ 0
SPI_NSS
Wire Notes Line
	3270 6490 3270 7535
Wire Notes Line
	1245 7535 1245 6490
Wire Notes Line
	825  3220 825  5935
Wire Notes Line
	825  5935 3255 5935
Wire Notes Line
	3255 5935 3255 3215
Wire Notes Line
	3255 3215 825  3215
Text Notes 1430 6560 2    50   ~ 0
HSE\n\n
Text Notes 1015 3185 2    50   ~ 10
USB\n
$Comp
L Connector:Conn_01x04_Female J4
U 1 1 5F893BB6
P 8725 3250
F 0 "J4" H 8250 3560 50  0000 L CNN
F 1 "SPI_F" H 8160 3475 50  0000 L CNN
F 2 "Connector_PinSocket_1.00mm:PinSocket_1x04_P1.00mm_Vertical" H 8725 3250 50  0001 C CNN
F 3 "~" H 8725 3250 50  0001 C CNN
	1    8725 3250
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x04_Male J6
U 1 1 5F893C5A
P 9105 3265
F 0 "J6" H 9700 3490 50  0000 C CNN
F 1 "SPI_M" H 9785 3415 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x04_P1.00mm_Vertical" H 9105 3265 50  0001 C CNN
F 3 "~" H 9105 3265 50  0001 C CNN
	1    9105 3265
	1    0    0    -1  
$EndComp
Text GLabel 9305 3465 2    50   Input ~ 0
SPI_NSS
Text GLabel 9305 3365 2    50   Input ~ 0
SPI_SCK
Text GLabel 9305 3265 2    50   Input ~ 0
SPI_MISO
Text GLabel 9305 3165 2    50   Input ~ 0
SPI_MOSI
Text GLabel 8525 3150 0    50   Input ~ 0
SPI_NSS
Text GLabel 8525 3250 0    50   Input ~ 0
SPI_SCK
Text GLabel 8525 3350 0    50   Input ~ 0
SPI_MISO
Text GLabel 8525 3450 0    50   Input ~ 0
SPI_MOSI
$Comp
L Connector:Conn_01x02_Male J7
U 1 1 5F8AB3E4
P 9185 3635
F 0 "J7" H 9955 3740 50  0000 C CNN
F 1 "USART_M" H 10035 3640 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x02_P1.00mm_Vertical" H 9185 3635 50  0001 C CNN
F 3 "~" H 9185 3635 50  0001 C CNN
	1    9185 3635
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Female J5
U 1 1 5F8AB532
P 8775 3600
F 0 "J5" H 7990 3645 50  0000 L CNN
F 1 "USART_F" H 7770 3560 50  0000 L CNN
F 2 "Connector_PinSocket_1.00mm:PinSocket_1x02_P1.00mm_Vertical" H 8775 3600 50  0001 C CNN
F 3 "~" H 8775 3600 50  0001 C CNN
	1    8775 3600
	1    0    0    -1  
$EndComp
Text GLabel 9385 3635 2    50   Input ~ 0
USART1_TX
Text GLabel 9385 3735 2    50   Input ~ 0
USART1_RX
Text GLabel 8575 3600 0    50   Input ~ 0
USART1_TX
Text GLabel 8575 3700 0    50   Input ~ 0
USART1_RX
$Comp
L Connector_Generic:Conn_02x05_Odd_Even J2
U 1 1 5F8CBB27
P 8310 4675
F 0 "J2" H 8350 4330 50  0000 C CNN
F 1 "SWD" H 8355 4265 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_2x05_P1.00mm_Vertical" H 8310 4675 50  0001 C CNN
F 3 "~" H 8310 4675 50  0001 C CNN
	1    8310 4675
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR024
U 1 1 5F8CCBE3
P 8110 4475
F 0 "#PWR024" H 8110 4325 50  0001 C CNN
F 1 "+3.3V" H 8125 4648 50  0000 C CNN
F 2 "" H 8110 4475 50  0001 C CNN
F 3 "" H 8110 4475 50  0001 C CNN
	1    8110 4475
	1    0    0    -1  
$EndComp
NoConn ~ 8110 4780
NoConn ~ 8605 4775
$Comp
L power:GND #PWR023
U 1 1 5F8D257D
P 8030 4685
F 0 "#PWR023" H 8030 4435 50  0001 C CNN
F 1 "GND" H 8035 4512 50  0000 C CNN
F 2 "" H 8030 4685 50  0001 C CNN
F 3 "" H 8030 4685 50  0001 C CNN
	1    8030 4685
	1    0    0    -1  
$EndComp
Wire Wire Line
	8110 4675 8030 4675
Wire Wire Line
	8030 4675 8030 4685
Wire Wire Line
	8110 4575 8030 4575
Wire Wire Line
	8030 4575 8030 4675
Connection ~ 8030 4675
Text GLabel 8610 4575 2    50   Input ~ 0
SW_CLK
Text GLabel 8610 4475 2    50   Input ~ 0
SWD_IO
$Comp
L power:GND #PWR030
U 1 1 5F8DE897
P 8165 6115
F 0 "#PWR030" H 8165 5865 50  0001 C CNN
F 1 "GND" H 8170 5942 50  0000 C CNN
F 2 "" H 8165 6115 50  0001 C CNN
F 3 "" H 8165 6115 50  0001 C CNN
	1    8165 6115
	1    0    0    -1  
$EndComp
Text GLabel 8610 4675 2    50   Input ~ 0
SWO
$Comp
L power:VCC #PWR01
U 1 1 5F8EBA2E
P 1175 1155
F 0 "#PWR01" H 1175 1005 50  0001 C CNN
F 1 "VCC" H 1192 1328 50  0000 C CNN
F 2 "" H 1175 1155 50  0001 C CNN
F 3 "" H 1175 1155 50  0001 C CNN
	1    1175 1155
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole_Pad MH2
U 1 1 5F8F51CA
P 8425 5460
F 0 "MH2" V 8335 5720 50  0000 L CNN
F 1 "MountingHole_Pad" V 8420 5655 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 8425 5460 50  0001 C CNN
F 3 "~" H 8425 5460 50  0001 C CNN
	1    8425 5460
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad MH1
U 1 1 5F8FAE69
P 8410 5650
F 0 "MH1" V 8340 5925 50  0000 L CNN
F 1 "MountingHole_Pad" V 8410 5825 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 8410 5650 50  0001 C CNN
F 3 "~" H 8410 5650 50  0001 C CNN
	1    8410 5650
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad MH3
U 1 1 5F8FCC49
P 8435 5840
F 0 "MH3" V 8390 6110 50  0000 L CNN
F 1 "MountingHole_Pad" V 8445 6000 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 8435 5840 50  0001 C CNN
F 3 "~" H 8435 5840 50  0001 C CNN
	1    8435 5840
	0    1    1    0   
$EndComp
$Comp
L Mechanical:MountingHole_Pad MH4
U 1 1 5F8FE9C1
P 8450 6045
F 0 "MH4" V 8405 6275 50  0000 L CNN
F 1 "MountingHole_Pad" V 8490 6195 50  0000 L CNN
F 2 "MountingHole:MountingHole_4.3mm_M4_Pad_Via" H 8450 6045 50  0001 C CNN
F 3 "~" H 8450 6045 50  0001 C CNN
	1    8450 6045
	0    1    1    0   
$EndComp
Wire Wire Line
	8325 5460 8165 5460
Wire Wire Line
	8165 5460 8165 5650
Wire Wire Line
	8335 5840 8165 5840
Connection ~ 8165 5840
Wire Wire Line
	8310 5650 8165 5650
Connection ~ 8165 5650
Wire Wire Line
	8165 5650 8165 5840
Text GLabel 8430 3895 0    50   Input ~ 0
I2C_SDA
Text GLabel 8430 3995 0    50   Input ~ 0
I2C_SCL
$Comp
L Connector:Conn_01x04_Female J3
U 1 1 5F941346
P 8630 3995
F 0 "J3" H 8657 3971 50  0000 L CNN
F 1 "BMP180_I2C" H 8657 3880 50  0000 L CNN
F 2 "Connector_PinSocket_1.00mm:PinSocket_1x04_P1.00mm_Vertical" H 8630 3995 50  0001 C CNN
F 3 "~" H 8630 3995 50  0001 C CNN
	1    8630 3995
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR027
U 1 1 5F945C3D
P 8430 4095
F 0 "#PWR027" H 8430 3845 50  0001 C CNN
F 1 "GND" V 8435 3922 50  0000 C CNN
F 2 "" H 8430 4095 50  0001 C CNN
F 3 "" H 8430 4095 50  0001 C CNN
	1    8430 4095
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR028
U 1 1 5F94C3F1
P 8430 4195
F 0 "#PWR028" H 8430 4045 50  0001 C CNN
F 1 "+3.3V" V 8425 4295 50  0000 L CNN
F 2 "" H 8430 4195 50  0001 C CNN
F 3 "" H 8430 4195 50  0001 C CNN
	1    8430 4195
	0    -1   -1   0   
$EndComp
Text GLabel 4715 4790 0    50   Input ~ 0
USART3_TX
Text GLabel 4715 4890 0    50   Input ~ 0
USART3_RX
$Comp
L Connector:Conn_01x04_Female J8
U 1 1 5F973B33
P 9340 2480
F 0 "J8" H 9367 2456 50  0000 L CNN
F 1 "HC-05 " H 9367 2365 50  0000 L CNN
F 2 "Connector_PinSocket_1.00mm:PinSocket_1x04_P1.00mm_Vertical" H 9340 2480 50  0001 C CNN
F 3 "~" H 9340 2480 50  0001 C CNN
	1    9340 2480
	1    0    0    -1  
$EndComp
Text GLabel 9140 2380 0    50   Input ~ 0
BT-RX
Text GLabel 9140 2480 0    50   Input ~ 0
BT-TX
$Comp
L power:GND #PWR031
U 1 1 5F97C449
P 9140 2580
F 0 "#PWR031" H 9140 2330 50  0001 C CNN
F 1 "GND" V 9145 2407 50  0000 C CNN
F 2 "" H 9140 2580 50  0001 C CNN
F 3 "" H 9140 2580 50  0001 C CNN
	1    9140 2580
	0    1    1    0   
$EndComp
$Comp
L power:+5V #PWR03
U 1 1 5F981763
P 1770 1590
F 0 "#PWR03" H 1770 1440 50  0001 C CNN
F 1 "+5V" H 1835 1730 50  0000 C CNN
F 2 "" H 1770 1590 50  0001 C CNN
F 3 "" H 1770 1590 50  0001 C CNN
	1    1770 1590
	1    0    0    -1  
$EndComp
Wire Wire Line
	1770 1590 1995 1590
$Comp
L power:+5V #PWR032
U 1 1 5F9863DC
P 9140 2680
F 0 "#PWR032" H 9140 2530 50  0001 C CNN
F 1 "+5V" V 9080 2810 50  0000 C CNN
F 2 "" H 9140 2680 50  0001 C CNN
F 3 "" H 9140 2680 50  0001 C CNN
	1    9140 2680
	0    -1   -1   0   
$EndComp
Text GLabel 10190 2225 0    50   Input ~ 0
USART3_TX
$Comp
L Device:R_Small R6
U 1 1 5F9941F1
P 10290 2225
F 0 "R6" V 10450 2175 50  0000 L CNN
F 1 "1k" V 10365 2130 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 10290 2225 50  0001 C CNN
F 3 "~" H 10290 2225 50  0001 C CNN
F 4 "C11702" H 0   0   50  0001 C CNN "LCSC Part #"
	1    10290 2225
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R7
U 1 1 5F99B551
P 10430 2330
F 0 "R7" H 10265 2375 50  0000 L CNN
F 1 "2.2k" H 10230 2300 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 10430 2330 50  0001 C CNN
F 3 "~" H 10430 2330 50  0001 C CNN
F 4 "C25879" H 0   0   50  0001 C CNN "LCSC Part #"
	1    10430 2330
	-1   0    0    1   
$EndComp
Wire Wire Line
	10390 2225 10430 2225
Wire Wire Line
	10430 2225 10430 2230
$Comp
L power:GND #PWR037
U 1 1 5F9A6E61
P 10430 2430
F 0 "#PWR037" H 10430 2180 50  0001 C CNN
F 1 "GND" H 10435 2257 50  0000 C CNN
F 2 "" H 10430 2430 50  0001 C CNN
F 3 "" H 10430 2430 50  0001 C CNN
	1    10430 2430
	1    0    0    -1  
$EndComp
Text GLabel 10680 2230 2    50   Input ~ 0
BT-RX
Wire Wire Line
	10680 2230 10430 2230
Connection ~ 10430 2230
Text GLabel 10700 2710 2    50   Input ~ 0
BT-TX
Text GLabel 10225 2710 0    50   Input ~ 0
USART3_RX
Wire Wire Line
	10225 2710 10700 2710
Wire Notes Line
	7750 6410 11080 6410
Wire Notes Line
	11080 6410 11080 1980
Wire Notes Line
	11080 1980 7750 1980
Wire Notes Line
	7750 1980 7750 6410
Text Notes 7750 1950 0    50   ~ 10
Connectors
Wire Notes Line
	3630 2195 3635 2195
Wire Notes Line
	3630 2180 3630 5800
Text Notes 3625 2160 0    50   ~ 10
MCU\n
Wire Wire Line
	8350 6045 8165 6045
Wire Wire Line
	8165 5840 8165 6045
Connection ~ 8165 6045
Wire Wire Line
	8165 6045 8165 6115
NoConn ~ 6015 3790
NoConn ~ 6010 3890
NoConn ~ 6010 3995
NoConn ~ 6010 4095
NoConn ~ 6015 4590
NoConn ~ 6020 5290
NoConn ~ 4720 5290
NoConn ~ 4720 5190
NoConn ~ 4715 5100
NoConn ~ 4715 4985
NoConn ~ 4720 4685
NoConn ~ 4715 4585
NoConn ~ 4715 4290
NoConn ~ 4720 4200
NoConn ~ 8115 4875
Text GLabel 4715 3990 0    50   Input ~ 0
BOOT1
Text GLabel 4715 2890 0    50   Input ~ 0
BOOT0
Text GLabel 4715 3990 0    50   Input ~ 0
BOOT1
$Comp
L Switch:SW_Push_Dual SW3
U 1 1 5F858DEF
P 6755 3465
F 0 "SW3" H 6755 3750 50  0000 C CNN
F 1 "SW_Push_Dual" H 6755 3659 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm" H 6755 3665 50  0001 C CNN
F 3 "" H 6755 3665 50  0001 C CNN
	1    6755 3465
	1    0    0    -1  
$EndComp
Text GLabel 4715 2690 0    50   Input ~ 0
NRST
Text GLabel 6415 3540 0    50   Input ~ 0
NRST
Wire Wire Line
	6415 3540 6520 3540
Wire Wire Line
	6520 3540 6520 3465
Wire Wire Line
	6520 3465 6555 3465
Wire Wire Line
	6555 3665 6520 3665
Wire Wire Line
	6520 3665 6520 3540
Connection ~ 6520 3540
$Comp
L power:GND #PWR022
U 1 1 5F86DEBF
P 6985 3770
F 0 "#PWR022" H 6985 3520 50  0001 C CNN
F 1 "GND" H 6990 3597 50  0000 C CNN
F 2 "" H 6985 3770 50  0001 C CNN
F 3 "" H 6985 3770 50  0001 C CNN
	1    6985 3770
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C13
U 1 1 5F87E177
P 6760 3750
F 0 "C13" V 6890 3750 50  0000 C CNN
F 1 "100n" V 6950 3705 50  0000 C CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6760 3750 50  0001 C CNN
F 3 "~" H 6760 3750 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    6760 3750
	0    1    1    0   
$EndComp
Wire Wire Line
	6985 3770 6985 3750
Wire Wire Line
	6985 3665 6955 3665
Wire Wire Line
	6985 3665 6985 3465
Wire Wire Line
	6985 3465 6955 3465
Connection ~ 6985 3665
Wire Wire Line
	6860 3750 6985 3750
Connection ~ 6985 3750
Wire Wire Line
	6985 3750 6985 3665
Wire Wire Line
	6660 3750 6520 3750
Wire Wire Line
	6520 3750 6520 3665
Connection ~ 6520 3665
NoConn ~ 4715 3390
NoConn ~ 4715 3890
Text Notes 5040 7430 2    50   ~ 0
CL = 2 * (Cr - Cparasitic) = 22p\nCr = 12.5p
Text Notes 3470 6565 2    50   ~ 0
LSE\n\n
Wire Notes Line
	5390 6490 5390 7535
Wire Notes Line
	1245 6490 5390 6490
Wire Notes Line
	1245 7535 5390 7535
Text GLabel 4715 3490 0    50   Input ~ 0
LSE_IN
Text GLabel 4715 3590 0    50   Input ~ 0
LSE_OUT
Wire Notes Line
	7425 5800 7425 2180
Wire Notes Line
	3630 5800 7425 5800
Wire Notes Line
	3630 2180 7425 2180
Connection ~ 1995 1290
$Comp
L Device:CP1_Small C2
U 1 1 5F945646
P 2150 1410
F 0 "C2" H 2241 1456 50  0000 L CNN
F 1 "10u" H 2241 1365 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2150 1410 50  0001 C CNN
F 3 "~" H 2150 1410 50  0001 C CNN
F 4 "C13585" H 0   0   50  0001 C CNN "LCSC Part #"
	1    2150 1410
	1    0    0    -1  
$EndComp
Wire Wire Line
	2150 1310 2150 1290
Wire Wire Line
	2150 1290 1995 1290
Wire Wire Line
	2150 1510 2150 1590
$Comp
L Device:C_Small C5
U 1 1 5F95D739
P 3735 1385
F 0 "C5" H 3575 1430 50  0000 L CNN
F 1 "10u" H 3530 1325 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 3735 1385 50  0001 C CNN
F 3 "~" H 3735 1385 50  0001 C CNN
F 4 "C15525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    3735 1385
	1    0    0    -1  
$EndComp
Wire Wire Line
	1710 1290 1995 1290
$Comp
L Device:Ferrite_Bead_Small L1
U 1 1 5F9703F7
P 8460 1235
F 0 "L1" V 8540 1140 50  0000 C CNN
F 1 "Ferrite_Bead_Small" V 8735 1065 50  0000 C CNN
F 2 "Inductor_SMD:L_0805_2012Metric" V 8390 1235 50  0001 C CNN
F 3 "~" H 8460 1235 50  0001 C CNN
F 4 "C1017" H 0   0   50  0001 C CNN "LCSC Part #"
	1    8460 1235
	0    1    1    0   
$EndComp
$Comp
L Connector:USB_B_Micro J1
U 1 1 5F984833
P 1555 3760
F 0 "J1" H 1610 4227 50  0000 C CNN
F 1 "USB_B_Micro" H 1610 4136 50  0000 C CNN
F 2 "Connector_USB:USB_Micro-B_Molex-105017-0001" H 1705 3710 50  0001 C CNN
F 3 "~" H 1705 3710 50  0001 C CNN
	1    1555 3760
	1    0    0    -1  
$EndComp
Wire Wire Line
	1855 3560 1945 3560
Wire Wire Line
	1945 3560 1945 3465
NoConn ~ 1455 4160
Wire Wire Line
	1855 3960 1895 3960
Wire Wire Line
	1555 4160 1895 4160
Wire Wire Line
	1895 3960 1895 4160
Connection ~ 1895 4160
Wire Wire Line
	1895 4160 1895 4185
$Comp
L Switch:SW_DIP_x02 SW1
U 1 1 5F87266D
P 6735 2680
F 0 "SW1" H 6735 3047 50  0000 C CNN
F 1 "BOOT0_BOOT1" H 6735 2956 50  0000 C CNN
F 2 "Button_Switch_THT:SW_DIP_SPSTx02_Slide_6.7x6.64mm_W7.62mm_P2.54mm_LowProfile" H 6735 2680 50  0001 C CNN
F 3 "" H 6735 2680 50  0001 C CNN
	1    6735 2680
	1    0    0    -1  
$EndComp
Text GLabel 6320 2580 0    50   Input ~ 0
BOOT0
Wire Wire Line
	6320 2580 6345 2580
Text GLabel 6320 2680 0    50   Input ~ 0
BOOT1
Wire Wire Line
	6435 2680 6420 2680
$Comp
L Device:R_Small R8
U 1 1 5F8B447A
P 6345 2825
F 0 "R8" H 6195 2875 50  0000 L CNN
F 1 "1k" H 6165 2800 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 6345 2825 50  0001 C CNN
F 3 "~" H 6345 2825 50  0001 C CNN
F 4 "C11702" H 0   0   50  0001 C CNN "LCSC Part #"
	1    6345 2825
	1    0    0    -1  
$EndComp
Wire Wire Line
	6345 2725 6345 2580
Connection ~ 6345 2580
Wire Wire Line
	6345 2580 6435 2580
$Comp
L Device:R_Small R9
U 1 1 5F8BF284
P 6420 2825
F 0 "R9" H 6465 2865 50  0000 L CNN
F 1 "1k" H 6470 2790 50  0000 L CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 6420 2825 50  0001 C CNN
F 3 "~" H 6420 2825 50  0001 C CNN
F 4 "C11702" H 0   0   50  0001 C CNN "LCSC Part #"
	1    6420 2825
	1    0    0    -1  
$EndComp
Wire Wire Line
	6420 2725 6420 2680
Connection ~ 6420 2680
Wire Wire Line
	6420 2680 6320 2680
$Comp
L power:+3.3V #PWR0101
U 1 1 5F8CB285
P 7290 2465
F 0 "#PWR0101" H 7290 2315 50  0001 C CNN
F 1 "+3.3V" H 7305 2638 50  0000 C CNN
F 2 "" H 7290 2465 50  0001 C CNN
F 3 "" H 7290 2465 50  0001 C CNN
	1    7290 2465
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R10
U 1 1 5F8E7EAE
P 7135 2580
F 0 "R10" V 6939 2580 50  0000 C CNN
F 1 "1k" V 7030 2580 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7135 2580 50  0001 C CNN
F 3 "~" H 7135 2580 50  0001 C CNN
F 4 "C11702" H 0   0   50  0001 C CNN "LCSC Part #"
	1    7135 2580
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R11
U 1 1 5F8ED893
P 7135 2680
F 0 "R11" V 7215 2680 50  0000 C CNN
F 1 "1k" V 7280 2685 50  0000 C CNN
F 2 "Resistor_SMD:R_0402_1005Metric" H 7135 2680 50  0001 C CNN
F 3 "~" H 7135 2680 50  0001 C CNN
F 4 "C11702" H 0   0   50  0001 C CNN "LCSC Part #"
	1    7135 2680
	0    1    1    0   
$EndComp
Wire Wire Line
	7235 2580 7290 2580
Wire Wire Line
	7290 2580 7290 2465
Wire Wire Line
	7235 2680 7290 2680
Wire Wire Line
	7290 2680 7290 2580
Connection ~ 7290 2580
$Comp
L power:GND #PWR0102
U 1 1 5F8FFC16
P 6380 2950
F 0 "#PWR0102" H 6380 2700 50  0001 C CNN
F 1 "GND" H 6385 2777 50  0000 C CNN
F 2 "" H 6380 2950 50  0001 C CNN
F 3 "" H 6380 2950 50  0001 C CNN
	1    6380 2950
	1    0    0    -1  
$EndComp
Wire Wire Line
	6345 2925 6345 2950
Wire Wire Line
	6345 2950 6380 2950
Wire Wire Line
	6420 2925 6420 2950
Wire Wire Line
	6420 2950 6380 2950
Connection ~ 6380 2950
$Comp
L Device:Crystal Y2
U 1 1 5F92949B
P 4525 6830
F 0 "Y2" H 4525 7098 50  0000 C CNN
F 1 "32.768kHz" H 4525 7007 50  0000 C CNN
F 2 "Crystal:Crystal_SMD_3215-2Pin_3.2x1.5mm" H 4525 6830 50  0001 C CNN
F 3 "~" H 4525 6830 50  0001 C CNN
F 4 "C32346" H 0   0   50  0001 C CNN "LCSC Part #"
	1    4525 6830
	1    0    0    -1  
$EndComp
Text GLabel 4240 6830 0    50   Input ~ 0
LSE_IN
Text GLabel 4790 6830 2    50   Input ~ 0
LSE_OUT
$Comp
L Device:C_Small C7
U 1 1 5F95A927
P 4335 6935
F 0 "C7" H 4125 6920 50  0000 L CNN
F 1 "22p" H 4105 6845 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4335 6935 50  0001 C CNN
F 3 "~" H 4335 6935 50  0001 C CNN
F 4 "C1804" H 0   0   50  0001 C CNN "LCSC Part #"
	1    4335 6935
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR016
U 1 1 5F960AC6
P 2260 7115
F 0 "#PWR016" H 2260 6865 50  0001 C CNN
F 1 "GND" H 2390 7020 50  0000 C CNN
F 2 "" H 2260 7115 50  0001 C CNN
F 3 "" H 2260 7115 50  0001 C CNN
	1    2260 7115
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C1
U 1 1 5F98AB0B
P 2015 7000
F 0 "C1" H 1845 7030 50  0000 L CNN
F 1 "12p" H 1800 6960 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 2015 7000 50  0001 C CNN
F 3 "~" H 2015 7000 50  0001 C CNN
F 4 "C1792" H 0   0   50  0001 C CNN "LCSC Part #"
	1    2015 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2015 7100 2015 7115
$Comp
L power:GND #PWR017
U 1 1 5F99B389
P 4525 7055
F 0 "#PWR017" H 4525 6805 50  0001 C CNN
F 1 "GND" H 4530 6882 50  0000 C CNN
F 2 "" H 4525 7055 50  0001 C CNN
F 3 "" H 4525 7055 50  0001 C CNN
	1    4525 7055
	1    0    0    -1  
$EndComp
Wire Wire Line
	4525 7045 4525 7055
Connection ~ 4525 7045
Wire Wire Line
	4335 6835 4335 6830
Wire Wire Line
	4335 6830 4375 6830
Wire Wire Line
	4335 7035 4335 7045
Wire Wire Line
	4335 7045 4525 7045
$Comp
L Device:C_Small C9
U 1 1 5F9DFE6C
P 4720 6940
F 0 "C9" H 4825 6930 50  0000 L CNN
F 1 "22p" H 4805 6840 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 4720 6940 50  0001 C CNN
F 3 "~" H 4720 6940 50  0001 C CNN
F 4 "C1804" H 0   0   50  0001 C CNN "LCSC Part #"
	1    4720 6940
	1    0    0    -1  
$EndComp
Wire Wire Line
	4720 6840 4720 6830
Wire Wire Line
	4720 6830 4675 6830
Wire Wire Line
	4720 7040 4720 7045
Wire Wire Line
	4525 7045 4720 7045
Wire Wire Line
	4240 6830 4335 6830
Connection ~ 4335 6830
Wire Wire Line
	4790 6830 4720 6830
Connection ~ 4720 6830
Text GLabel 2465 3600 0    50   Input ~ 0
USB_DP
Wire Wire Line
	2465 3600 2535 3600
$Comp
L Device:R_Small R12
U 1 1 5F870CA8
P 2535 3500
F 0 "R12" H 2575 3535 50  0000 L CNN
F 1 "1.5k" H 2560 3465 50  0000 L CNN
F 2 "Resistor_SMD:R_0201_0603Metric" H 2535 3500 50  0001 C CNN
F 3 "~" H 2535 3500 50  0001 C CNN
F 4 "C22483" H 0   0   50  0001 C CNN "LCSC Part #"
	1    2535 3500
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0103
U 1 1 5F87682B
P 2535 3400
F 0 "#PWR0103" H 2535 3250 50  0001 C CNN
F 1 "+3.3V" H 2640 3520 50  0000 C CNN
F 2 "" H 2535 3400 50  0001 C CNN
F 3 "" H 2535 3400 50  0001 C CNN
	1    2535 3400
	1    0    0    -1  
$EndComp
Text GLabel 8610 4875 2    50   Input ~ 0
NRST
Wire Wire Line
	1450 5230 1475 5230
Wire Wire Line
	2015 7115 2260 7115
Connection ~ 2260 7115
Wire Wire Line
	2260 7115 2365 7115
Wire Wire Line
	2260 6975 2260 7115
$Comp
L Device:C_Small C14
U 1 1 5F811B43
P 8750 1335
F 0 "C14" H 8530 1360 50  0000 L CNN
F 1 "100n" H 8520 1280 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 8750 1335 50  0001 C CNN
F 3 "~" H 8750 1335 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    8750 1335
	1    0    0    -1  
$EndComp
Connection ~ 8750 1235
Wire Wire Line
	8560 1235 8750 1235
$Comp
L Device:C_Small C15
U 1 1 5F9581DC
P 8985 1335
F 0 "C15" H 9065 1385 50  0000 L CNN
F 1 "1u" H 9045 1300 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 8985 1335 50  0001 C CNN
F 3 "~" H 8985 1335 50  0001 C CNN
F 4 "C15849" H 0   0   50  0001 C CNN "LCSC Part #"
	1    8985 1335
	1    0    0    -1  
$EndComp
Wire Wire Line
	8750 1435 8865 1435
Wire Wire Line
	8865 1490 8865 1435
Connection ~ 8865 1435
Wire Wire Line
	8865 1435 8985 1435
Wire Wire Line
	8750 1235 8945 1235
Connection ~ 8945 1235
Wire Wire Line
	8945 1235 8985 1235
Wire Wire Line
	5615 2490 5615 2480
Wire Wire Line
	2330 1290 2150 1290
Connection ~ 2150 1290
Wire Wire Line
	1175 1155 1175 1290
$Comp
L Device:C_Small C16
U 1 1 5F9F0E7A
P 6460 1390
F 0 "C16" H 6552 1436 50  0000 L CNN
F 1 "100n" H 6552 1345 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6460 1390 50  0001 C CNN
F 3 "~" H 6460 1390 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    6460 1390
	1    0    0    -1  
$EndComp
Wire Wire Line
	6460 1490 6460 1540
Wire Wire Line
	6460 1540 6070 1540
Connection ~ 6070 1540
Wire Wire Line
	6460 1290 6460 1230
Wire Wire Line
	6460 1230 6070 1230
Connection ~ 6070 1230
$Comp
L Device:C_Small C17
U 1 1 5FA07138
P 6820 1385
F 0 "C17" H 6912 1431 50  0000 L CNN
F 1 "100n" H 6912 1340 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 6820 1385 50  0001 C CNN
F 3 "~" H 6820 1385 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    6820 1385
	1    0    0    -1  
$EndComp
Wire Wire Line
	6820 1285 6820 1230
Wire Wire Line
	6820 1230 6460 1230
Connection ~ 6460 1230
Wire Wire Line
	6820 1485 6820 1540
Wire Wire Line
	6820 1540 6460 1540
Connection ~ 6460 1540
$Comp
L Device:C_Small C18
U 1 1 5FA1ED15
P 7200 1375
F 0 "C18" H 7292 1421 50  0000 L CNN
F 1 "100n" H 7292 1330 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 7200 1375 50  0001 C CNN
F 3 "~" H 7200 1375 50  0001 C CNN
F 4 "C1525" H 0   0   50  0001 C CNN "LCSC Part #"
	1    7200 1375
	1    0    0    -1  
$EndComp
Wire Wire Line
	7200 1275 7200 1230
Wire Wire Line
	7200 1230 6820 1230
Connection ~ 6820 1230
Wire Wire Line
	7200 1475 7200 1540
Wire Wire Line
	7200 1540 6820 1540
Connection ~ 6820 1540
Wire Notes Line
	7640 1800 9285 1800
Wire Notes Line
	9285 1800 9285 980 
Wire Notes Line
	1100 980  9285 980 
$Comp
L Connector:Conn_01x02_Male J9
U 1 1 5FA55922
P 9950 4290
F 0 "J9" H 10056 4468 50  0000 C CNN
F 1 "VCC_GND" H 10056 4377 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x02_P1.00mm_Vertical" H 9950 4290 50  0001 C CNN
F 3 "~" H 9950 4290 50  0001 C CNN
	1    9950 4290
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR018
U 1 1 5FA5630D
P 10250 4275
F 0 "#PWR018" H 10250 4125 50  0001 C CNN
F 1 "VCC" H 10267 4448 50  0000 C CNN
F 2 "" H 10250 4275 50  0001 C CNN
F 3 "" H 10250 4275 50  0001 C CNN
	1    10250 4275
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 4275 10250 4290
Wire Wire Line
	10250 4290 10150 4290
$Comp
L power:GND #PWR019
U 1 1 5FA69F37
P 10250 4390
F 0 "#PWR019" H 10250 4140 50  0001 C CNN
F 1 "GND" H 10255 4217 50  0000 C CNN
F 2 "" H 10250 4390 50  0001 C CNN
F 3 "" H 10250 4390 50  0001 C CNN
	1    10250 4390
	1    0    0    -1  
$EndComp
Wire Wire Line
	10250 4390 10150 4390
$Comp
L Connector:Conn_01x02_Male J11
U 1 1 5FA77335
P 10010 4850
F 0 "J11" H 10116 5028 50  0000 C CNN
F 1 "5V_GND" H 10116 4937 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x02_P1.00mm_Vertical" H 10010 4850 50  0001 C CNN
F 3 "~" H 10010 4850 50  0001 C CNN
	1    10010 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	10310 4835 10310 4850
Wire Wire Line
	10310 4850 10210 4850
$Comp
L power:GND #PWR035
U 1 1 5FA77344
P 10310 4950
F 0 "#PWR035" H 10310 4700 50  0001 C CNN
F 1 "GND" H 10315 4777 50  0000 C CNN
F 2 "" H 10310 4950 50  0001 C CNN
F 3 "" H 10310 4950 50  0001 C CNN
	1    10310 4950
	1    0    0    -1  
$EndComp
Wire Wire Line
	10310 4950 10210 4950
$Comp
L power:+5V #PWR034
U 1 1 5FA8572A
P 10310 4835
F 0 "#PWR034" H 10310 4685 50  0001 C CNN
F 1 "+5V" H 10375 4975 50  0000 C CNN
F 2 "" H 10310 4835 50  0001 C CNN
F 3 "" H 10310 4835 50  0001 C CNN
	1    10310 4835
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_01x02_Male J10
U 1 1 5FA8C589
P 9975 5475
F 0 "J10" H 10081 5653 50  0000 C CNN
F 1 "3V3_GND" H 10081 5562 50  0000 C CNN
F 2 "Connector_PinHeader_1.00mm:PinHeader_1x02_P1.00mm_Vertical" H 9975 5475 50  0001 C CNN
F 3 "~" H 9975 5475 50  0001 C CNN
	1    9975 5475
	1    0    0    -1  
$EndComp
Wire Wire Line
	10275 5460 10275 5475
Wire Wire Line
	10275 5475 10175 5475
$Comp
L power:GND #PWR033
U 1 1 5FA8C592
P 10275 5575
F 0 "#PWR033" H 10275 5325 50  0001 C CNN
F 1 "GND" H 10280 5402 50  0000 C CNN
F 2 "" H 10275 5575 50  0001 C CNN
F 3 "" H 10275 5575 50  0001 C CNN
	1    10275 5575
	1    0    0    -1  
$EndComp
Wire Wire Line
	10275 5575 10175 5575
$Comp
L power:+3.3V #PWR025
U 1 1 5FA93B4F
P 10275 5460
F 0 "#PWR025" H 10275 5310 50  0001 C CNN
F 1 "+3.3V" H 10290 5633 50  0000 C CNN
F 2 "" H 10275 5460 50  0001 C CNN
F 3 "" H 10275 5460 50  0001 C CNN
	1    10275 5460
	1    0    0    -1  
$EndComp
Text GLabel 1855 3860 2    50   Input ~ 0
USB_CONN-
Text GLabel 1855 3760 2    50   Input ~ 0
USB_CONN+
$EndSCHEMATC
