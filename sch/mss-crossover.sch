v 20130925 2
T 42300 59600 9 10 1 0 0 6 3
Free-mo
Accessory
Bus (~16VAC)
N 46800 59000 46800 58800 4
N 46800 59900 46800 60100 4
N 48800 60100 50900 60100 4
T 63900 43100 9 10 1 0 0 0 1
Free-mo MSS-Compatible Integrated Crossover Module
T 63700 42800 9 10 1 0 0 0 1
mss-crossover.sch
T 63900 42500 9 10 1 0 0 0 1
1
T 65400 42500 9 10 1 0 0 0 1
1
T 67600 42500 9 10 1 0 0 0 1
Nathan D. Holmes
T 67600 42800 9 10 1 0 0 0 1
$Revision: 82 $
T 58700 47700 9 10 1 0 0 2 3
Notes:
1) All caps X5R or X7R, 6.3V or better ceramic unless otherwise noted.

C 43100 59500 1 0 1 termblk2-1.sym
{
T 42100 60150 5 10 0 0 0 6 1
device=TERMBLK2
T 42700 60400 5 10 1 1 0 6 1
refdes=J3
T 43100 59500 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 59400 48000 1 0 0 hole-1.sym
{
T 59400 48000 5 10 0 1 0 0 1
device=HOLE
T 59600 48600 5 10 1 1 0 4 1
refdes=H1
T 59400 48000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 59900 48000 1 0 0 hole-1.sym
{
T 59900 48000 5 10 0 1 0 0 1
device=HOLE
T 60100 48600 5 10 1 1 0 4 1
refdes=H2
T 59900 48000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 60400 48000 1 0 0 hole-1.sym
{
T 60400 48000 5 10 0 1 0 0 1
device=HOLE
T 60600 48600 5 10 1 1 0 4 1
refdes=H3
T 60400 48000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
C 60900 48000 1 0 0 hole-1.sym
{
T 60900 48000 5 10 0 1 0 0 1
device=HOLE
T 61100 48600 5 10 1 1 0 4 1
refdes=H4
T 60900 48000 5 10 0 0 0 0 1
footprint=STANDOFF_HEX_n4
}
N 45100 60100 47200 60100 4
N 48000 58800 48000 59500 4
C 48900 59900 1 270 0 capacitor-1.sym
{
T 49600 59700 5 10 0 1 270 0 1
device=CAPACITOR
T 49200 59600 5 10 1 1 0 0 1
refdes=C2
T 49800 59700 5 10 0 0 270 0 1
symversion=0.1
T 49200 59100 5 10 1 1 0 0 1
value=1uF
T 48900 59900 5 10 0 0 0 0 1
footprint=0805
}
N 49100 59900 49100 60100 4
N 49100 59000 49100 58800 4
C 41500 52500 1 0 0 current-transformer.sym
{
T 41900 53900 5 10 1 1 0 0 1
refdes=T1
T 41800 53800 5 10 0 0 0 0 1
device=transformer
T 41500 52500 5 10 0 0 0 0 1
footprint=CST306
}
N 42900 53700 46100 53700 4
{
T 45400 54100 5 10 1 1 0 0 1
netname=VBIAS
}
N 44700 53600 44700 53700 4
N 43300 53700 43300 53600 4
N 42900 52700 44900 52700 4
C 50700 60500 1 0 0 5V-plus-1.sym
N 50900 60500 50900 60100 4
C 53400 48100 1 0 0 avrprog-1.sym
{
T 53400 49700 5 10 0 1 0 0 1
device=AVRPROG
T 54000 49400 5 10 1 1 0 0 1
refdes=J4
T 53400 48100 5 10 0 0 0 0 1
footprint=TC2030-NL
}
N 54800 49100 55100 49100 4
N 55100 49100 55100 49600 4
C 54700 47900 1 0 0 gnd-1.sym
N 54800 48300 54800 48200 4
C 46600 59900 1 270 0 Cap_H-2.sym
{
T 46900 59600 5 10 1 1 0 0 1
refdes=C1
T 48100 59900 5 10 0 0 270 0 1
device=Capacitor
T 46100 59600 5 10 1 1 0 2 1
value=68uF
T 46600 59900 5 10 0 1 0 0 1
footprint=cap-elec-Panasonic-FK--D6.30-H5.80-mm
T 45500 59100 5 10 1 1 0 0 1
description=35V Electrolytic
}
C 44800 52700 1 90 0 resistor-1.sym
{
T 44400 53000 5 10 0 0 90 0 1
device=RESISTOR
T 44500 53100 5 10 1 1 90 0 1
refdes=R3
T 45000 53100 5 10 1 1 90 0 1
value=1k
T 44800 52700 5 10 0 0 90 0 1
footprint=0805
}
N 45000 53700 45000 54200 4
N 45000 54200 45300 54200 4
N 45800 52700 46800 52700 4
N 46100 52700 46100 53300 4
N 47700 52700 48400 52700 4
N 48400 52700 48400 53500 4
N 48400 53500 49000 53500 4
C 49400 52600 1 0 0 gnd-1.sym
N 49000 53100 49000 51900 4
N 50200 51900 50200 53300 4
N 47900 51900 47700 51900 4
{
T 47100 51800 5 10 1 1 0 0 1
netname=VBIAS
}
N 51200 53300 52300 53300 4
{
T 52400 53200 5 10 1 1 0 0 1
netname=IDET1
}
C 51700 52100 1 0 0 gnd-1.sym
C 49300 53700 1 0 0 5V-plus-1.sym
C 46200 52600 1 0 1 res-pack4-1.sym
{
T 46200 52600 5 10 0 0 0 6 1
slot=1
T 45400 52400 5 10 1 1 0 6 1
value=10k
T 46000 52200 5 10 1 1 0 6 1
footprint=RPACK4-1206
T 45700 52400 5 10 1 1 0 6 1
refdes=R4
}
C 46400 52600 1 0 0 res-pack4-1.sym
{
T 46400 52600 5 10 0 0 0 0 1
slot=2
T 47200 52400 5 10 1 1 0 0 1
value=10k
T 46600 52200 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 46900 52400 5 10 1 1 0 0 1
refdes=R4
}
C 46400 55200 1 0 0 5V-plus-1.sym
C 46500 52800 1 0 0 gnd-1.sym
N 50300 53300 50000 53300 4
N 48800 51900 49200 51900 4
N 50100 51900 50200 51900 4
C 42900 53600 1 270 0 mmbd4448dw-1.sym
{
T 43500 53200 5 10 0 0 270 0 1
device=MMBD4448DW
T 43200 52400 5 10 1 1 0 0 1
refdes=D4
T 42898 53605 5 10 0 1 270 0 1
footprint=SOT363
T 42900 53600 5 10 0 0 0 0 1
slot=1
}
C 43500 52700 1 270 1 mmbd4448dw-1.sym
{
T 44300 53900 5 10 1 1 180 2 1
device=MMBD4448DW
T 43800 52600 5 10 1 1 180 6 1
refdes=D4
T 43498 52695 5 10 0 1 90 2 1
footprint=SOT363
T 43500 52700 5 10 0 0 180 6 1
slot=2
}
N 43900 53600 43900 53700 4
C 47500 53100 1 0 0 mmbd4448dw-1.sym
{
T 47900 53700 5 10 0 0 0 0 1
device=MMBD4448DW
T 47900 54000 5 10 1 1 0 0 1
refdes=D5
T 47495 53098 5 10 0 1 0 0 1
footprint=SOT363
T 47500 53100 5 10 0 0 0 0 1
slot=1
}
N 47500 53500 47100 53500 4
C 49900 53200 1 0 0 res-pack4-1.sym
{
T 49900 53200 5 10 0 0 0 0 1
slot=3
T 50800 53800 5 10 1 1 0 0 1
value=10k
T 50200 53600 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 50500 53800 5 10 1 1 0 0 1
refdes=R4
}
C 64700 58500 1 0 0 gnd-1.sym
N 64100 58800 64800 58800 4
N 64800 58800 64800 59200 4
N 63200 59700 64300 59700 4
{
T 63200 59800 5 10 1 1 0 0 1
netname=ISET1
}
C 64600 60300 1 0 0 5V-plus-1.sym
N 64800 60300 64800 60100 4
C 54900 49600 1 0 0 5V-plus-1.sym
N 52900 49100 53400 49100 4
{
T 52300 49000 5 10 1 1 0 0 1
netname=MISO
}
N 52900 48700 53400 48700 4
{
T 52400 48600 5 10 1 1 0 0 1
netname=SCK
}
N 55200 48700 54800 48700 4
{
T 55300 48700 5 10 1 1 0 0 1
netname=MOSI
}
N 53400 48300 52900 48300 4
{
T 52800 48200 5 10 1 1 0 6 1
netname=\_RESET\_
}
C 46900 55000 1 270 0 capacitor-1.sym
{
T 47600 54800 5 10 0 1 270 0 1
device=CAPACITOR
T 47200 54700 5 10 1 1 0 0 1
refdes=C4
T 47800 54800 5 10 0 0 270 0 1
symversion=0.1
T 47200 54200 5 10 1 1 0 0 1
value=0.1uF
T 46900 55000 5 10 0 0 0 0 1
footprint=0805
}
N 46600 55200 46600 53900 4
N 46600 55000 47100 55000 4
C 47000 53800 1 0 0 gnd-1.sym
C 64900 59200 1 90 0 pot-1.sym
{
T 64000 60000 5 10 0 0 90 0 1
device=VARIABLE_RESISTOR
T 65300 59900 5 10 1 1 180 0 1
refdes=R7
T 63400 60000 5 10 0 0 90 0 1
footprint=bourns3266
T 65000 59500 5 10 1 1 0 0 1
value=100k
}
C 42200 47300 1 0 0 rj45-1.sym
{
T 42200 50200 5 10 0 0 0 0 1
device=RJ45
T 41600 47000 5 10 0 1 0 0 1
footprint=modular_8p8c_lp.fp
T 42200 49200 5 10 1 1 0 0 1
refdes=J1
T 42000 47000 5 10 1 1 0 0 1
description=MSS Bus A End
}
C 49400 47500 1 0 1 rj45-1.sym
{
T 49400 50400 5 10 0 0 0 6 1
device=RJ45
T 50000 47200 5 10 0 1 0 6 1
footprint=modular_8p8c_lp.fp
T 49400 49400 5 10 1 1 0 6 1
refdes=J2
T 49600 47100 5 10 1 1 0 6 1
description=MSS Bus B End
}
N 43100 48500 46300 48500 4
N 46300 48500 46300 49100 4
N 43100 48300 46500 48300 4
N 43100 48100 44400 48100 4
N 47100 48300 48500 48300 4
N 43100 47700 47500 47700 4
N 47700 47500 47700 47700 4
N 47700 47500 43100 47500 4
N 43100 47900 47300 47900 4
N 47300 48900 47300 47900 4
C 44300 45600 1 0 0 gnd-1.sym
N 44400 45900 44400 48100 4
C 43900 60300 1 180 1 bridge-2.sym
{
T 44100 59300 5 10 1 1 180 6 1
refdes=U1
T 44100 59100 5 10 1 1 180 6 1
device=MB110S
T 44100 58900 5 10 0 0 180 6 1
symversion=0.1
T 43900 60300 5 10 0 0 0 0 1
footprint=MB1-PLCC4
}
N 43100 60100 43900 60100 4
N 43100 59700 43900 59700 4
N 43900 59700 43900 59600 4
C 45300 58500 1 0 0 gnd-1.sym
N 45100 59600 45400 59600 4
N 45400 59600 45400 58800 4
N 45400 58800 49100 58800 4
C 50300 58800 1 90 0 resistor-1.sym
{
T 49900 59100 5 10 0 0 90 0 1
device=RESISTOR
T 50300 58800 5 10 0 0 90 0 1
footprint=0805
T 50000 59000 5 10 1 1 90 0 1
refdes=R1
T 50500 59000 5 10 1 1 90 0 1
value=330
}
N 50200 59700 50200 60100 4
C 50400 57800 1 90 0 led-3.sym
{
T 50400 57800 5 10 0 0 0 0 1
footprint=0805
T 50650 57750 5 10 1 1 90 0 1
device=GREEN LED
T 49850 58250 5 10 1 1 90 0 1
refdes=D1
}
C 50100 57400 1 0 0 gnd-1.sym
N 50200 57700 50200 57800 4
N 50200 58700 50200 58800 4
C 42000 55200 1 0 0 mcp1702-1.sym
{
T 42900 56200 5 10 0 1 0 0 1
footprint=SOT23
T 43400 56200 5 10 1 1 0 6 1
refdes=U3
T 42900 56200 5 10 1 1 0 6 1
device=AP2120
}
C 42700 54400 1 0 0 gnd-1.sym
C 43700 55800 1 270 0 capacitor-1.sym
{
T 44400 55600 5 10 0 1 270 0 1
device=CAPACITOR
T 44600 55600 5 10 0 0 270 0 1
symversion=0.1
T 43700 55800 5 10 0 0 0 0 1
footprint=0805
T 44000 55500 5 10 1 1 0 0 1
refdes=C3
T 44000 55000 5 10 1 1 0 0 1
value=1uF
}
N 43600 55800 45400 55800 4
{
T 45600 55700 5 10 1 1 0 0 1
netname=VBIAS
}
N 42800 54700 42800 55200 4
N 42800 54900 44900 54900 4
C 45000 54900 1 90 0 resistor-1.sym
{
T 44600 55200 5 10 0 0 90 0 1
device=RESISTOR
T 45000 54900 5 10 0 0 90 0 1
footprint=0805
T 45400 55500 5 10 1 1 180 0 1
refdes=R2
T 45400 55200 5 10 1 1 180 0 1
value=330
}
T 44100 56200 9 10 1 0 0 0 2
Note: R2 prevents current pushed through the feedback networks of
 the op-amps from affecting the 1.2V reference rail
C 41400 56300 1 0 0 5V-plus-1.sym
N 42000 55800 41600 55800 4
N 41600 55800 41600 56300 4
N 45800 48300 45800 51300 4
{
T 45700 49300 5 10 1 1 90 0 1
netname=MSS_OCC
}
N 44200 50600 42600 50600 4
{
T 41700 50600 5 10 1 1 0 0 1
netname=OCC_EN
}
N 46300 49100 48500 49100 4
N 43100 48900 47100 48900 4
N 48500 48500 46500 48500 4
N 46500 48500 46500 48300 4
N 48500 48900 47300 48900 4
N 48500 48700 47100 48700 4
N 47100 48700 47100 48900 4
N 43100 48700 46900 48700 4
N 46900 48700 46900 48100 4
N 46900 48100 48500 48100 4
C 44200 50200 1 0 0 NCV8402A-1.sym
{
T 43600 51100 5 10 1 1 0 0 1
device=NCV8402A
T 44800 50700 5 10 0 0 0 0 1
footprint=SOT223
T 43600 51300 5 10 1 1 0 0 1
refdes=Q1
}
C 44600 49400 1 0 0 gnd-1.sym
N 44700 51000 44900 51000 4
N 44700 51000 44700 51300 4
N 44700 51300 45800 51300 4
C 43600 50000 1 0 0 resistor-1.sym
{
T 43900 50400 5 10 0 0 0 0 1
device=RESISTOR
T 43600 49800 5 10 1 1 0 0 1
refdes=R9
T 44100 49800 5 10 1 1 0 0 1
value=100k
T 43600 50000 5 10 0 0 0 0 1
footprint=0805
}
N 44700 49700 44700 50200 4
N 44500 50100 44700 50100 4
N 43400 50100 43400 50600 4
N 43400 50100 43600 50100 4
N 47100 48300 47100 47000 4
C 47000 46700 1 0 0 gnd-1.sym
C 46100 53100 1 0 0 tsv912-1.sym
{
T 46925 53250 5 8 0 0 0 0 1
device=TSV912
T 46400 53400 5 10 1 1 0 0 1
refdes=U4
T 45400 53400 5 10 1 1 0 0 1
device=TSV912
T 46100 53100 5 10 0 0 0 0 1
slot=1
T 46100 53100 5 10 0 0 0 0 1
footprint=SO8
}
C 49000 52900 1 0 0 tsv912-1.sym
{
T 49825 53050 5 8 0 0 0 0 1
device=TSV912
T 49300 53200 5 10 1 1 0 0 1
refdes=U4
T 48500 53800 5 10 1 1 0 0 1
device=TSV912
T 49000 52900 5 10 0 0 0 0 1
slot=2
T 49000 52900 5 10 0 0 0 0 1
footprint=SO8
}
C 55300 53100 1 0 0 ATtiny48-mlf32.sym
{
T 60100 58200 5 10 1 1 0 0 1
refdes=U5
T 56700 58200 5 10 1 1 0 6 1
device=ATtiny48
T 57595 53495 5 10 1 1 0 8 1
footprint=QFN32_5x5_0.5
}
N 58100 53300 58100 53000 4
N 58600 53300 58600 53000 4
C 58000 52700 1 0 0 gnd-1.sym
N 58100 53000 59100 53000 4
N 59100 53000 59100 53300 4
C 60100 59400 1 270 0 capacitor-1.sym
{
T 60800 59200 5 10 0 1 270 0 1
device=CAPACITOR
T 60400 59100 5 10 1 1 0 0 1
refdes=C7
T 61000 59200 5 10 0 0 270 0 1
symversion=0.1
T 60400 58600 5 10 1 1 0 0 1
value=0.1uF
T 60100 59400 5 10 0 0 0 0 1
footprint=0805
}
C 60900 59400 1 270 0 capacitor-1.sym
{
T 61600 59200 5 10 0 1 270 0 1
device=CAPACITOR
T 61200 59100 5 10 1 1 0 0 1
refdes=C8
T 61800 59200 5 10 0 0 270 0 1
symversion=0.1
T 61200 58600 5 10 1 1 0 0 1
value=0.1uF
T 60900 59400 5 10 0 0 0 0 1
footprint=0805
}
N 60300 58500 61100 58500 4
C 61000 58200 1 0 0 gnd-1.sym
C 58500 59400 1 0 0 5V-plus-1.sym
N 58700 59400 61100 59400 4
N 58700 58300 59200 58300 4
N 58700 59400 58700 58300 4
N 61800 55800 64300 55800 4
C 64100 55800 1 270 0 capacitor-1.sym
{
T 64800 55600 5 10 0 1 270 0 1
device=CAPACITOR
T 65000 55600 5 10 0 0 270 0 1
symversion=0.1
T 64100 55800 5 10 0 0 0 0 1
footprint=0805
T 64400 55500 5 10 1 1 0 0 1
refdes=C9
T 64400 55000 5 10 1 1 0 0 1
value=1uF
}
C 64200 54600 1 0 0 gnd-1.sym
N 64300 55800 64300 56300 4
N 65000 56100 64300 56100 4
{
T 64500 56200 5 10 1 1 0 0 1
netname=\_RESET\_
}
C 64100 57200 1 0 0 5V-plus-1.sym
N 55600 55000 54900 55000 4
{
T 54300 55000 5 10 1 1 0 0 1
netname=MOSI
}
N 55600 54700 54900 54700 4
{
T 54300 54700 5 10 1 1 0 0 1
netname=MISO
}
N 55600 54400 54900 54400 4
{
T 54400 54400 5 10 1 1 0 0 1
netname=SCK
}
C 51600 53300 1 270 0 capacitor-1.sym
{
T 52300 53100 5 10 0 1 270 0 1
device=CAPACITOR
T 51900 53000 5 10 1 1 0 0 1
refdes=C5
T 52500 53100 5 10 0 0 270 0 1
symversion=0.1
T 51900 52500 5 10 1 1 0 0 1
value=1uF
T 51600 53300 5 10 0 0 0 0 1
footprint=0805
}
C 63900 59700 1 270 0 capacitor-1.sym
{
T 64600 59500 5 10 0 1 270 0 1
device=CAPACITOR
T 63500 59400 5 10 1 1 0 0 1
refdes=C6
T 64800 59500 5 10 0 0 270 0 1
symversion=0.1
T 63500 58900 5 10 1 1 0 0 1
value=1uF
T 63900 59700 5 10 0 0 0 0 1
footprint=0805
}
C 49200 51800 1 0 0 resistor-1.sym
{
T 49500 52200 5 10 0 0 0 0 1
device=RESISTOR
T 49200 51800 5 10 0 0 0 0 1
footprint=0805
T 49500 51700 5 10 1 1 180 0 1
refdes=R6
T 50100 51700 5 10 1 1 180 0 1
value=100k
}
C 47900 51800 1 0 0 resistor-1.sym
{
T 48200 52200 5 10 0 0 0 0 1
device=RESISTOR
T 47900 51800 5 10 0 0 0 0 1
footprint=0805
T 48300 51700 5 10 1 1 180 0 1
refdes=R5
T 48700 51700 5 10 1 1 180 0 1
value=1k
}
C 64200 57600 1 270 0 res-pack4-1.sym
{
T 64200 57600 5 10 0 0 270 0 1
slot=4
T 65200 56800 5 10 1 1 0 0 1
value=10k
T 64600 56600 5 10 1 1 0 0 1
footprint=RPACK4-1206
T 64900 56800 5 10 1 1 0 0 1
refdes=R4
}
N 63200 57300 63200 59700 4
C 36800 42200 0 0 0 title-bordered-D.sym
C 51100 61500 1 0 1 termblk2-1.sym
{
T 50100 62150 5 10 0 0 0 6 1
device=TERMBLK2
T 50700 62400 5 10 1 1 0 6 1
refdes=J5
T 51100 61500 5 10 0 1 0 0 1
footprint=TERMBLK2_200MIL
}
C 51400 61200 1 0 0 gnd-1.sym
N 51100 61700 51500 61700 4
N 51500 61700 51500 61500 4
C 54700 63000 1 0 0 5V-plus-1.sym
N 51100 62100 53300 62100 4
{
T 52900 62300 5 10 1 1 0 6 1
netname=OCC_IN
}
N 55600 57600 54900 57600 4
C 52700 62000 1 270 0 capacitor-1.sym
{
T 53400 61800 5 10 0 1 270 0 1
device=CAPACITOR
T 53600 61800 5 10 0 0 270 0 1
symversion=0.1
T 52700 62000 5 10 0 0 0 0 1
footprint=0805
T 53000 61700 5 10 1 1 0 0 1
refdes=C10
T 53000 61200 5 10 1 1 0 0 1
value=1uF
}
C 52800 60600 1 0 0 gnd-1.sym
N 52900 61100 52900 60900 4
N 52900 62100 52900 62000 4
N 61800 57600 62300 57600 4
{
T 62500 57500 5 10 1 1 0 0 1
netname=IDET1
}
N 61800 57300 63200 57300 4
N 61800 54100 64400 54100 4
N 65300 54100 65700 54100 4
C 65500 52500 1 270 1 led-3.sym
{
T 65500 52500 5 10 0 0 0 6 1
footprint=0805
T 65250 52750 5 10 1 1 90 2 1
device=RED
T 66050 52850 5 10 1 1 90 2 1
refdes=D2
}
N 65700 53400 65700 54100 4
C 65800 52200 1 0 1 gnd-1.sym
N 64300 53800 64700 53800 4
C 64500 51900 1 270 1 led-3.sym
{
T 64500 51900 5 10 0 0 0 6 1
footprint=0805
T 64250 52150 5 10 1 1 90 2 1
device=AMBER
T 65100 52200 5 10 1 1 90 2 1
refdes=D3
}
N 64700 52800 64700 53800 4
C 64800 51600 1 0 1 gnd-1.sym
C 64700 53700 1 0 1 res-pack2-1.sym
{
T 64700 53700 5 10 0 0 0 6 1
footprint=RPACK2-0606
T 64700 53700 5 10 0 0 0 6 1
slot=2
T 64100 53500 5 10 1 1 0 6 1
value=1k
T 63500 53500 5 10 1 1 0 0 1
refdes=R8
}
C 65700 54000 1 0 1 res-pack2-1.sym
{
T 65700 54000 5 10 0 0 0 6 1
footprint=RPACK2-0606
T 65105 54500 5 10 1 1 0 6 1
refdes=R8
T 65100 54300 5 10 1 1 0 6 1
value=1k
}
T 67400 53200 9 10 1 0 0 6 1
Occupancy Detect
T 64100 52300 9 10 1 0 0 6 1
Heartbeat
N 61800 53800 63400 53800 4
N 55600 56700 54900 56700 4
{
T 54000 56700 5 10 1 1 0 0 1
netname=OCC_EN
}
C 47200 59500 1 0 0 78l05-1.sym
{
T 48800 60800 5 10 0 0 0 0 1
device=7805
T 48600 60500 5 10 1 1 0 6 1
refdes=U2
T 47200 59500 5 10 0 1 0 0 1
footprint=SOT89
}
N 47500 47900 48500 47900 4
N 47700 47700 48500 47700 4
N 47500 47900 47500 47700 4
C 54300 62700 1 90 1 mosfet-with-diode-1.sym
{
T 53800 61800 5 10 0 0 270 2 1
device=NPN_TRANSISTOR
T 54300 61800 5 10 1 1 0 6 1
refdes=Q2
T 54300 61600 5 10 1 1 0 6 1
value=BSS138
T 54300 62700 5 10 0 0 0 6 1
footprint=SOT23_MOS
}
N 54900 63000 53800 63000 4
N 53800 63000 53800 62700 4
N 54300 62100 54900 62100 4
N 54900 62100 54900 57600 4
C 45300 46300 1 0 0 header2-1.sym
{
T 46300 46950 5 10 0 0 0 0 1
device=HEADER2
T 45700 47200 5 10 1 1 0 0 1
refdes=JP1
T 45300 46300 5 10 0 1 0 6 1
footprint=JUMPER2
}
N 45200 47700 45200 46900 4
N 45200 46900 45300 46900 4
N 45300 46500 44400 46500 4
T 45100 45300 9 10 1 0 0 0 4
JP1 connects MSS bus pin 7 to MSS Ground
In the 2.0.0 spec, pin 7 is optionally ground, optionally 
some other vendor-specific use.

C 55000 62100 1 90 0 resistor-1.sym
{
T 54600 62400 5 10 0 0 90 0 1
device=RESISTOR
T 55000 62100 5 10 0 0 90 0 1
footprint=0805
T 54700 62300 5 10 1 1 90 0 1
refdes=R10
T 55200 62300 5 10 1 1 90 0 1
value=10k
}
