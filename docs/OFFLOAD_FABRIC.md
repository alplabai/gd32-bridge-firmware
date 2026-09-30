# Unused offload fabric: verdicts (gh#65)

Decision record. Document numbers refer to the GD32G553 User Manual Rev1.2
(UM) and GD32G553xx Datasheet Rev2.0 (DS).

## LPTIMER tick -- not wired; arithmetic only

A 50 ms SysTick already retires the main-loop `__WFI()` in Run/Sleep
(gh#54), and `hal/gd32/power.c` stops it across Deep-sleep entry. An LPTIMER
tick would add a periodic wake inside Deep-sleep, which changes power and
wake semantics, so it is not enabled. If it ever is, mind the period maths:
at IRC32K/16 = 2 kHz (500 us per
count) a 2 ms tick is CARL = 3, not 3999 (3999 is 2 s). IRC32K is 28..36 kHz
(DS p.125, Table 4-23), so every timeout derived from the tick must tolerate
-12.5 % / +14.3 %. The RTC wakeup timer already in `power.c` gives the same
tick with no new peripheral; prefer it unless LPTIMER's DMAMUX/TRIGSEL
fan-out is wanted. Bring-up traps: EXTI line 35 must be configured as well
as the NVIC vector (UM p.210, p.214); CTL0/INTEN are writable only with
LPTEN = 0, CAR/CMPV only with LPTEN = 1 (UM p.908, p.934-935); CNT needs two
matching reads (UM p.936); chapter 24 names APB2 as the bus but RCU and the
base address say APB1 -- trust chapter 4.

## CMP into TIMER0/TIMER7 BREAK0 -- blocked

No current-sense or rail-monitor net reaches a free GD32 pad on this SoM
revision, and the protocol has no fault-report opcode, so a hardware break
would be a silent actuator shutdown. Needed first: a sense net on PB13
(ball H1, CMP4_IP, DS p.47) on the next E1M-X revision, plus a fault-report
field/opcode (wire MINOR bump).

Intended sequence: enable CMPEN (RCU_APB2EN bit 3); leave PB13 analog (reset
state); CMP4_CS (UM p.510): CMP4MSEL = 001 (VREFINT/2), CMP4PSEL = 0,
CMP4HST = 011, blanking optional via CMP4BLK[3:0]; wait tSTART_SCALER 60 us
then tSTART 2 us (DS p.140); set BRK0CMP4EN (bit 13 of TIMERx_AFCTL0, offset
0x8C, PROT[1:0] = 00) on TIMER0 and TIMER7; set BRKEN in TIMERx_CCHP0; set
CMP4LK last, only after bench validation (one-way until MCU reset, UM p.496).
Take bit names from the register diagram (p.510), not the p.511-512 text.

## DMAMUX synchronization -- rejected

NBR[4:0] is 5 bits (UM p.316): at most 32 requests per event against a
69-byte envelope. Documented at the DMA channel setup in
`hal/transport_hw_gd32.c`.

## CLA -- rejected

PA8 is not a TRIGSEL pad and the CLA has no counter (UM p.329-330); it is
unusable in Deep-sleep. Documented next to `spi_cs_exti_init()`.
