#pragma once

/*
 * Temporary Phase 4.7 hardware smoke harness.
 *
 * Adds WRAM (region 0x0C) on top of the 4.6 read-only matrix.  Like 4.6,
 * this is not production savestate UI/storage; it runs once during boot,
 * before the normal FPGA UART tasks are started, to validate read-only
 * state_port regions from the Phase 4.7 FPGA bitstream.
 *
 * Diagnostic ramp before the final gate (40, 256, 8192, 32768) lets a
 * partial-WRAM bitstream still light up some packets even if the full
 * 32 KiB stream fails late.
 */

void FusionSavestateSmoke47_RunBlocking(void);
