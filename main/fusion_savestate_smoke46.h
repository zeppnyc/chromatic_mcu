#pragma once

/*
 * Temporary Phase 4.6 hardware smoke harness.
 *
 * This is not production savestate UI/storage.  It runs once during boot,
 * before the normal FPGA UART tasks are started, to validate read-only
 * state_port regions from the Phase 4.5 FPGA bitstream.
 */

void FusionSavestateSmoke46_RunBlocking(void);
