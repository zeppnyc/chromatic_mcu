#pragma once

#include <stdbool.h>

/*
 * Phase 4.8a transport reliability probe.
 *
 * This is MCU-only and read-only: it captures the Phase 4.7 region matrix
 * through the existing state-port read protocol, validates stream integrity,
 * and reports detailed transport diagnostics. It does not write flash and it
 * never sends BEGIN_LOAD / WRITE_STREAM_BEGIN / WRITE_COMMIT.
 *
 * Final-only boot probing is available only when the firmware is built with
 * FUSION_TRANSPORT_PROBE_BOOT_AUTO. The default production build has no
 * auto-run path; use the REPL command instead.
 */

bool FusionSavestateTransportProbe_RunRepl(bool reduced_logging);
void FusionSavestateTransportProbe_RunBootProbeOnce(void);
void FusionSavestateTransportProbe_RegisterCommands(void);
