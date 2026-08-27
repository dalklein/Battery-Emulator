#ifndef ENABLE_SENSE_H
#define ENABLE_SENSE_H

/* Delta E-series 12 V "battery enable" line — sense and publish.
 *
 * The Delta asserts a 12 V permission signal to the battery once it is satisfied it is
 * talking to a valid pack over Modbus. On a real LG RESU that is what allows the pack to
 * bring up its internal DC-DC. It is therefore the single cleanest pass/fail signal for
 * "did the emulator convince the inverter?" — and it needs no DC connection to observe.
 *
 * This module exists so the line can be MEASURED before the inverter module is written,
 * using the same board and the same pin the emulator will later use for
 * allows_contactor_closing(). Nothing here is throwaway.
 *
 * It is deliberately self-contained and READ-ONLY with respect to the rest of the
 * emulator: it does NOT touch datalayer.system.status.inverter_allows_contactor_closing,
 * which defaults to true and is consulted by battery code to decide contactor closing.
 * Writing that flag from here would change behaviour elsewhere. See enable_sense_drive_datalayer.
 */

#include <stdbool.h>
#include <stdint.h>

/** Claim and configure the pin. Safe to call when MQTT is not up yet. Returns false if the
 *  HAL has no enable pin for this board or the pin is already allocated to something else. */
bool enable_sense_init(void);

/** Poll + publish. Call from core_loop (1 ms cadence) so edges are timestamped precisely. */
void enable_sense_loop(void);

/** Last debounced state, or false before the first stable read. */
bool enable_sense_state(void);

/** true  = 12 V present reads HIGH  (resistor divider straight to the GPIO)
 *  false = 12 V present reads LOW   (optocoupler pulling the pin down)
 *  Default true. Set before enable_sense_init(). */
extern bool enable_sense_active_high;

/** OFF by default, and think before turning it on. When true this module also drives
 *  datalayer.system.status.inverter_allows_contactor_closing. That flag gates contactor
 *  closing in several battery drivers, so enabling this changes real behaviour — it is
 *  meant for the milestone-2 emulator, not for the measurement phase. */
extern bool enable_sense_drive_datalayer;

#endif
