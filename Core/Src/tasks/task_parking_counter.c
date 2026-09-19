#include "door_control.h"

void vTaskParkingCounter(void *pv)
{
    (void)pv;
    parking_crossing_evt_t evt;

    for (;;) {
        /* Blocks until Pi4 (via vTaskUART_Pi) reports a confirmed crossing.
           This is deliberately a SEPARATE event from EVT_CAR_MATCH: match
           only means "authorized to enter", fired on approach. Crossing
           means the car actually passed the gate line — that's the only
           thing that should move the counter, so a car that gets access
           but backs out never gets counted. Direction detection itself is
           Pi4-side work (e.g. tracking bounding-box motion across a virtual
           line in the frame, or two beam sensors) — out of scope here. */
        if (xQueueReceive(xParkingCrossingQueue, &evt, portMAX_DELAY) == pdTRUE) {

            if (evt.direction == DIR_ENTRY) {
                if (xSemaphoreTake(xParkingSlotSem, 0) != pdTRUE) {
                    /* Semaphore already at 0 — an entry was confirmed while
                       we had no free slot on record. Should not happen if
                       the full-lot gate above is working; treat as a
                       miscount/anomaly and flag it rather than silently
                       going negative (counting semaphore can't go negative
                       anyway — this branch just means we dropped the
                       event). Log it. */
                    lcd_update_t err = {0};
                    strncpy(err.line1, "COUNT ERROR", 16);
                    strncpy(err.line2, "Entry w/ 0 free", 16);
                    xQueueSend(xLcdQueue, &err, 0);
                }
            } else { /* DIR_EXIT */
                if (xSemaphoreGive(xParkingSlotSem) != pdTRUE) {
                    /* Give failed => already at MAX_PARK_SLOTS. Means an
                       exit was double-reported, or an earlier entry was
                       never counted. Also an anomaly worth surfacing. */
                    lcd_update_t err = {0};
                    strncpy(err.line1, "COUNT ERROR", 16);
                    memcpy(err.line2, "Exit w/ lot full", 16); /* 16 LCD columns; byte 17 stays NUL */
                    xQueueSend(xLcdQueue, &err, 0);
                }
            }

            lcd_update_t upd = {0};
            snprintf(upd.line1, sizeof(upd.line1), "Slots: %u/%u",
                      ParkingSlots_Count(), MAX_PARK_SLOTS);
            strncpy(upd.line2, ParkingSlots_Available() ? "" : "LOT FULL", 16);
            xQueueSend(xLcdQueue, &upd, 0);
        }
    }
}
