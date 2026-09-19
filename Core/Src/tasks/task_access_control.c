#include "door_control.h"

/* ---------------------------------------------------------------------- */
/*  Two-step verification helpers                                         */
/* ---------------------------------------------------------------------- */
static void confirm_entry(void)
{
    if (!ParkingSlots_Available()) {
        /* Both steps were completed correctly, but the lot is full — deny
           at the last moment. This is a real scenario: camera matched,
           driver tapped the card, only then do we know slots ran out
           (e.g. someone else took the last one moments earlier). */
        lcd_update_t full = {0};
        strncpy(full.line1, "PARKING FULL", 16);
        strncpy(full.line2, "Please wait...", 16);
        xQueueSend(xLcdQueue, &full, 0);
        buzzer_pattern_t p = BUZZ_DENIED;
        xQueueSend(xBuzzerQueue, &p, 0);
        return;
    }

    door_cmd_msg_t m = { .door = DOOR_FRONT, .cmd = CMD_OPEN };
    xQueueSend(xFrontDoorCmdQueue, &m, 0);

    /* Direction is known from the verification order itself — no camera
       bbox-size/direction logic needed. Slot accounting (semaphore take)
       happens in vTaskParkingCounter when it drains this event. */
    parking_crossing_evt_t evt = { .direction = DIR_ENTRY };
    xQueueSend(xParkingCrossingQueue, &evt, 0);

    lcd_update_t ok = {0};
    strncpy(ok.line1, "Welcome home", 16);
    xQueueSend(xLcdQueue, &ok, 0);
}

static void confirm_exit(void)
{
    door_cmd_msg_t m = { .door = DOOR_FRONT, .cmd = CMD_OPEN };
    xQueueSend(xFrontDoorCmdQueue, &m, 0);

    parking_crossing_evt_t evt = { .direction = DIR_EXIT };
    xQueueSend(xParkingCrossingQueue, &evt, 0);

    lcd_update_t ok = {0};
    strncpy(ok.line1, "Goodbye!", 16);
    strncpy(ok.line2, "Drive safe", 16);
    xQueueSend(xLcdQueue, &ok, 0);
}

/* ---------------------------------------------------------------------- */
/*  vTaskAccessControl — front-door two-step verification                 */
/*                                                                        */
/*  ENTRY: camera matches the car first, then the driver taps the NFC     */
/*         card within TWO_STEP_TIMEOUT_MS.                               */
/*  EXIT:  the driver taps the card first, then the camera confirms the   */
/*         car within the same window.                                    */
/*  The ORDER the two events arrive in is what tells us the direction —   */
/*  this replaces needing the camera to infer entry/exit from bbox size.  */
/*  Manual button always wins immediately and cancels any pending         */
/*  half-completed sequence.                                              */
/* ---------------------------------------------------------------------- */
void vTaskAccessControl(void *pv)
{
    (void)pv;
    verify_state_t state = VERIFY_IDLE;
    TickType_t     pending_since = 0;
    yolo_result_t  yolo;
    nfc_evt_t      nfc;

    for (;;) {
        bool button_evt = false; /* TODO: shared flag/queue from vTaskButton */
        bool cam_event = (xQueueReceive(xYoloResultQueue, &yolo, 0) == pdTRUE) && yolo.match;
        bool nfc_event = (xQueueReceive(xNfcEventQueue, &nfc, 0) == pdTRUE) && nfc.valid;

        if (button_evt) {
            /* Manual override always wins and clears any stale
               half-verification so it can't leak into the next attempt. */
            state = VERIFY_IDLE;
            door_cmd_msg_t m = { .door = DOOR_FRONT, .cmd = CMD_MANUAL_OVERRIDE };
            xQueueSend(xFrontDoorCmdQueue, &m, 0);
        }

        if (state != VERIFY_IDLE &&
            (xTaskGetTickCount() - pending_since) > pdMS_TO_TICKS(TWO_STEP_TIMEOUT_MS)) {
            lcd_update_t warn = {0};
            strncpy(warn.line1, "VERIFY TIMEOUT", 16);
            strncpy(warn.line2,
                    state == VERIFY_CAM_PENDING ? "No card tapped" : "No cam confirm",
                    16);
            xQueueSend(xLcdQueue, &warn, 0);
            state = VERIFY_IDLE;
        }

        switch (state) {
        case VERIFY_IDLE:
            if (cam_event) {
                state = VERIFY_CAM_PENDING;
                pending_since = xTaskGetTickCount();
                lcd_update_t upd = {0};
                strncpy(upd.line1, "Car recognized", 16);
                memcpy(upd.line2, "Tap card to enter", 16); /* 16 LCD columns; byte 17 stays NUL */
                xQueueSend(xLcdQueue, &upd, 0);
            } else if (nfc_event) {
                state = VERIFY_NFC_PENDING;
                pending_since = xTaskGetTickCount();
                lcd_update_t upd = {0};
                strncpy(upd.line1, "Card accepted", 16);
                strncpy(upd.line2, "Confirming exit", 16);
                xQueueSend(xLcdQueue, &upd, 0);
            }
            break;

        case VERIFY_CAM_PENDING:
            if (nfc_event) {
                confirm_entry();
                state = VERIFY_IDLE;
            } else if (cam_event) {
                pending_since = xTaskGetTickCount(); /* car still in frame — refresh window */
            }
            break;

        case VERIFY_NFC_PENDING:
            if (cam_event) {
                confirm_exit();
                state = VERIFY_IDLE;
            } else if (nfc_event) {
                pending_since = xTaskGetTickCount();
            }
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
