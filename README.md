# Door control on STM32F103C8

The application starts in `Core/Src/main.c` through `DoorControl_Start()`.
`Core/Src/door_control.c` owns the one-slot entry/exit flow, hardware setup,
queues, and the existing front/sky motor state machines. The active FreeRTOS
tasks remain in separate files under `Core/Src/tasks/`: RFID/IR input, Pi UART,
front motor, LCD2004, and buzzer. Older task sources remain available but are
not started by the active application.

## Entry and exit

1. The LCD starts at `XE: 0/1 - CON SLOT`. To enter, a card UID and a Pi
   `CAR_DETECTED` event must both arrive within 15 seconds, in either order.
   One signal alone cannot open the gate.
2. The BTS7960 opens for up to 8 seconds. When the IR sensor detects the car,
   the count changes to `1/1`, the LCD reports full, and the buzzer sounds once.
3. Three seconds after opening finishes, the motor closes for up to 8 seconds.
   Closing waits while IR is blocked. If IR is blocked during closing, the
   motor stops in a fault state and a reset is required.
4. Once IR clears and the gate closes, a different card UID opens the gate
   for exit and changes the count to `0/1`. Pi confirmation is required only
   for entry. Remove each card before presenting another one.

The count and entry UID are held in RAM and return to 0/1 on reset. A single
IR obstacle sensor cannot identify vehicle type or direction; the exit count
is changed by the second card, not by an exit crossing measurement. The motor
travel limits are time based, with no end-stop or current measurement.

## Connections

| Device | STM32 pin |
| --- | --- |
| RC522 SDA/SS, SCK, MISO, MOSI, RST | PB12, PB13, PB14, PB15, PB9 |
| LCD2004 I2C SCL, SDA | PB6, PB7 |
| BTS7960 RPWM, LPWM, R_EN, L_EN | PA0, PA1, PA2, PA3 |
| IR digital output, active low | PB8 |
| Active-high buzzer input | PA11 |
| Pi GPIO14/TXD, physical pin 8 | PA10/USART1_RX |
| Pi GPIO15/RXD, physical pin 10 | PA9/USART1_TX |

Connect Pi and STM32 grounds. Both UARTs use 115200 baud, 8N1, and 3.3 V
logic. The Pi sends exactly `CAR_DETECTED\n` once per new car detection. For
example, a Pi application using a serial port can call
`serial_port.write(b"CAR_DETECTED\n")` after its detector confirms a car.
The Pi camera application is maintained separately from this firmware.

The LCD driver probes PCF8574/PCF8574A addresses 0x20-0x27 and 0x38-0x3F;
it expects P0=RS, P2=E, P3=backlight, and P4-P7=data. Check the backpack's
actual mapping and contrast setting if its backlight works but text is blank.

## Build and diagnose

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Flash `build/Debug/Bring_upSTM32.elf`. In the debugger, inspect
`door_control_status`, `door_lcd_address7`, `door_pi_detect_count`, and
`door_occupied`. Status 2 means no LCD I2C response; 4 means RC522 did not
initialize; 7 means the application started; 8 means a card was read; 9 means
USART1 setup failed. The Pi counter increments when a valid UART detection
reaches the flow task. If the LCD shows `KIEM TRA RC522 SPI2`, fix RC522
power/wiring before testing the entry flow.

The older task files can be compiled for separate checks with
`-DDOOR_BUILD_LEGACY_TASKS=ON`. Their individual `DOOR_TASK_<NAME>` options
control compilation only; the active application does not start those tasks.
