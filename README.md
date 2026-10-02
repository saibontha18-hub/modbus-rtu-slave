# Modbus RTU Slave

A Modbus RTU slave protocol stack in C99 — no dynamic allocation, no libc I/O
in the stack itself. It handles the serial line protocol that lets a
microcontroller act as a sensor/actuator node on an industrial RS-485 bus:
frame validation, CRC-16, register read/write commands, and proper Modbus
exception responses.

## Features

- **Function codes:** `0x01` Read Coils, `0x03` Read Holding Registers,
  `0x04` Read Input Registers, `0x05` Write Single Coil,
  `0x06` Write Single Register, `0x0F` Write Multiple Coils,
  `0x10` Write Multiple Registers
- **Coil bank:** bit-packed LSB-first per the Modbus spec, with the same
  range/exception discipline as the register banks
- **Master helpers** (`src/modbus_master.*`): request builders for all
  supported function codes — construct valid CRC'd request frames for
  integration use; invalid arguments safely produce no frame
- **Exceptions:** `0x01` illegal function, `0x02` illegal data address,
  `0x03` illegal data value
- **Spec-correct silence rules:** corrupted frames, wrong-slave frames, and
  broadcast reads get no response; broadcast writes apply silently
- **RTU framing:** silence-delimited receiver (`modbus_rx_t`) using an
  injectable millisecond clock — no hardware timers needed for tests
- **Hardware abstraction:** all UART access goes through `hal_uart_t`, so the
  same stack runs on a real target and on the host

## Layout

```
hal/        UART abstraction (hal_uart.h) + software mock (mock_uart.*)
            The mock replays scripted RX bytes, captures TX, and provides a
            controllable millisecond clock for the silence detection.
src/        The stack: modbus_rtu.h / modbus_rtu.c (slave),
            modbus_master.h / modbus_master.c (request builders)
app/        Demo: wires the slave to the mock UART and runs a scripted
            master session (read regs -> write reg -> read back ->
            write coil -> read coils).
tests/      Self-contained test suite (30 tests, no external framework).
```

## Build & test

```sh
make test     # builds and runs the 30-test suite
make demo && ./demo
```

Everything compiles under `gcc -Wall -Wextra -Werror -std=c11 -pedantic`.

## Demo output

```
--- read holding regs 0..1 ---
RX       ( 8 bytes): 01 03 00 00 00 02 C4 0B
TX       ( 9 bytes): 01 03 04 09 29 00 00 28 67
--- write holding reg 1 = 0x1234 ---
RX       ( 8 bytes): 01 06 00 01 12 34 D5 7D
TX       ( 8 bytes): 01 06 00 01 12 34 D5 7D
--- read holding reg 1 ---
RX       ( 8 bytes): 01 03 00 01 00 01 D5 CA
TX       ( 7 bytes): 01 03 02 12 34 B5 33
--- write single coil 5 = ON ---
RX       ( 8 bytes): 01 05 00 05 FF 00 9C 3B
TX       ( 8 bytes): 01 05 00 05 FF 00 9C 3B
--- read coils 0..9 ---
RX       ( 8 bytes): 01 01 00 00 00 0A BC 0D
TX       ( 7 bytes): 01 01 02 20 00 A0 3C
```

## Porting to real hardware

1. Implement `hal_uart_t` with your UART peripheral (blocking read with
   timeout, write, and a millisecond tick — `SysTick` or a hardware timer).
2. Compute the 3.5-character silence for your baud rate
   (e.g. ~4 ms at 9600 baud, ~1.75 ms at 19200) and pass it as
   `frame_gap_ms` to `modbus_rx_ready()`.
3. Point `modbus_map_t` at your real register image and call
   `modbus_process_frame()` for each delimited frame.

Tested on Linux with the mock UART; the stack itself is platform-agnostic.
