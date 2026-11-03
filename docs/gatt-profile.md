# BLE GATT Profile — ACU v3 Telemetry

> **Superseded.** The ACU no longer sends the binary frame below. It sends plain text over the
> RN4871 transparent UART (USART1), readable in any BLE serial terminal: one line whenever a key
> state changes, and a status block every second. Phone commands: `stop`, `start`, `flush`.
> See `Core/Src/ble_handler.c`.

## Overview
The ACU v3 transmits a 15-byte binary frame at 10 Hz over a custom BLE GATT service using an RN4871 module.

## Service
- **UUID**: `E18A0001-1E11-4F63-A23A-2D84F600A5D1`
- **Type**: Primary Service

## Characteristic
- **UUID**: `E18A0002-1E11-4F63-A23A-2D84F600A5D1`
- **Properties**: Notify (0x10) + Read (0x02)
- **Descriptors**: Client Characteristic Configuration (0x2902) — enable/disable notifications
- **Value**: 15 bytes, little-endian, updated at 10 Hz

## Data Format

| Byte | Type    | Field          | Scale    | Example Value     |
|------|---------|----------------|----------|-------------------|
| 0    | uint8   | State Machine  | 1        | 2 = AS_ON         |
| 1    | uint8   | ASSI Status    | 1        | 4 = EMERGENCY     |
| 2    | uint8   | Mission        | 1        | 4 = TRACKDRIVE    |
| 3-4  | uint16 LE | Hydraulic P1 | 0.01 bar | 5000 = 50.00 bar  |
| 5-6  | uint16 LE | Hydraulic P2 | 0.01 bar | 4800 = 48.00 bar  |
| 7-8  | uint16 LE | Pneumatic P1  | 0.01 bar | 6200 = 62.00 bar  |
| 9-10 | uint16 LE | Pneumatic P2  | 0.01 bar | 6100 = 61.00 bar  |
| 11-12| int16 LE  | Chip Temp     | 0.01 °C  | 3250 = 32.50 °C   |
| 13   | uint8     | Solenoid Front| 0/1      | 1 = engaged        |
| 14   | uint8     | Solenoid Rear | 0/1      | 0 = disengaged     |

## State Machine Values (Byte 0)

| Value | Enum           | Meaning           |
|-------|----------------|-------------------|
| 0     | Start          | Initializing      |
| 1     | IDLE           | Idle, WDT enabled |
| 2     | AS_ON          | Autonomous active |
| 3     | EMERGENCY      | Emergency state   |

## ASSI Status Values (Byte 1)

| Value | Enum                | Meaning          |
|-------|---------------------|------------------|
| 1     | AS_STATE_OFF        | System off       |
| 2     | AS_STATE_READY      | System ready     |
| 3     | AS_STATE_DRIVING    | Autonomous driving |
| 4     | AS_STATE_EMERGENCY  | Emergency active |
| 5     | AS_STATE_FINISHED   | Run finished     |

## Mission Values (Byte 2)

| Value | Enum          | Meaning       |
|-------|---------------|---------------|
| 0     | MANUAL        | Manual mode   |
| 1     | ACCELERATION  | Acceleration  |
| 2     | SKIDPAD       | Skidpad       |
| 3     | TRACKDRIVE    | Track drive   |
| 4     | EBS_TEST      | EBS test      |
| 5     | INSPECTION    | Inspection    |
| 6     | AUTOCROSS     | Autocross     |

## RN4871 Configuration

Configure the RN4871 module once (persists in flash). Send these commands via UART at 115200 baud:

```
$$$
SS,E18A0001-1E11-4F63-A23A-2D84F600A5D1
PS,E18A0002-1E11-4F63-A23A-2D84F600A5D1
SH,001A
PC,1A
---
```

Each command responds with `AOK` on success.

### Command Reference

| Command | Purpose                          |
|---------|----------------------------------|
| `$$$`   | Enter command mode               |
| `SS,...`| Set primary service UUID         |
| `PS,...`| Set primary characteristic UUID  |
| `SH,001A`| Set device name (26 chars max)  |
| `PC,1A` | Set connection mode (1 = auto-advertise) |
| `---`   | Exit command mode, start advertising |

## Client Connection Flow

1. **Scan** for devices advertising service `E18A0001-1E11-4F63-A23A-2D84F600A5D1`
2. **Connect** to discovered device
3. **Discover services** — find the characteristic `E18A0002-1E11-4F63-A23A-2D84F600A5D1`
4. **Enable notifications** — write 0x0001 to CCCD (0x2902)
5. **Read initial value** — optional, characteristic supports Read
6. **Parse notifications** — 15-byte little-endian frames at 10 Hz

## Byte Order

All multi-byte fields are **little-endian** (ARM Cortex-M native). Example:
- Hydraulic P1 = 0x1388 → bytes [0x88, 0x13] → value = 5000 → 50.00 bar
