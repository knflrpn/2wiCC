# 2wiCC -- Switch (2) Controller Controller
Nintendo Switch Controller Emulator for RP2040/RP2350.

2wiCC is a controller emulator that presents itself as a Switch (1) Pro Controller to the console while providing control from a PC via UART commands.

## Hardware Requirements

**Recommended**: Waveshare RP2040 Zero (for compact size and onboard RGB LED)
- Any RP2040-based board should work with pin configuration changes
- Optional: External WS2812 RGB LED on GPIO 16 (if not using RP2040 Zero)

## Connections

| Function | GPIO Pin | Notes |
|----------|----------|-------|
| UART TX | 0 (default) | Serial output from RP2040 |
| UART RX | 1 (default) | Serial input to RP2040 |
| Status LED | 16 | WS2812 RGB LED (onboard on RP2040 Zero) |
| VSYNC Input | 14 | Optional: for frame synchronization |

## Assembly Options

### Option 1: Dual-Board Setup (Recommended)
Configure a second RP2040 board as a USB-UART bridge and house both boards in an enclosure:
1. Flash the second board with a USB-UART bridge firmware.  Recommended is the dedicated bridge for 2wiCC, available at
2. Cross-wire UART pins (TX→RX, RX→TX) between boards
3. Connect grounds between boards

### Option 2: Direct Connection
Connect 2wiCC directly to your host system via a UART-to-USB adapter.

- **Baud Rate**: 460,800 bps
- **Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Flow Control**: None

## LED Status Indicator

The RGB LED provides visual feedback:
- **Blue**: USB connection established
- **Red Pulse**: Heartbeat indicates that controller state is being updated

## Command Reference

### Controller Input Commands

| Command | Parameter | Description |
|---------|-----------|-------------|
| `QD` | 6 hex digits | Queue digital-only controller state (3 bytes) |
| `QF` | 18 hex digits | Queue full controller state (9 bytes: 3 digital + 6 analog) |

**Controller State Format**:
- Digital (3 bytes): Button states as bitmask (bit 7-0 as a hex character encoded in ASCII)
    - First byte: [ZR, R, Right SL, Right SR, A, B, X, Y] (bits 7-0)
    - Second byte: [charging_grip, unused, Capture, Home, Left Stick, Right Stick, +, -] (bits 7-0)
    - Third byte: [ZL, L, Left SL, Left SR, D-pad Left, D-pad Right, D-pad Up, D-pad Down] (bits 7-0)
- **Analog (6 bytes)**: Left stick X, Y, Right stick X, Y, encoded in the Pro Controller data format (see https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md).  Center position is 0x800 and full swing is +/- 0x600.

### Playback Mode Commands

| Command | Parameter | Description |
|---------|-----------|-------------|
| `SPM` | `RT` or `BUF` | Set playback mode (Real-Time or Buffered) |

### Queue Management Commands

| Command | Parameter | Description |
|---------|-----------|-------------|
| `GQR` | None | Get queue remaining space |
| `GQS` | None | Get total queue size (default 1024) |

### Synchronization Commands

| Command | Parameter | Description |
|---------|-----------|-------------|
| `VSYNC` | `0` or `1` | Disable/enable VSYNC synchronization |
| `VSD` | 4 hex digits | Set frame delay in microseconds (0000-FFFF) |

### Recording Commands

| Command | Parameter | Description |
|---------|-----------|-------------|
| `REC` | `0` or `1` | Stop/start input recording |
| `GRR` | None | Get recording buffer remaining space |
| `GRS` | None | Get total recording buffer size (default 1024) |
| `GR` | `0` or `1` | Get recorded data (0=start from beginning, 1=continue) |

### System Commands

| Command | Parameter | Description |
|---------|-----------|-------------|
| `ID` | None | Returns `+2wiCC` (device identification) |
| `VER` | None | Returns firmware version |
| `GCS` | None | Get USB connection status (0=disconnected, 1=connected) |
| `LED` | `0` or `1` | Disable/enable status LED |
| `ECHO` | text | Echo back the provided text |

## Usage Examples

### Basic Controller Input
```
+QD 080000    # Press A button (neutral sticks)
+QD 000000    # Release all buttons
```

### Full Controller State with Analog Sticks
```
+QF 000000000880000880    # A button + neutral sticks
```

### Recording Workflow
```
+REC 1                    # Start recording
# Play the game...
+REC 0                    # Stop recording
+GR 0                     # Download recorded inputs
```

Repeat `+GR 1` until the entire recording is retrieved.

## Recording Format

Recorded inputs use run-length encoding and are returned in this format:
```
+R [18 hex digits]x[2 hex digits]
```
- **18 hex digits**: Full controller state (9 bytes)
- **x**: Separator
- **2 hex digits**: Number of consecutive frames with this state (1-240)

Example: `+R 000000000880000880x74` means neutral controller for 0x74 frames.

## Playback Modes

### Real-Time Mode (`SPM RT`)
- Always uses the most recently queued controller state
- Best for live input injection

### Buffered Mode (`SPM BUF`)
- Plays controller states sequentially from the queue
- Advances one state per frame (60 FPS if VSYNC disabled, otherwise according to VSYNC)
- Automatically switches back to real-time if buffer runs empty

