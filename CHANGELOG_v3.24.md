# ZiGate+ v2 Firmware v3.24 - Network Recovery & OTA Support

## Overview

This firmware release adds complete **Network Recovery** functionality, enabling seamless backup and restore of Zigbee networks. This is particularly useful for:

- **ZHA (Zigbee Home Automation)** integration with Home Assistant
- **zigpy-zigate** library support for network migration
- Coordinator hardware replacement without losing paired devices
- Network state backup before firmware updates

## New Features

### Network Recovery Commands

| Command | Code | Description |
|---------|------|-------------|
| `NWK_RECOVERY_EXTRACT_REQ` | 0x0600 | Extract basic network parameters |
| `NWK_RECOVERY_EXTRACT_RSP` | 0x8600 | Response with network data |
| `NWK_RECOVERY_RESTORE_REQ` | 0x0601 | Restore network from backup |
| `NWK_RECOVERY_RESTORE_RSP` | 0x8601 | Restore confirmation |
| `NWK_RECOVERY_EXTRACT_EXT_REQ` | 0x0602 | Extract network + device table |
| `NWK_RECOVERY_EXTRACT_EXT_RSP` | 0x8602 | Response with full network state |
| `NWK_RECOVERY_RESTORE_EXT_REQ` | 0x0603 | Restore network + device table |
| `NWK_RECOVERY_RESTORE_EXT_RSP` | 0x8603 | Restore confirmation |

### Network Recovery Data Structure

**Basic Recovery (72 bytes)**:
- Extended PAN ID (8 bytes)
- Coordinator IEEE address (8 bytes)
- PAN ID (2 bytes)
- Network address (2 bytes)
- Channel (1 byte)
- Network Update ID (1 byte)
- Network key (16 bytes)
- Key sequence number (1 byte)
- Security level (1 byte)
- Outgoing frame counter (4 bytes)
- APS frame counter (4 bytes)
- Trust Center address (8 bytes)

**Extended Recovery (adds)**:
- Device count (1 byte)
- Device table (up to 64 devices × 12 bytes each):
  - IEEE address (8 bytes)
  - Network address (2 bytes)
  - Flags (1 byte)

### Additional Improvements

- **NWK Status Indication** (0x8703): Real-time network status events
- **Fixed permit join broadcast**: 0xFFFC now correctly opens coordinator
- **Fixed NULL pointer checks**: Improved stability
- **Fixed binding table access**: Corrected field access for group bindings
- **Re-enabled high power mode**: FCC/CE power settings now work

## Version Information

- **Firmware Version**: 5.324 (0x00050324)
- **Protocol**: Zigbee 3.0
- **Chip**: NXP JN5189

## Required Software

To use the Network Recovery features, you need:

- **zigpy-zigate** v0.13.0 or higher
- **ZHA** (Home Assistant Zigbee integration) latest version

## Usage Example (Python)

```python
import zigpy_zigate

# Extract network backup
backup = await api.network_recovery_extract()

# Restore network from backup
await api.network_recovery_restore(backup)

# Extended backup with device table
full_backup = await api.network_recovery_extract_ext()

# Full restore with devices
await api.network_recovery_restore_ext(full_backup)
```

## OTA Update Support

This firmware version also includes tested OTA (Over-The-Air) update support:

- Block request/response handling (0x8501/0x0502)
- Image notify (0x0505)
- Upgrade end request/response (0x8503/0x0504)
- Load new image (0x0500)

OTA updates have been tested with:
- ZLinky firmware (248KB, ~10 minutes transfer time)
- Transfer rate: ~400 bytes/second (limited by device, not coordinator)

## Compatibility

- **Backward compatible** with ZiGate protocol v1
- Works with existing zigpy-zigate installations
- No changes required for basic Zigbee functionality
