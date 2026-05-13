# Radar Policy

Radar work targets HLK-LD6002 over UART.

## Input Policy

- UART raw bytes are for capture validation and parser debugging only.
- Training and feature baselines must use parsed radar CSV/features.
- Do not feed raw UART bytes directly into a model.

## LD6002 Parser Contract

Parser must handle:

- SOF
- ID
- LEN
- TYPE
- HEAD_CKSUM
- DATA
- DATA_CKSUM

Endian rules:

- Header fields are big-endian.
- DATA values are little-endian.

Known frame types:

- `0x0A13`: phase
- `0x0A14`: breathing rate
- `0x0A15`: heart rate
- `0x0A16`: distance

## Checksums and Errors

- Reject frames with invalid header checksum.
- Reject frames with invalid data checksum.
- Preserve enough error counters to debug link quality.
- Handle incomplete frames and resynchronization.

## Required Tests

Parser tests must cover:

- phase frame,
- breathing-rate frame,
- heart-rate frame,
- distance frame,
- checksum error,
- incomplete frame,
- endian conversion.

## Feature Route

First-stage radar work should produce:

- stable parsed CSV,
- presence / breathing / heart / distance / phase fields where available,
- radar quality score,
- motion score,
- simple baseline features.

Avoid complex deep radar models until parser quality and simple baselines are stable.
