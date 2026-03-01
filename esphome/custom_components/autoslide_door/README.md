# Autoslide Door (ESPHome External Component)

This component integrates an Autoslide automatic door opener with ESPHome over UART.
At a high level it:

- Communicates using the Autoslide AT+ protocol over 19200‑baud UART.
- Publishes door state (mode, motion, lock, forces, etc.) to ESPHome entities.
- Accepts ESPHome commands and sends them to the door as AT+UPDATE frames.
- Responds to Autoslide UPSEND frames with AT+REPLY acknowledgements.
- Performs a one‑time startup sync request, and retries on timeout with backoff.

## How It Works (High Level)

The Autoslide device speaks an AT+ protocol:

- **AT+UPDATE**: Commands sent from ESPHome to the door.
- **AT+RESULT**: Response to a command (success or failure).
- **AT+UPSEND**: Unprompted status updates from the door.
- **AT+REPLY**: Required acknowledgement for each UPSEND.

This component:

1. Waits for the door startup sweep (or a 25s timeout).
2. Requests the full status once (`REQUEST_ALL`).
3. Maintains internal state and publishes to ESPHome entities.
4. Sends commands one at a time and waits for `AT+RESULT`.
5. Marks offline on command timeouts and retries with backoff.

## Deploy Using ESPHome CLI

1. Ensure your ESPHome YAML points to this local component:
   ```yaml
   external_components:
     - source:
         type: local
         path: ../custom_components
       components: [ autoslide_door ]
   ```

2. Compile or run your config:
   ```bash
   esphome compile path/to/autoslide_door.yaml
   esphome run path/to/autoslide_door.yaml
   esphome logs path/to/autoslide_door.yaml
   ```

## Notes

- UART is 19200 baud, 5V signaling on the Autoslide board.
- If the OEM BLE module is attached, it can interfere with UART traffic.
- The component expects to control the UART bus exclusively for best results.

## Reference

See `autoslide_programmer_guide.md` in this directory for protocol details.
