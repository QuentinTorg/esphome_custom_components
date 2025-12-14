#!/usr/bin/env python3
# Autoslide emulator for ESPHome testing
# Implements the AT+ protocol per the provided programming guide.
#
# Protocol summary:
# - Frames: ASCII text, start "AT+" and terminated by ESC (0x1B).
# - Controller -> Device: AT+UPDATE,<k:v>   (single key/value), AT+REPLY,r:1 (ack to UPSEND)
# - Device -> Controller: AT+RESULT,...      (reply to UPDATE), AT+UPSEND,m:*,c:*,n:* (unsolicited)
#
# Keys and ranges:
#   a: door mode (0..3)           0:AUTO 1:STACK 2:LOCK 3:PET
#   e: open speed (0..1)          0:FAST 1:SLOW
#   g: secure pet (0..1)          0:ON 1:OFF
#   j: open hold duration (00..25) seconds, MUST be two digits when set
#   C: open force (0..7)
#   z: close force (0..7)
#   A: close end force (0..7)
#   b: trigger open (0..3)        0:MASTER 1:INDOOR 2:PET 3:STACK
#   d: status request (0..1)      0:ALL_SETTINGS 1:VERSION_INFO
#   m: motion state (RO, 0..2)    0:STOPPED 1:OPENING 2:CLOSING
#   c: locked (RO, 0..1)          0:UNLOCKED 1:LOCKED  (derived from a==2)
#   n: motion trigger (RO, 0..5)  0:NONE 1:INDOOR 2:MASTER 3:PET 4:STACK 5:SOFTWARE
#
# Behavioral notes:
# - Only one k:v per AT+UPDATE.
# - Device replies to UPDATE with AT+RESULT,r:1 or a data payload (e.g., d:0 returns settings).
# - Device periodically sends AT+UPSEND,m:*,c:*,n:* and will ignore further commands until it
#   receives AT+REPLY,r:1. (This emulator enforces that gate.)
# - Trigger (b) starts an open-hold-close cycle unless locked (mode==2), in which case r:0.
# - Open/close durations are affected by open speed (e) and CLI timing parameters.

import argparse
import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, Dict

try:
    import serial  # pyserial
except Exception as e:
    print("Error: pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    raise

ESC = b"\x1b"  # 0x1B terminator


def now() -> float:
    return time.monotonic()


@dataclass
class Config:
    port: str
    baud: int = 9600
    bytesize: int = 8
    parity: str = "N"
    stopbits: int = 1
    status_interval_s: float = 2.0        # periodic UPSEND interval
    open_time_fast_s: float = 2.0
    open_time_slow_s: float = 4.0
    close_time_fast_s: float = 2.0
    close_time_slow_s: float = 4.0
    log_io: bool = True
    upsend_requires_reply: bool = True    # gate command processing until REPLY
    upsend_retry_s: Optional[float] = None  # if set, resend UPSEND if not acked


class AutoslideState:
    def __init__(self):
        # Writable settings (defaults mirror the example where reasonable)
        self.a_mode = 0  # AUTO
        self.e_open_speed = 1  # 0 FAST, 1 SLOW (example showed e:1)
        self.g_secure_pet = 0  # 0 ON, 1 OFF (example g:0)
        self.j_hold_s = 1      # seconds; keep numeric internally; send as 2-digit
        self.C_open_force = 0
        self.z_close_force = 0
        self.A_close_end_force = 7

        # Read-only dynamic status
        self.m_motion = 0      # 0 STOPPED, 1 OPENING, 2 CLOSING
        self.n_trigger = 0     # last trigger
        # c_locked is derived from mode (a==2)

        # Internal door position: 0.0 closed, 1.0 open
        self.pos = 0.0

    @property
    def c_locked(self) -> int:
        return 1 if self.a_mode == 2 else 0

    def format_settings_payload(self) -> str:
        # Example format: AT+RESULT,a:0,e:1,g:0,C:0,j:01,z:0,A:7
        return f"a:{self.a_mode},e:{self.e_open_speed},g:{self.g_secure_pet},C:{self.C_open_force}," \
               f"j:{self.j_hold_s:02d},z:{self.z_close_force},A:{self.A_close_end_force}"

    def format_status_triplet(self) -> str:
        # For UPSEND: m,c,n triplet
        return f"m:{self.m_motion},c:{self.c_locked},n:{self.n_trigger}"


class AutoslideEmulator:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.state = AutoslideState()

        self.ser = serial.Serial(
            port=cfg.port,
            baudrate=cfg.baud,
            bytesize=self._to_bytesize(cfg.bytesize),
            parity=self._to_parity(cfg.parity),
            stopbits=self._to_stopbits(cfg.stopbits),
            timeout=0.05,   # non-blocking-ish
            write_timeout=0.5,
        )

        self.rx_buf = bytearray()
        self.last_status_time = 0.0

        # UPSEND/REPLY gating
        self.waiting_for_reply = False
        self.last_upsend_sent_time = 0.0
        self.pending_upsend: Optional[str] = None  # last sent UPSEND payload

        # Motion schedule times
        self.t_open_end: Optional[float] = None
        self.t_hold_end: Optional[float] = None
        self.t_close_end: Optional[float] = None

        # Thread safety (single-threaded loop; lock for ser writes)
        self._tx_lock = threading.Lock()

    # --- Serial helpers ---

    @staticmethod
    def _to_parity(p: str):
        p = p.upper()
        if p == "N":
            return serial.PARITY_NONE
        if p == "E":
            return serial.PARITY_EVEN
        if p == "O":
            return serial.PARITY_ODD
        if p == "M":
            return serial.PARITY_MARK
        if p == "S":
            return serial.PARITY_SPACE
        raise ValueError("Invalid parity")

    @staticmethod
    def _to_stopbits(sb: int):
        if sb == 1:
            return serial.STOPBITS_ONE
        if sb == 2:
            return serial.STOPBITS_TWO
        raise ValueError("Invalid stop bits")

    @staticmethod
    def _to_bytesize(bs: int):
        mapping = {
            5: serial.FIVEBITS,
            6: serial.SIXBITS,
            7: serial.SEVENBITS,
            8: serial.EIGHTBITS,
        }
        if bs not in mapping:
            raise ValueError("Invalid byte size")
        return mapping[bs]

    def _log_tx(self, s: str):
        if self.cfg.log_io:
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] TX: {repr(s)}")

    def _log_rx(self, s: str):
        if self.cfg.log_io:
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] RX: {repr(s)}")

    def send_line(self, line: str):
        # Append ESC terminator
        payload = line.encode("ascii") + ESC
        with self._tx_lock:
            self._log_tx(line + "\\x1b")
            self.ser.write(payload)
            self.ser.flush()

    # --- Protocol primitives ---

    def send_result_ok(self):
        self.send_line("AT+RESULT,r:1")

    def send_result_fail(self):
        self.send_line("AT+RESULT,r:0")

    def send_result_payload(self, kv_payload: str):
        # For d:0 case: send payload rather than r:1
        self.send_line(f"AT+RESULT,{kv_payload}")

    def send_upsend_status(self):
        # If we require REPLY and already waiting, optionally retry or skip
        if self.cfg.upsend_requires_reply and self.waiting_for_reply:
            if self.cfg.upsend_retry_s is not None:
                if now() - self.last_upsend_sent_time >= self.cfg.upsend_retry_s:
                    # re-send the last pending UPSEND
                    if self.pending_upsend is not None:
                        self._emit_upsend(self.pending_upsend, resend=True)
            return

        payload = self.state.format_status_triplet()
        self._emit_upsend(payload, resend=False)

    def _emit_upsend(self, payload: str, resend: bool):
        # Emit AT+UPSEND with payload m:*,c:*,n:*
        self.send_line(f"AT+UPSEND,{payload}")
        self.last_upsend_sent_time = now()
        self.pending_upsend = payload
        if self.cfg.upsend_requires_reply:
            self.waiting_for_reply = True
        if self.cfg.log_io and resend:
            ts = time.strftime("%H:%M:%S")
            print(f"[{ts}] Note: re-sent UPSEND awaiting REPLY")

    # --- Parsing and processing ---

    def run(self):
        print(f"Autoslide emulator running on {self.cfg.port} @ {self.cfg.baud} 8{self.cfg.parity}{self.cfg.stopbits}")
        print("Press Ctrl+C to exit.")
        try:
            while True:
                self._poll_rx()
                self._tick_motion()
                self._tick_periodic_status()
        except KeyboardInterrupt:
            print("\nExiting.")
        finally:
            self.ser.close()

    def _poll_rx(self):
        try:
            data = self.ser.read(1024)
        except serial.SerialException as e:
            print(f"Serial error: {e}", file=sys.stderr)
            time.sleep(0.5)
            return

        if not data:
            return

        self.rx_buf.extend(data)
        # Split on ESC terminator; process complete frames
        while True:
            try:
                idx = self.rx_buf.index(ESC)
            except ValueError:
                break  # no complete frame yet

            frame = self.rx_buf[:idx]
            del self.rx_buf[: idx + 1]
            if not frame:
                continue

            try:
                line = frame.decode("ascii", errors="strict")
            except UnicodeDecodeError:
                continue

            self._log_rx(line + "\\x1b")
            self._handle_line(line.strip())

    def _handle_line(self, line: str):
        # Expect "AT+<CMD>,<payload>"
        if not line.startswith("AT+"):
            return

        if line.startswith("AT+REPLY"):
            # Controller acknowledges our UPSEND
            ok = self._parse_ack(line)
            if ok:
                self.waiting_for_reply = False
                self.pending_upsend = None
            return

        # If we are awaiting REPLY, ignore other commands (as per guide)
        if self.waiting_for_reply:
            # Emulate device that won't respond to other commands until REPLY
            if self.cfg.log_io:
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] Info: ignoring command while awaiting AT+REPLY.")
            return

        if line.startswith("AT+UPDATE"):
            ok = self._handle_update(line)
            return

        # Unknown/unsupported command
        self.send_result_fail()

    def _parse_kv_payload(self, payload: str) -> Dict[str, str]:
        d: Dict[str, str] = {}
        if not payload:
            return d
        parts = [p for p in payload.split(",") if p]
        for p in parts:
            if ":" not in p:
                raise ValueError("Bad kv (missing colon)")
            k, v = p.split(":", 1)
            if not k or v is None:
                raise ValueError("Bad kv (empty)")
            d[k] = v
        return d

    def _split_cmd_payload(self, line: str) -> Tuple[str, str]:
        # "AT+CMD,<payload>" or "AT+CMD" (no payload)
        if "," in line:
            cmd, rest = line.split(",", 1)
        else:
            cmd, rest = line, ""
        return cmd, rest

    def _parse_ack(self, line: str) -> bool:
        # Expect AT+REPLY,r:1
        _, payload = self._split_cmd_payload(line)
        try:
            kv = self._parse_kv_payload(payload)
        except ValueError:
            return False
        return kv.get("r") == "1"

    def _handle_update(self, line: str) -> bool:
        _, payload = self._split_cmd_payload(line)

        # Enforce single kv
        try:
            kv = self._parse_kv_payload(payload)
        except ValueError:
            self.send_result_fail()
            return False

        if len(kv) != 1:
            self.send_result_fail()
            return False

        key, sval = next(iter(kv.items()))
        # Route by key
        handlers = {
            "a": self._update_a_mode,
            "e": self._update_e_open_speed,
            "g": self._update_g_secure_pet,
            "j": self._update_j_hold,
            "C": self._update_C_open_force,
            "z": self._update_z_close_force,
            "A": self._update_A_close_end_force,
            "b": self._update_b_trigger,
            "d": self._update_d_status_req,
        }
        fn = handlers.get(key)
        if not fn:
            self.send_result_fail()
            return False

        return fn(sval)

    # --- UPDATE handlers ---

    def _update_a_mode(self, sval: str) -> bool:
        if not sval.isdigit():
            self.send_result_fail()
            return False
        val = int(sval)
        if val < 0 or val > 3:
            self.send_result_fail()
            return False
        self.state.a_mode = val
        # Update lock (derived), and optionally broadcast current status
        self.send_result_ok()
        # Push status to reflect lock change
        self.send_upsend_status()
        return True

    def _update_e_open_speed(self, sval: str) -> bool:
        if not sval.isdigit():
            self.send_result_fail()
            return False
        val = int(sval)
        if val not in (0, 1):
            self.send_result_fail()
            return False
        self.state.e_open_speed = val
        self.send_result_ok()
        return True

    def _update_g_secure_pet(self, sval: str) -> bool:
        if not sval.isdigit():
            self.send_result_fail()
            return False
        val = int(sval)
        if val not in (0, 1):
            self.send_result_fail()
            return False
        self.state.g_secure_pet = val
        self.send_result_ok()
        return True

    def _update_j_hold(self, sval: str) -> bool:
        # MUST be two digits 00..25
        if len(sval) != 2 or not sval.isdigit():
            self.send_result_fail()
            return False
        val = int(sval)
        if val < 0 or val > 25:
            self.send_result_fail()
            return False
        self.state.j_hold_s = val
        self.send_result_ok()
        return True

    def _update_C_open_force(self, sval: str) -> bool:
        if not sval.isdigit():
            self.send_result_fail()
            return False
        val = int(sval)
        if val < 0 or val > 7:
            self.send_result_fail()
            return False
        self.state.C_open_force = val
        self.send_result_ok()
        return True

    def _update_z_close_force(self, sval: str) -> bool:
        if not sval.isdigit():
            self.send_result_fail()
            return False
        val = int(sval)
        if val < 0 or val > 7:
            self.send_result_fail()
            return False
        self.state.z_close_force = val
        self.send_result_ok()
        return True

    def _update_A_close_end_force(self, sval: str) -> bool:
        if not sval.isdigit():
            self.send_result_fail()
            return False
        val = int(sval)
        if val < 0 or val > 7:
            self.send_result_fail()
            return False
        self.state.A_close_end_force = val
        self.send_result_ok()
        return True

    def _update_b_trigger(self, sval: str) -> bool:
        if not sval.isdigit():
            self.send_result_fail()
            return False
        trigger = int(sval)
        if trigger < 0 or trigger > 3:
            self.send_result_fail()
            return False

        # If locked, fail
        if self.state.c_locked == 1:
            self.send_result_fail()
            return False

        # Map b -> n: 0:MASTER->2, 1:INDOOR->1, 2:PET->3, 3:STACK->4
        mapping = {0: 2, 1: 1, 2: 3, 3: 4}
        self.state.n_trigger = mapping[trigger]  # not using 5 (SOFTWARE) here because b encodes source

        # Start motion cycle if currently stopped
        if self.state.m_motion == 0:
            self._start_open_cycle()

        self.send_result_ok()
        return True

    def _update_d_status_req(self, sval: str) -> bool:
        if not sval.isdigit():
            self.send_result_fail()
            return False
        val = int(sval)
        if val not in (0, 1):
            self.send_result_fail()
            return False

        if val == 0:
            # Return full settings payload (per example, no r:1 key, just the keys below)
            self.send_result_payload(self.state.format_settings_payload())
            return True

        if val == 1:
            # Version info unspecified in the guide; acknowledge success.
            self.send_result_ok()
            return True

        self.send_result_fail()
        return False

    # --- Motion state machine ---

    def _start_open_cycle(self):
        # Transition to OPENING
        self.state.m_motion = 1
        # Time based on speed
        tnow = now()
        if self.state.e_open_speed == 0:
            open_dur = self.cfg.open_time_fast_s
        else:
            open_dur = self.cfg.open_time_slow_s

        if self.state.e_open_speed == 0:
            close_dur = self.cfg.close_time_fast_s
        else:
            close_dur = self.cfg.close_time_slow_s

        self.t_open_end = tnow + open_dur
        self.t_hold_end = self.t_open_end + float(self.state.j_hold_s)
        self.t_close_end = self.t_hold_end + close_dur
        # Broadcast UPSEND for state change
        self.send_upsend_status()

    def _tick_motion(self):
        if self.state.m_motion == 0 and self.t_open_end is None:
            return  # idle

        tnow = now()

        # OPENING -> OPEN (STOPPED)
        if self.state.m_motion == 1 and self.t_open_end is not None and tnow >= self.t_open_end:
            self.state.m_motion = 0
            self.state.pos = 1.0
            # UPSEND for STOPPED at open
            self.send_upsend_status()

        # HOLD -> start CLOSING
        if self.state.pos >= 1.0 and self.t_hold_end is not None and tnow >= self.t_hold_end and self.state.m_motion == 0:
            self.state.m_motion = 2
            self.send_upsend_status()

        # CLOSING -> STOPPED (closed)
        if self.state.m_motion == 2 and self.t_close_end is not None and tnow >= self.t_close_end:
            self.state.m_motion = 0
            self.state.pos = 0.0
            # Reset trigger to NONE at end of cycle
            self.state.n_trigger = 0
            # Clear schedule
            self.t_open_end = None
            self.t_hold_end = None
            self.t_close_end = None
            self.send_upsend_status()

    def _tick_periodic_status(self):
        tnow = now()
        if tnow - self.last_status_time >= self.cfg.status_interval_s:
            self.last_status_time = tnow
            self.send_upsend_status()


def parse_args() -> Config:
    p = argparse.ArgumentParser(description="Autoslide serial emulator (AT+ protocol).")
    p.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0 or COM5)")
    p.add_argument("--baud", type=int, default=9600)
    p.add_argument("--bytesize", type=int, default=8, choices=(5, 6, 7, 8))
    p.add_argument("--parity", default="N", choices=("N", "E", "O", "M", "S"))
    p.add_argument("--stopbits", type=int, default=1, choices=(1, 2))
    p.add_argument("--status-interval", type=float, default=2.0, help="UPSEND interval (s)")
    p.add_argument("--open-fast", type=float, default=2.0, help="Open duration when e=0 (s)")
    p.add_argument("--open-slow", type=float, default=4.0, help="Open duration when e=1 (s)")
    p.add_argument("--close-fast", type=float, default=2.0, help="Close duration when e=0 (s)")
    p.add_argument("--close-slow", type=float, default=4.0, help="Close duration when e=1 (s)")
    p.add_argument("--no-log-io", action="store_true", help="Disable RX/TX logging")
    p.add_argument("--no-upsend-reply-gate", action="store_true", help="Do not gate command handling on REPLY")
    p.add_argument("--upsend-retry", type=float, default=None, help="Resend UPSEND if not acked within this many seconds")
    args = p.parse_args()
    return Config(
        port=args.port,
        baud=args.baud,
        bytesize=args.bytesize,
        parity=args.parity,
        stopbits=args.stopbits,
        status_interval_s=args.status_interval,
        open_time_fast_s=args.open_fast,
        open_time_slow_s=args.open_slow,
        close_time_fast_s=args.close_fast,
        close_time_slow_s=args.close_slow,
        log_io=not args.no_log_io,
        upsend_requires_reply=not args.no_upsend_reply_gate,
        upsend_retry_s=args.upsend_retry,
    )


def main():
    cfg = parse_args()
    emu = AutoslideEmulator(cfg)
    emu.run()


if __name__ == "__main__":
    main()