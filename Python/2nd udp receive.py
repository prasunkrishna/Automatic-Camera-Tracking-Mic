#!/usr/bin/env python3
"""
i2s_udp_receiver.py
Receive PCM16 mono audio over UDP from one or more ESP transmitters and play/mix it.

Packet format (from ESP firmware):
    [0]         : sender_id        (uint8)
    [1:]        : little-endian signed 16-bit samples
    packet_len  : 1 + 2*SAMPLES_PER_PACKET

Defaults match:
    SAMPLE_RATE = 16000 Hz
    PACKET_MS   = 30 ms
    SAMPLES_PER_PACKET = 480
    PACKET_SIZE = 961 bytes
"""

import socket
import struct
import threading
from collections import defaultdict, deque
import numpy as np
import sounddevice as sd
import time

# --------------------------- CONFIG ---------------------------

UDP_IP            = "0.0.0.0"
UDP_PORT          = 4210

SAMPLE_RATE       = 16000         # Must match ESP firmware
PACKET_MS         = 30            # Must match ESP firmware
SAMPLES_PER_PACKET= int(SAMPLE_RATE * PACKET_MS / 1000)  # 480
PACKET_SIZE       = 1 + 2 * SAMPLES_PER_PACKET           # 961

# Gain trim after all processing (1.0 = unity)
OUTPUT_GAIN       = 1.5           # try 1.5–2.0 if too quiet; watch clipping

# Jitter buffering
JITTER_PACKETS    = 2             # hold ~60ms of audio per sender before playback

# Noise gate (simple)
ENABLE_NOISE_GATE = True
GATE_THRESHOLD_RMS= 0.01          # below this RMS -> attenuate
GATE_ATTENUATION  = 0.05          # scale when gated (0 = full mute)

# Gentle smoothing across packet edges (IIR)
ENABLE_SMOOTHING  = True
SMOOTH_ALPHA      = 0.2           # 0=no smoothing, 1=heavy smoothing (use ~0.1–0.3)

# Print debugging
VERBOSE_PACKETS   = False         # print each packet's first few samples
PRINT_LEVEL_SEC   = 5             # print average RMS every N sec (0 disables)

# Restrict to specific senders? None = accept all
SENDER_WHITELIST  = None          # e.g., {1,2} to allow only ID 1 & 2

# --------------------------------------------------------------


# Buffers: per-sender deques holding recent samples (float32)
buffers = defaultdict(lambda: deque(maxlen=JITTER_PACKETS * SAMPLES_PER_PACKET))
active_senders = set()

# Per-sender smoothing history
prev_out = defaultdict(float)

# Lock for thread-safe buffer ops
buf_lock = threading.Lock()


def decode_packet(data):
    """Return (sender_id, numpy.int16 array) or (None, None) if bad."""
    if len(data) != PACKET_SIZE:
        return None, None
    sender_id = data[0]
    if SENDER_WHITELIST and sender_id not in SENDER_WHITELIST:
        return None, None
    # unpack little-endian signed 16-bit
    fmt = "<{}h".format(SAMPLES_PER_PACKET)
    samples = struct.unpack(fmt, data[1:])
    return sender_id, np.array(samples, dtype=np.int16)


def apply_noise_gate(x):
    """If RMS below threshold, attenuate."""
    if not ENABLE_NOISE_GATE:
        return x
    rms = np.sqrt(np.mean(np.square(x)))
    if rms < GATE_THRESHOLD_RMS:
        return x * GATE_ATTENUATION
    return x


def apply_smoothing(sender_id, x):
    """Simple 1st-order IIR smoothing across packet edges."""
    if not ENABLE_SMOOTHING:
        return x
    alpha = SMOOTH_ALPHA
    y = np.empty_like(x)
    prev = prev_out[sender_id]
    for i, s in enumerate(x):
        prev = (1 - alpha) * prev + alpha * s
        y[i] = prev
    prev_out[sender_id] = prev
    return y


def udp_listener(stop_event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    sock.setblocking(False)
    print(f"Listening on UDP {UDP_IP}:{UDP_PORT} ...")

    last_print = time.time()

    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(PACKET_SIZE)
        except BlockingIOError:
            time.sleep(0.001)
            continue
        except Exception as e:
            print(f"[UDP ERROR] {e}")
            continue

        sender_id, pcm16 = decode_packet(data)
        if sender_id is None:
            continue

        # Convert to float -1..+1
        f = pcm16.astype(np.float32) / 32768.0

        # Post-process per config
        f = apply_noise_gate(f)
        f = apply_smoothing(sender_id, f)

        with buf_lock:
            buffers[sender_id].extend(f.tolist())
            active_senders.add(sender_id)

        if VERBOSE_PACKETS:
            print(f"[Sender {sender_id}] {addr[0]} → {pcm16[:5].tolist()} ...")

        # Periodic level diagnostics
        if PRINT_LEVEL_SEC > 0 and (time.time() - last_print) >= PRINT_LEVEL_SEC:
            with buf_lock:
                for sid in active_senders:
                    arr = np.array(buffers[sid], dtype=np.float32)
                    if arr.size:
                        rms = np.sqrt(np.mean(arr * arr))
                        peak = np.max(np.abs(arr))
                        print(f"[lvl] sender {sid}: RMS={rms:.3f} Peak={peak:.3f}")
            last_print = time.time()

    sock.close()


def audio_callback(outdata, frames, time_info, status):
    mixed = np.zeros(frames, dtype=np.float32)

    with buf_lock:
        sender_list = list(active_senders)

    for sid in sender_list:
        with buf_lock:
            sbuf = buffers[sid]
            if len(sbuf) >= frames:
                # Pull contiguous frames
                s = [sbuf.popleft() for _ in range(frames)]
            else:
                # Underflow -> pad silence
                need = frames - len(sbuf)
                s = list(sbuf)
                sbuf.clear()
                s.extend([0.0] * need)
        mixed += np.array(s, dtype=np.float32)

    # Scale down if mixing >1 sender
    if len(sender_list) > 1:
        mixed /= len(sender_list)

    # Output gain
    mixed *= OUTPUT_GAIN

    # Clip final
    mixed = np.clip(mixed, -1.0, 1.0)
    outdata[:] = mixed.reshape(-1, 1)


def main():
    stop_event = threading.Event()
    t = threading.Thread(target=udp_listener, args=(stop_event,), daemon=True)
    t.start()

    print("🔊 Starting audio output (Ctrl+C to stop)")
    try:
        with sd.OutputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            dtype='float32',
            callback=audio_callback,
            blocksize=SAMPLES_PER_PACKET
        ):
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        stop_event.set()
        t.join()


if __name__ == "__main__":
    main()
