# The Keyence Scanner: Lessons from Reverse-Engineering

## The Problem

The Keyence LJ-V7200 laser scanner has no public protocol documentation. Keyence distributes their driver as a Windows DLL. To use the scanner on Ubuntu with Python, we had to reverse-engineer the TCP protocol from a ROS-Industrial C++ driver and trial-and-error testing.

This document records what we got wrong, what we learned, and what the final working architecture looks like.

---

## Mistake 1: Continuous Streaming vs. On-Demand Requests

**What we tried first:** Open a TCP connection, send a trigger command, and read profiles in a continuous loop. The scanner blasts data at over 1000 profiles per second. We tried to keep up with a `while True` loop.

**Why it failed:** The scanner's firehose overwhelmed the OS TCP buffer (64-128KB). When the buffer filled, the scanner dropped the connection. We also tried draining the buffer with non-blocking reads before each capture, but the `while True` drain loop would sometimes never exit because new data arrived faster than we could drain it, freezing the GUI.

**The correct approach:** The scanner must be used in **on-demand request-response mode**. Open the connection once. Send the trigger command only when you need a profile. Read exactly one response. The scanner is silent between requests. No streaming. No buffer overflow. No drain loops.

**The insight:** The `01 01 00 00` bytes at the end of the trigger command put the scanner in continuous streaming mode. We initially thought this was the only mode. Later we discovered that `00 00 00 00` requests a single profile and keeps the scanner silent afterward. For batch capture, we keep the connection open and send `01 01 00 00` for each request — the scanner responds with one profile and waits for the next trigger. The connection stays alive but idle between requests.

---

## Mistake 2: The Double-Line Artifact (20-Bit Unpacking)

**What we saw:** The scanner profile appeared as two separate parallel lines instead of one continuous surface. The Z values alternated between two distinct numbers (e.g., 0.0 and 0.26208).

**The cause:** The Keyence packs two 20-bit signed depth values into 5 bytes. The middle byte (`b2`) is shared — its high nibble belongs to P0, its low nibble belongs to P1. Our initial unpacking misassigned these nibbles, causing even-indexed and odd-indexed points to get different depth values.

**The fix:** The correct little-endian unpacking is:

```python
p0 = ((b2 & 0x0F) << 16) | (b1 << 8) | b0
p1 = (b4 << 12) | (b3 << 4) | (b2 >> 4)
```

P0 gets `b2` low nibble as its highest 4 bits, `b1` as middle, `b0` as lowest. P1 gets `b4` as highest 8 bits, `b3` as middle, `b2` high nibble as lowest 4 bits.

**Verification:** We plotted the corrected profile with matplotlib. The surface appeared as a single continuous line with realistic depth variation. The Gemini confirmed the byte order by analyzing the ROS-Industrial C++ source.

---

## Mistake 3: The Phantom 0.26208 Depth

**What we saw:** Even after fixing the unpacking, the profile contained a depth value of exactly 0.26208 meters. This appeared even when the scanner was pointed at empty space.

**The cause:** The Keyence uses `-524285` as an error code for out-of-range or dead pixels. Our unit conversion multiplied this by `0.0000005` (meters per step) and then flipped the sign (optical convention), producing `+0.26208`. We were displaying the scanner's error flag as if it were a real measurement.

**The physics check:** The LJ-V7200 has a measurement range of 100mm ±20mm. The maximum physical distance it can measure is 120mm. A value of 262mm is physically impossible — it's more than double the sensor's range. The value can only be an error code.

**The fix:** Filter invalid points before converting to meters:

```python
INVALID_LOWER_BOUND = -524280
valid_mask = (raw_z > INVALID_LOWER_BOUND) & (raw_z != 0)
```

This strips error codes at the source. Only genuine surface measurements reach the application.

---

## Mistake 4: The 60-Byte ACK Confusion

**What we saw:** When using the `00 00 00 00` single-profile command, the scanner responded with exactly 60 bytes instead of the expected 2000+ byte profile.

**The cause:** The 60-byte response is a **Command Acknowledgment (ACK)** — the scanner confirming it received the trigger. The actual profile data was supposed to come from a second command (`02 00 00 00` fetch request). We were treating the ACK as profile data and trying to parse it.

**The confusion:** The Gemini initially suggested a two-step handshake (trigger then fetch) based on documentation for the LJ-8000 series, which supports this mode. The LJ-7000 series does not. The `02 00 00 00` fetch command does not return profile data on the LJ-V7200.

**The resolution:** The LJ-V7200 requires the `01 01 00 00` streaming trigger. With this command, the scanner returns a full profile immediately — no separate fetch step needed. The 60-byte ACK was a red herring caused by using the wrong command bytes for this scanner model.

---

## Mistake 5: The Connection Lifecycle

**What we tried:** Connect for each profile, capture, disconnect. Or connect once and read continuously with a drain loop.

**Why both failed:** Per-profile connect/disconnect adds 2-5ms of TCP handshake overhead per frame — fine for occasional use, but wasteful. Continuous reading with drain loops caused buffer overflows and GUI freezes.

**The correct approach:** **Persistent connection, on-demand requests.** Open the socket once at the start of the welding pass. Send a trigger only when you need a profile. Read one response. The scanner waits silently between triggers. Close the socket when the pass is complete.

This is the same pattern as the RTDE driver for the UR robot: the connection is persistent but idle. Data flows only when requested. No streaming. No buffer management. No drain loops. No timeouts.

---

## What the Final Architecture Looks Like

```
Application calls capture_profiles(count=200, interval=0.1)
    ↓
Driver opens TCP socket (once)
    ↓
For each profile:
    sendall(KEYENCE_TRIGGER_REQUEST)
    recv(4) → response size
    recv(size) → profile data
    unpack 20-bit → raw Z values
    filter invalid points (-524285)
    convert to meters
    flip to optical convention
    return (points, colors)
    sleep(interval)
    ↓
Driver closes TCP socket
```

**Key constants:**
- `KEYENCE_FUNDAMENTAL_LENGTH_UNIT = 1e-8` (0.01 µm per step)
- `KEYENCE_INVALID_LOWER_BOUND = -524280` (error code threshold)
- Trigger command ends with `01 01 00 00` (streaming mode, single response)
- Response header is 84 bytes, profile data follows
- 800 points per profile for the LJ-V7200

---

## Lessons for Future Sensor Integration

1. **Don't assume continuous streaming is the only mode.** Many industrial sensors support on-demand request-response but don't document it publicly.

2. **Error codes can look like real data after unit conversion.** Always identify and filter error flags before scaling. Check physical limits — if a value is physically impossible, it's an error code.

3. **Persistent connections are better than per-request connections.** Opening a TCP socket takes milliseconds. Keep it open for the duration of the task.

4. **Sleep between requests, not during reads.** The socket should be read as soon as data arrives. Pacing should happen between requests, not while data is waiting in the buffer.

5. **Reverse-engineering requires multiple sources.** The ROS-Industrial C++ driver gave us the protocol structure. The Gemini gave us the byte order correction and the error code identification. Physical testing with a target object confirmed the results. No single source had the complete answer.

6. **When the sensor has no public documentation, every value is suspect until verified against physical reality.** The `0.26208` was mathematically correct but physically impossible. The double line was algorithmically valid but geometrically wrong. Only testing against a real object could distinguish correct behavior from plausible-looking errors.

---

*Document version 1.0*
*Hatch (孵) — Keyence LJ-V7200 Integration Notes*