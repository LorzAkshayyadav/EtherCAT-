# EtherCAT Distributed Clocks (DC) with the IgH EtherCAT Master

A practical guide to running EtherCAT slaves in **DC SYNC0 mode** with the IgH (EtherLab) master. It is written for a setup of 5 slaves, with a PIC32 + LAN9252 RFID slave at position 0, cycling at **1 ms**.

---

## Table of Contents

1. [What DC Is (Simple Explanation)](#1-what-dc-is-simple-explanation)
2. [How DC Works Step by Step](#2-how-dc-works-step-by-step)
3. [Key Terms](#3-key-terms)
4. [DC Register Reference](#4-dc-register-reference)
5. [Slave-Side Requirements (ESI + Firmware)](#5-slave-side-requirements-esi--firmware)
6. [Master-Side Configuration (IgH API)](#6-master-side-configuration-igh-api)
7. [Choosing the Reference Clock](#7-choosing-the-reference-clock)
8. [Configuring Multiple Slaves (5-Slave Example)](#8-configuring-multiple-slaves-5-slave-example)
9. [The Cyclic Loop](#9-the-cyclic-loop)
10. [Choosing the SYNC0 Shift Time](#10-choosing-the-sync0-shift-time)
11. [Verifying DC Is Working](#11-verifying-dc-is-working)
12. [Troubleshooting](#12-troubleshooting)
13. [Linux Real-Time Tips](#13-linux-real-time-tips)
14. [Quick Checklist](#14-quick-checklist)

---

## 1. What DC Is (Simple Explanation)

Imagine every slave is a person wearing a wristwatch. You want all of them to clap at **exactly the same moment** every 1 ms. Three things get in the way:

| Problem | Why it happens |
|---|---|
| Watches show different times | Each slave's clock started counting at power-on, at a different moment |
| Watches run at different speeds | Every slave's quartz crystal is slightly off (e.g. ±25 ppm) |
| Messages arrive late | The frame takes time to travel through the cable and each slave |

Distributed Clocks solves all three:

- **Delays are measured** once at startup.
- **All clocks are set to the same time** once at startup.
- **Clocks are continuously corrected** every cycle so they don't drift apart.
- **SYNC0 fires** a hardware pulse on every slave at the same instant (typically within ~100 ns of each other).

```
         PC clock (application time)
              │  ecrt_master_sync_reference_clock()
              ▼
     ┌──────────────────┐
     │ Reference slave  │  ← the "master watch" (default: first DC-capable slave)
     └──────────────────┘
              │  ecrt_master_sync_slave_clocks()
     ┌────────┼────────┬────────┐
     ▼        ▼        ▼        ▼
  Slave 1  Slave 2  Slave 3  Slave 4   ← follow the reference clock
```

---

## 2. How DC Works Step by Step

### Step 1: Propagation delay measurement (startup, done by the master)

The master sends a broadcast write to register `0x0900`. Every slave latches the time the frame passed each of its ports, both going out and coming back. From these timestamps the master calculates the cable and processing delay from the reference clock to every slave. It then writes the result into **`0x0928` (System Time Delay)** of each slave.

### Step 2: Offset compensation (startup, done by the master)

The master reads each slave's local time. It calculates how far each one is from the reference (or from the application time), and writes the correction into **`0x0920` (System Time Offset)**. After this, every slave's **`0x0910` (System Time)** shows approximately the same value.

### Step 3: Drift compensation (every cycle)

Every cycle, the master sends a special datagram that **reads `0x0910` from the reference clock** and **writes that value into `0x0910` of every slave after it**. Each slave's ESC compares the received time with its own:

```
difference = local_time − (received_time + propagation_delay)
```

The difference is shown in **`0x092C` (System Time Difference)**. The ESC's internal control loop speeds up or slows down the local clock slightly until the difference is close to zero.

`ecrt_master_sync_reference_clock()` additionally writes the **PC's application time** into the reference slave. The reference follows the PC, and all other slaves follow the reference.

### Step 4: SYNC0 pulse generation

Once the clocks agree, the master programs each slave's **SYNC unit** with:

- a **start time** (`0x0990`): when the first pulse should occur, and
- a **cycle time** (`0x09A0`): how often it repeats (1,000,000 ns = 1 ms).

Then it activates the unit (`0x0981`). From then on, every slave raises its **SYNC0 pin** at the same instant every cycle. The slave's microcontroller (PIC32) receives SYNC0 as an interrupt and runs its application, for example copying outputs to hardware and sampling inputs.

---

## 3. Key Terms

| Term | Meaning |
|---|---|
| **System Time** | 64-bit time in nanoseconds since 1 Jan 2000, kept by each DC-capable ESC |
| **Reference clock** | The slave whose clock all others follow. IgH default: first DC-capable slave on the bus |
| **Application time** | The PC's time, given to the master with `ecrt_master_application_time()` |
| **SYNC0 / SYNC1** | Hardware pulse outputs of the ESC, generated from the synchronised clock |
| **AssignActivate** | 16-bit value from the ESI that tells the master how to program `0x0980`/`0x0981` |
| **Cycle time** | Interval between SYNC0 pulses (here 1 ms = 1,000,000 ns) |
| **Shift time** | Offset of the SYNC0 pulse relative to the cycle start (application time grid) |
| **ESC** | EtherCAT Slave Controller chip (here the Microchip LAN9252) |
| **SSC** | Beckhoff Slave Stack Code, the slave firmware stack running on the PIC32 |
| **Free run / SM sync / DC sync** | The three slave sync modes: no sync, sync to frame arrival, sync to DC SYNC0 |

---

## 4. DC Register Reference

All registers are in the ESC (LAN9252) and can be read with `ethercat reg_read`.

### 4.1 Time and synchronisation registers

| Address | Size | Name | Description |
|---|---|---|---|
| `0x0900` | 4 × 32 bit | Receive Time Port 0–3 | Time the frame passed each port. Used for delay measurement. Written by a broadcast to `0x0900` |
| `0x0910` | 64 bit | **System Time** | The synchronised DC time (ns). The cyclic sync datagram writes here |
| `0x0918` | 64 bit | Receive Time ECAT Processing Unit | Local time when the delay-measurement frame was processed |
| `0x0920` | 64 bit | **System Time Offset** | Correction between local time and system time. Set by the master at startup |
| `0x0928` | 32 bit | **System Time Delay** | Propagation delay from the reference clock to this slave (ns) |
| `0x092C` | 32 bit | **System Time Difference** | Current sync error. **Sign-magnitude**: bit 31 = sign, bits 30..0 = magnitude in ns |
| `0x0930` | 16 bit | Speed Counter Start | Drift-control loop bandwidth |
| `0x0932` | 16 bit | Speed Counter Diff | Current speed correction applied to the local clock |
| `0x0934` | 8 bit | System Time Diff Filter Depth | Filter depth for the time-difference measurement |
| `0x0935` | 8 bit | Speed Counter Filter Depth | Filter depth for the speed correction |

**Reading `0x092C`:** bit 31 = 0 means the local time is ahead of or equal to the received time; bit 31 = 1 means it is behind. Use `-t sm32` in the `ethercat` tool to decode this automatically.

### 4.2 SYNC unit registers

| Address | Size | Name | Description |
|---|---|---|---|
| `0x0980` | 8 bit | Cyclic Unit Control | Bit 0: SYNC out unit controlled by 0 = ECAT (master), 1 = PDI (µC) |
| `0x0981` | 8 bit | **Activation** | Bit 0: cyclic operation enable. Bit 1: SYNC0 enable. Bit 2: SYNC1 enable |
| `0x0982` | 16 bit | Pulse Length of SYNC Signals | SYNC pulse width in units of 10 ns (0 = acknowledge mode). Usually set from the EEPROM |
| `0x098E` | 8 bit | SYNC0 Status | Bit 0 set when a SYNC0 pulse occurred (acknowledge mode) |
| `0x098F` | 8 bit | SYNC1 Status | Same for SYNC1 |
| `0x0990` | 64 bit | **Start Time Cyclic Operation / Next SYNC0 Pulse** | Write: start time. Read: time of the next SYNC0 pulse |
| `0x0998` | 64 bit | Next SYNC1 Pulse | Time of the next SYNC1 pulse |
| `0x09A0` | 32 bit | **SYNC0 Cycle Time** | SYNC0 period in ns (1,000,000 for 1 ms) |
| `0x09A4` | 32 bit | SYNC1 Cycle Time | SYNC1 period/offset in ns |

### 4.3 How AssignActivate maps to registers

`AssignActivate` is a 16-bit value from the ESI file:

- **High byte → `0x0981`** (Activation)
- **Low byte → `0x0980`** (Cyclic Unit Control)

| AssignActivate | `0x0981` | Meaning |
|---|---|---|
| `0x0000` | `0x00` | DC off (free run or SM-synchronous) |
| `0x0300` | `0x03` | Cyclic operation + **SYNC0** (most common) |
| `0x0700` | `0x07` | Cyclic operation + SYNC0 + SYNC1 |

Always take the value from your slave's ESI file (`<Dc><OpMode><AssignActivate>`).

### 4.4 Related status registers

| Address | Name | Use |
|---|---|---|
| `0x0130` | AL Status | Current state (1 = INIT, 2 = PREOP, 4 = SAFEOP, 8 = OP; bit 4 = error) |
| `0x0134` | AL Status Code | Error code if the slave refused a state change (see [Troubleshooting](#12-troubleshooting)) |

---

## 5. Slave-Side Requirements (ESI + Firmware)

DC only works if the **slave** supports it. The master cannot force it.

### 5.1 ESI file: `<Dc>` section

The slave's ESI (XML) must describe its DC modes. The master tool and the slave's EEPROM (SII) must match.

```xml
<Dc>
  <OpMode>
    <Name>DcSync</Name>
    <Desc>DC-Synchron (SYNC0)</Desc>
    <AssignActivate>#x300</AssignActivate>
    <CycleTimeSync0 Factor="1">0</CycleTimeSync0>
    <ShiftTimeSync0>0</ShiftTimeSync0>
  </OpMode>
  <OpMode>
    <Name>SM-Synchron</Name>
    <Desc>SM-Synchron</Desc>
    <AssignActivate>#x0</AssignActivate>
  </OpMode>
</Dc>
```

After changing the ESI, **regenerate and re-flash the EEPROM (SII)**, for example with `ethercat sii_write`.

### 5.2 Firmware (Beckhoff SSC on PIC32 + LAN9252)

| Item | Requirement |
|---|---|
| SSC define | `DC_SUPPORTED = 1` in the SSC Tool configuration |
| SYNC0 hardware | LAN9252 SYNC0 pin wired to a PIC32 external interrupt pin |
| SYNC0 ISR | SYNC0 interrupt calls the SSC `Sync0_Isr()` handler |
| Object `0x1C32` | Sync Manager 2 (outputs) parameters must exist |
| Object `0x1C33` | Sync Manager 3 (inputs) parameters must exist |
| Execution time | The application in the SYNC0 ISR must finish well within 1 ms |

### 5.3 Sync Manager parameter objects (`0x1C32` / `0x1C33`)

| Subindex | Name | Notes |
|---|---|---|
| `0x01` | Sync mode | 0 = free run, 1 = SM-synchronous, **2 = DC SYNC0**, 3 = DC SYNC1 |
| `0x02` | Cycle time | Cycle time in ns |
| `0x03` | Shift time | Shift in ns |
| `0x04` | Sync modes supported | Bit field of supported modes |
| `0x05` | Minimum cycle time | Fastest cycle the slave supports. **Must be ≤ 1,000,000 ns** |
| `0x06` | Calc and copy time | Time the slave needs to process data |
| `0x09` | Delay time | Hardware delay |
| `0x0A` | SYNC0 cycle time | Current SYNC0 cycle |
| `0x0B` | **SM event missed counter** | Should stay at 0 (or stop increasing) in OP |
| `0x0C` | **Cycle time too small counter** | Should stay at 0 |
| `0x20` | Sync error | TRUE if synchronisation was lost |

Some SSC builds require the master to write the sync mode explicitly:

```cpp
ecrt_slave_config_sdo16(sc, 0x1C32, 0x01, 0x0002);  // DC SYNC0
```

Only do this if the object exists and is writable, otherwise the slave won't reach SAFE-OP.

---

## 6. Master-Side Configuration (IgH API)

### 6.1 Main API calls

| Function | When | Purpose |
|---|---|---|
| `ecrt_slave_config_dc(sc, assign_activate, sync0_cycle, sync0_shift, sync1_cycle, sync1_shift)` | Before activate | Enables DC SYNC for one slave |
| `ecrt_master_select_reference_clock(master, sc)` | Before activate | Chooses the reference clock slave (optional) |
| `ecrt_master_application_time(master, ns)` | Before activate and every cycle | Tells the master the PC's current time |
| `ecrt_master_sync_reference_clock(master)` | Every cycle (or every N cycles) | Syncs the reference slave to the PC time |
| `ecrt_master_sync_slave_clocks(master)` | Every cycle | Syncs all other slaves to the reference clock |
| `ecrt_master_reference_clock_time(master, &t)` | Optional | Reads the reference clock time (for "master follows slave" schemes) |

### 6.2 `ecrt_slave_config_dc` parameters

```cpp
ecrt_slave_config_dc(
    sc,         // slave config handle
    0x0300,     // AssignActivate from the ESI (SYNC0)
    1000000,    // SYNC0 cycle time: 1 ms in ns
    500000,     // SYNC0 shift: 500 us (see section 10)
    0,          // SYNC1 cycle time: 0 = SYNC1 unused
    0           // SYNC1 shift
);
```

### 6.3 Startup order

```
ecrt_request_master()
ecrt_master_create_domain()
ecrt_master_slave_config()          ← for each slave
ecrt_slave_config_pdos()            ← for each slave
ecrt_domain_reg_pdo_entry_list()
ecrt_slave_config_dc()              ← for each slave that should use SYNC0
ecrt_master_select_reference_clock()  (optional)
ecrt_master_application_time()      ← initial time
ecrt_master_activate()
ecrt_domain_data()
→ start the cyclic loop IMMEDIATELY
```

The master finishes the DC setup (delay measurement, offsets, SYNC start time) **inside the first cycles after activation**. Start the cyclic loop right away and keep it running at a steady rate, otherwise the slaves will not reach OP.

---

## 7. Choosing the Reference Clock

### 7.1 Default behaviour

IgH automatically uses the **first DC-capable slave** on the bus as the reference clock. With your RFID slave at position 0, it is already the reference. No code is needed.

### 7.2 Selecting a different slave

Call this **before** `ecrt_master_activate()`:

```cpp
// Example: make the slave at position 2 the reference clock
ec_slave_config_t *sc_ref = ecrt_master_slave_config(master, 0, 2,
                                                     REF_VENDOR_ID, REF_PRODUCT_CODE);
if (!sc_ref) {
    fprintf(stderr, "Reference slave config failed\n");
    return -1;
}
if (ecrt_master_select_reference_clock(master, sc_ref)) {
    fprintf(stderr, "Could not select reference clock\n");
}
```

### 7.3 Rules for the reference clock

1. **It must be DC-capable.** Check with `ethercat slaves -v -p<pos>`; look for "Distributed clocks: yes".
2. **It should be the first DC-capable slave in the line.** The sync datagram reads the time *at* the reference and writes it to the slaves *after* it. Slaves positioned *before* the reference receive the datagram before it has picked up the fresh time, so they get stale values and sync badly.
3. **Prefer a slave with a 64-bit DC clock.** 32-bit DC clocks wrap every ~4.3 s and are less convenient.
4. **Only change it for a good reason**, for example if slave 0 does not support DC.

### 7.4 Clock sync vs. SYNC0 output

These are two separate things:

| | Who gets it | How |
|---|---|---|
| **Clock synchronisation** | *All* DC-capable slaves | Automatic, once DC is used and `ecrt_master_sync_slave_clocks()` runs |
| **SYNC0 pulse output** | Only slaves you configure | `ecrt_slave_config_dc()` per slave |

---

## 8. Configuring Multiple Slaves (5-Slave Example)

Each slave that should run in DC mode needs its own config and its own `ecrt_slave_config_dc()` call. Use each slave's own vendor ID, product code and AssignActivate value.

```cpp
#define CYCLE_NS       1000000U
#define SYNC0_SHIFT_NS (CYCLE_NS / 2)

struct SlaveDef {
    uint16_t position;
    uint32_t vendor_id;
    uint32_t product_code;
    uint16_t assign_activate;   // 0 = do not enable SYNC0 on this slave
    ec_sync_info_t *syncs;      // PDO configuration for this slave
};

// Adjust IDs/values for your real slaves
static SlaveDef slaves[] = {
    {0, 0x0000007b, 0x00009252, 0x0300, rfid_syncs},    // RFID slave, reference clock
    {1, 0x0000007b, 0x00009252, 0x0300, slave1_syncs},
    {2, 0x0000007b, 0x00009252, 0x0300, slave2_syncs},
    {3, 0x0000007b, 0x00009252, 0x0300, slave3_syncs},
    {4, 0x0000007b, 0x00009252, 0x0000, slave4_syncs},  // e.g. no DC: runs SM-synchronous
};
static const int NUM_SLAVES = sizeof(slaves) / sizeof(slaves[0]);
static ec_slave_config_t *sc[NUM_SLAVES];

int configure_slaves(ec_master_t *master)
{
    for (int i = 0; i < NUM_SLAVES; i++) {
        sc[i] = ecrt_master_slave_config(master, 0, slaves[i].position,
                                         slaves[i].vendor_id, slaves[i].product_code);
        if (!sc[i]) {
            fprintf(stderr, "Slave %d: config failed\n", i);
            return -1;
        }
        if (ecrt_slave_config_pdos(sc[i], EC_END, slaves[i].syncs)) {
            fprintf(stderr, "Slave %d: PDO config failed\n", i);
            return -1;
        }
        if (slaves[i].assign_activate) {
            ecrt_slave_config_dc(sc[i], slaves[i].assign_activate,
                                 CYCLE_NS, SYNC0_SHIFT_NS, 0, 0);
        }
    }

    // Reference clock: slave 0 (also the IgH default if it is DC-capable)
    ecrt_master_select_reference_clock(master, sc[0]);
    return 0;
}
```

All DC slaves should use the **same cycle time**. Using the same shift is normal too, so they all fire SYNC0 at the same instant.

---

## 9. The Cyclic Loop

```cpp
struct timespec wakeup, now;
unsigned int sync_ref_counter = 0;

clock_gettime(CLOCK_MONOTONIC, &wakeup);

while (run) {
    // 1. Wait for the next cycle start (absolute time: no drift)
    wakeup = timespec_add_ns(wakeup, CYCLE_NS);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup, NULL);

    // 2. Receive inputs
    ecrt_master_receive(master);
    ecrt_domain_process(domain1);

    // 3. Application: read inputs, write outputs
    process_data();

    // 4. Queue outputs
    ecrt_domain_queue(domain1);

    // 5. DC: give the master the current time, then sync clocks
    clock_gettime(CLOCK_MONOTONIC, &now);
    ecrt_master_application_time(master, TIMESPEC2NS(now));

    if (sync_ref_counter) {
        sync_ref_counter--;
    } else {
        sync_ref_counter = 1;   // sync reference clock every 2nd cycle
        ecrt_master_sync_reference_clock(master);
    }
    ecrt_master_sync_slave_clocks(master);

    // 6. Send
    ecrt_master_send(master);
}
```

### Rules for the loop

| Rule | Reason |
|---|---|
| Use `clock_nanosleep(..., TIMER_ABSTIME, ...)`, not `usleep()` | `usleep` adds up error each cycle; DC needs a stable period |
| Use the **same clock** (`CLOCK_MONOTONIC`) everywhere | Mixing clocks causes jumps in application time |
| Call `ecrt_master_application_time()` **every cycle** | The master does not count time itself |
| Call `ecrt_master_sync_slave_clocks()` **every cycle** | Keeps drift compensation running |
| No `printf`/`cout` every cycle | Console output blocks and causes jitter. Print every ~1000 cycles at most |
| No memory allocation, file I/O or blocking calls inside the loop | They cause page faults and latency spikes |

---

## 10. Choosing the SYNC0 Shift Time

The shift time decides **where within the 1 ms cycle** the SYNC0 pulse fires.

```
time ─────────────────────────────────────────────────────────────►

PC:    │send frame│                                 │send frame│
       ▼                                            ▼
Bus:   ├──frame travels (~10–100 us)──┤             ├──── ...
                                      ▲
Slave:                     outputs arrive in SM2
                                               ▲
SYNC0:                                    pulse (shift)
       │◄────────────── shift ───────────────►│
       │◄──────────────────── cycle = 1 ms ─────────────────►│
```

SYNC0 should fire:

- **after** the frame has delivered the new outputs to every slave, and
- **early enough** that the slave finishes processing and updates its inputs before the next frame reads them.

| Shift | Effect |
|---|---|
| Too small | SYNC0 fires before the new outputs arrive, so the slave uses last cycle's data and "SM event missed" counters increase |
| Too large | Inputs are not ready when the next frame arrives, so the master reads stale inputs |
| **~50 % of the cycle (500 µs)** | Safe starting point with lots of margin on both sides |

Tune it once everything works. Measure the frame send jitter on the PC and the processing time on the PIC32, then place SYNC0 accordingly. Many systems end up between 20 % and 50 % of the cycle.

---

## 11. Verifying DC Is Working

Run these **while your master application is running** and the slaves are in OP. Replace `-p0` with any slave position.

### 11.1 Is the slave DC-capable?

```bash
ethercat slaves -v -p0
```

Look for `Distributed clocks: yes` and the DC system time width (32 or 64 bit).

### 11.2 Is the slave in OP?

```bash
ethercat slaves
```

Expected: `0  0:0  OP  +  <slave name>`

### 11.3 Is SYNC0 activated?

```bash
ethercat reg_read -p0 -t uint8 0x0981
```

Expected: **`0x03`** (cyclic operation + SYNC0). `0x00` means DC is not active.

### 11.4 Is the SYNC0 cycle time correct?

```bash
ethercat reg_read -p0 -t uint32 0x09A0
```

Expected: **`1000000`** (1 ms).

### 11.5 Is the system time running?

```bash
ethercat reg_read -p0 -t uint64 0x0910
```

Run it several times. The value must keep increasing.

### 11.6 What is the propagation delay?

```bash
ethercat reg_read -p0 -t uint32 0x0928
```

0 or very small for the reference clock; increases for slaves further down the line (typically tens to hundreds of ns per slave).

### 11.7 Is the clock synchronised? (most important)

```bash
ethercat reg_read -p0 -t sm32 0x092C
```

Expected: a **small, stable** value, ideally below ~1000 ns, and not drifting. Large or constantly growing values mean sync is not working.

Watch all 5 slaves continuously:

```bash
watch -n 0.5 'for p in 0 1 2 3 4; do echo -n "slave $p: "; ethercat reg_read -p$p -t sm32 0x092C; done'
```

### 11.8 When is the next SYNC0 pulse?

```bash
ethercat reg_read -p0 -t uint64 0x0990
```

Read it a few times together with `0x0910`. It should always be a little ahead of System Time and move forward in 1 ms steps.

### 11.9 Firmware counters

```bash
ethercat upload -p0 0x1C32 0x0B   # SM event missed counter
ethercat upload -p0 0x1C32 0x0C   # cycle time too small counter
ethercat upload -p0 0x1C32 0x20   # sync error
```

The counters should not keep increasing. Sync error should be FALSE (0).

### 11.10 Oscilloscope (definitive test)

1. Probe the LAN9252 **SYNC0 pin** on slave 0. You should see a clean pulse every **1.000 ms** (1 kHz).
2. Probe SYNC0 on two slaves at once (e.g. slave 0 and slave 3). The pulses should line up within about 100 ns.
3. Optionally toggle a PIC32 GPIO in the SYNC0 ISR to see the interrupt latency and the application's execution time.

### Verification summary

| Check | Command | Good result |
|---|---|---|
| DC capable | `ethercat slaves -v -p0` | `Distributed clocks: yes` |
| State | `ethercat slaves` | `OP` |
| SYNC0 active | `reg_read -t uint8 0x0981` | `0x03` |
| Cycle time | `reg_read -t uint32 0x09A0` | `1000000` |
| Clock running | `reg_read -t uint64 0x0910` | Increasing |
| Sync error | `reg_read -t sm32 0x092C` | Small and stable |
| Missed events | `upload 0x1C32 0x0B` | Not increasing |
| Hardware | Oscilloscope on SYNC0 | 1 kHz, aligned across slaves |

---

## 12. Troubleshooting

### 12.1 Read the AL status code

If a slave stays in PREOP/SAFEOP or drops out of OP:

```bash
ethercat reg_read -p0 -t uint16 0x0134
```

or check the kernel log:

```bash
dmesg | grep -i ethercat
```

### 12.2 DC-related AL status codes

| Code | Meaning | Typical cause |
|---|---|---|
| `0x001A` | Synchronization error | Sync lost during operation |
| `0x001B` | Sync Manager watchdog | Master stopped sending process data |
| `0x002C` | Fatal sync error | SYNC0 pulses stopped arriving at the µC |
| `0x002D` | No sync error | Slave got no SYNC events after switching to OP |
| `0x0030` | Invalid DC SYNC configuration | Wrong AssignActivate, or firmware without `DC_SUPPORTED` |
| `0x0032` | PLL error | Slave cannot lock to the DC; master cycle too irregular |
| `0x0033` | DC Sync IO error | SYNC0 interrupt not wired/handled on the µC |
| `0x0034` | DC Sync timeout | No SYNC0 interrupt received in time |
| `0x0035` | DC invalid sync cycle time | Cycle time not supported by the slave |
| `0x0036` | DC SYNC0 cycle time | SYNC0 cycle invalid (e.g. below the minimum in `0x1C32:05`) |
| `0x0037` | DC SYNC1 cycle time | SYNC1 cycle invalid |

### 12.3 Common problems

| Symptom | Likely cause | Fix |
|---|---|---|
| Slave stuck in SAFE-OP, code `0x0030` | ESI/EEPROM has no `<Dc>` section or wrong AssignActivate | Fix the ESI, re-flash the EEPROM, use the correct AssignActivate |
| Code `0x0033`/`0x0034` | SYNC0 not reaching the PIC32 | Check wiring and IRQ setup; check the SYNC0 pin on a scope |
| `0x092C` large or growing | `ecrt_master_sync_slave_clocks()` not called every cycle, or unstable cycle time | Call it every cycle; use `clock_nanosleep` with `TIMER_ABSTIME` |
| Slaves reach OP, then drop out | Cycle jitter too high | Use a PREEMPT_RT kernel, SCHED_FIFO, `mlockall`, no printing in the loop |
| Slaves before the reference sync badly | Reference clock not first in the line | Make the first DC-capable slave the reference |
| `0x0981` reads `0x00` | DC never activated | Check that `ecrt_slave_config_dc()` is called before `ecrt_master_activate()` |
| Working counter is wrong | PDO mapping mismatch | Compare `ethercat pdos -p<pos>` with your `ec_pdo_entry_info_t` table |
| Missed SM events increasing | Shift time too small or too large | Adjust `SYNC0_SHIFT_NS` (start at 50 % of the cycle) |
| Works for a while, then "PLL error" | PC time jumps (NTP adjusting `CLOCK_REALTIME`) | Use `CLOCK_MONOTONIC` for application time |

### 12.4 Useful IgH tool commands

```bash
ethercat master              # master status, link, frame statistics
ethercat slaves              # all slaves and their states
ethercat slaves -v -p0       # detailed info for slave 0
ethercat pdos -p0            # PDO mapping of slave 0
ethercat config -v           # what the application configured (incl. DC)
ethercat states -p0 OP       # request a state change manually
ethercat reg_read -p0 -t <type> <address>
ethercat upload -p0 <index> <subindex>
```

---

## 13. Linux Real-Time Tips

DC only works as well as the master's timing. At 1 ms, aim for less than ~50 µs jitter.

| Tip | How |
|---|---|
| Real-time kernel | Use a `PREEMPT_RT` patched kernel |
| Real-time priority | `sched_setscheduler(0, SCHED_FIFO, ...)` with priority ~80 |
| Lock memory | `mlockall(MCL_CURRENT \| MCL_FUTURE)` |
| Pre-fault the stack | `memset` a stack buffer once at startup |
| Isolate a CPU | Kernel boot args `isolcpus=3 nohz_full=3 rcu_nocbs=3`, then pin the process with `taskset -c 3` |
| Dedicated NIC driver | Use an IgH native driver (e.g. `ec_igb`, `ec_e1000e`) instead of `ec_generic` when possible |
| Disable power saving | Disable CPU C-states and frequency scaling in BIOS or with `cpupower` |
| Measure jitter | Record the actual wake-up time vs. the expected `wakeup` each cycle and log the max/min periodically |

---

## 14. Quick Checklist

**Slave side**
- [ ] ESI has a `<Dc>` section with the correct `AssignActivate` (usually `#x300`)
- [ ] EEPROM (SII) re-flashed from the updated ESI
- [ ] SSC built with `DC_SUPPORTED = 1`
- [ ] LAN9252 SYNC0 pin wired to a PIC32 interrupt, ISR calls `Sync0_Isr()`
- [ ] `0x1C32:05` (minimum cycle time) ≤ 1,000,000 ns

**Master side**
- [ ] `ecrt_slave_config_dc()` called for every slave that should use SYNC0
- [ ] Reference clock = first DC-capable slave (default, or `ecrt_master_select_reference_clock()`)
- [ ] `ecrt_master_application_time()` called before `ecrt_master_activate()` and every cycle
- [ ] `ecrt_master_sync_reference_clock()` and `ecrt_master_sync_slave_clocks()` called in the loop
- [ ] Loop timed with `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...)`
- [ ] Real-time priority, `mlockall`, no printing every cycle

**Verification**
- [ ] All slaves in OP
- [ ] `0x0981` = `0x03` and `0x09A0` = `1000000`
- [ ] `0x092C` small and stable on all slaves
- [ ] `0x1C32:0B` not increasing
- [ ] SYNC0 visible on the oscilloscope at 1 kHz and aligned across slaves
