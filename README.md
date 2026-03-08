# SIMREACTOR — PRESSURIZED WATER REACTOR SIMULATION SYSTEM

**Document No.:** SIM-OPS-001
**Revision:** 1.0
**Plant Reference:** Westinghouse 4-Loop PWR, 3411 MWt / 1150 MWe

---

## TABLE OF CONTENTS

1. [System Overview](#1-system-overview)
2. [Technical Specifications](#2-technical-specifications)
3. [System Requirements & Installation](#3-system-requirements--installation)
4. [Standard Operating Procedure](#4-standard-operating-procedure)
   - 4.1 [Startup from Cold Shutdown](#41-startup-from-cold-shutdown)
   - 4.2 [Startup from Hot Zero Power](#42-startup-from-hot-zero-power)
   - 4.3 [Initialization at Full Power Stable](#43-initialization-at-full-power-stable)
5. [Control Interface Reference](#5-control-interface-reference)
6. [Reactor Protection System](#6-reactor-protection-system)
7. [Automated Control Systems](#7-automated-control-systems)
8. [API Reference](#8-api-reference)
9. [Warnings & Limitations](#9-warnings--limitations)

---

## 1. SYSTEM OVERVIEW

SIMREACTOR is a high-fidelity pressurized water reactor (PWR) simulation system
modeled on a Westinghouse 4-loop plant design. The simulation covers the complete
operational envelope from cold shutdown through full-power steady-state operation,
including transient and accident scenarios.

The system is comprised of three components:

| Component       | Description                                                     |
|-----------------|-----------------------------------------------------------------|
| `PwrSimEngine`  | Core physics library. Pure simulation, no I/O.                  |
| `PwrSimWeb`     | ASP.NET Core web server with retro-CRT control room interface.  |
| `PwrSimDemo`    | Console-based scenario runner (8 pre-scripted scenarios).       |

Physics models are executed in the following order each simulation tick:

> Control Rods → Boron → Reactivity Budget → Point Kinetics (6 delayed neutron
> groups) → Xe/Sm Poisons → ANS-5.1 Decay Heat (23 groups) → Fuel Rod Thermal
> (radial conduction) → Primary Coolant → Pressurizer → Steam Generator (LMTD)
> → Turbine/Generator → RCP Model → ECCS → Containment → Electrical → Fuel
> Integrity (DNBR, oxidation) → Burnup

---

## 2. TECHNICAL SPECIFICATIONS

### Reactor Core

| Parameter                    | Value              |
|------------------------------|--------------------|
| Rated Thermal Power          | 3411 MWt           |
| Rated Electrical Output      | 1150 MWe           |
| Reactor Coolant Loops        | 4                  |
| Fuel Type                    | UO₂ (pellet/clad)  |
| Moderator / Coolant          | Light water (H₂O)  |
| Neutron Delayed Groups       | 6                  |
| Decay Heat Groups (ANS-5.1)  | 23                 |

### Primary System

| Parameter                    | Value              |
|------------------------------|--------------------|
| Operating Pressure           | ~15.5 MPa (2250 psi) |
| Core Inlet Temperature       | ~291.7 °C          |
| Core Outlet Temperature      | ~324.7 °C          |
| Core ΔT (full power)         | ~33 °C             |
| Tavg (full power)            | ~308.2 °C          |

### Simulation Engine

| Parameter                    | Value              |
|------------------------------|--------------------|
| Maximum Physics Sub-step     | 0.05 s             |
| Simulation Speed Range       | 0.1× – 100×        |
| State Representation         | Binary-serializable value-type struct |
| External Dependencies        | .NET 8 SDK, ASP.NET Core only |

---

## 3. SYSTEM REQUIREMENTS & INSTALLATION

**Prerequisites:** .NET 8.0 SDK

**Build:**

```bash
dotnet build src/
```

**Launch web interface** (recommended for interactive operation):

```bash
cd src/PwrSimWeb && dotnet run
```

Then open `http://localhost:5000` in a browser.

**Launch console scenario runner:**

```bash
cd src/PwrSimDemo && dotnet run
```

The console runner executes 8 pre-scripted scenarios in sequence: cold startup,
load changes, xenon transients, LOCA, LOOP, RCP coast-down, and snapshot/restore.
No user input is required.

---

## 4. STANDARD OPERATING PROCEDURE

> **NOTE:** All three initialization states are available via the web interface
> under the **INIT** panel, or via the REST API. After any initialization, the
> simulation clock is paused. Enable auto-advance via the **RUN** control or
> `POST /api/autotick`.

---

### 4.1 Startup from Cold Shutdown

**Initial Conditions:** RCPs off, core cold (~20 °C), no RCP flow, control rods
fully inserted, boron at cold shutdown concentration, turbine offline.

**Step 1 — Initialize State**

Via web UI: Select **INIT → COLD SHUTDOWN**

Via API:
```
POST /api/init/cold
```

**Step 2 — Start Reactor Coolant Pumps**

Start all four RCPs in sequence. Each pump is addressed by index (0–3).

```
POST /api/rcp    { "pump": 0, "start": true }
POST /api/rcp    { "pump": 1, "start": true }
POST /api/rcp    { "pump": 2, "start": true }
POST /api/rcp    { "pump": 3, "start": true }
```

Verify primary flow fraction reaches ~1.0 in `/api/state` → `primary.flowFraction`
before proceeding.

**Step 3 — Establish Pressurizer Pressure**

Enable pressurizer heater auto-control and allow pressure to rise to the operating
setpoint (~15.5 MPa). This may take several simulated minutes.

```
POST /api/pressurizer/heaterauto   { "enabled": true }
POST /api/pressurizer/sprayauto    { "enabled": true }
```

Monitor `pressurizer.pressureMPa`. Proceed when pressure is stable at setpoint.

**Step 4 — Withdraw Control Rods / Achieve Criticality**

Enable auto-rod control. The Tavg program will manage Bank D to bring the reactor
to criticality and control power ascension.

```
POST /api/rod/auto    { "enabled": true }
```

Alternatively, withdraw rods manually by bank (Banks 0–3, fully withdrawn = 228
steps):

```
POST /api/rod/withdraw    { "bank": 0, "steps": 228 }
POST /api/rod/withdraw    { "bank": 1, "steps": 228 }
POST /api/rod/withdraw    { "bank": 2, "steps": 228 }
```

Bank D (bank index 3) is the controlling bank. Withdraw gradually and monitor
startup rate (`neutronics.startupRateDPM`). The RPS will trip on high startup
rate (>5 DPM) if flux rises too rapidly.

**Step 5 — Monitor Approach to Criticality**

Monitor `neutronics.thermalPowerFrac`. The reactor will achieve criticality and
power will begin rising. Auto-rod will stabilize Tavg at the programmed setpoint
for the current power level.

**Step 6 — Reduce Boron for Power Ascension**

As power rises, reduce boron concentration to maintain rod position in the
acceptable control band. Boron dilution is commanded by target concentration:

```
POST /api/boron    { "ppm": 900 }
```

Adjust in increments. Monitor `boron.ppm` for actual vs. target concentration.

**Step 7 — Bring Turbine Online**

Once thermal power is above ~15% and plant conditions are stable:

```
POST /api/turbine/online    { "enabled": true }
POST /api/turbine/governorauto    { "enabled": true }
POST /api/sg/fwauto    { "enabled": true }
```

**Step 8 — Raise to Full Power**

Set turbine load target and allow the governor and auto-rod to raise power:

```
POST /api/turbine/load    { "mwe": 1150 }
```

Enable auto-advance at desired simulation speed:

```
POST /api/autotick    { "enabled": true, "speed": 10 }
```

Monitor `electrical.generatorMW` and `neutronics.thermalPowerFrac` until full
power is established. Full power stable conditions: ~1150 MWe, Tavg ~308.2 °C,
pressure ~15.5 MPa.

---

### 4.2 Startup from Hot Zero Power

**Initial Conditions:** All 4 RCPs running at full speed, primary at operating
temperature and pressure (~15.5 MPa, Tavg ~291.7 °C), control rods fully
inserted, turbine offline. Plant is at Hot Zero Power (HZP) — subcritical.

**Step 1 — Initialize State**

Via web UI: Select **INIT → HOT ZERO POWER**

Via API:
```
POST /api/init/hzp
```

**Step 2 — Verify Initial Conditions**

Confirm via `/api/state`:
- `primary.flowFraction` ≈ 1.0 (all RCPs running)
- `pressurizer.pressureMPa` ≈ 15.5
- `primary.tavg` ≈ 291.7 °C
- `scram` = `Normal` (no active trips)
- `turbine.online` = false

**Step 3 — Enable Pressurizer Auto-Control**

```
POST /api/pressurizer/heaterauto    { "enabled": true }
POST /api/pressurizer/sprayauto     { "enabled": true }
```

**Step 4 — Enable Auto-Rod and Withdraw to Criticality**

```
POST /api/rod/auto    { "enabled": true }
```

With auto-rod enabled, withdraw Bank D to initiate a controlled power ascension:

```
POST /api/rod/withdraw    { "bank": 3, "steps": 50 }
```

Monitor `neutronics.startupRateDPM`. Keep startup rate below 5 DPM. Withdraw
in increments, waiting for power to stabilize between steps if performing
manually.

**Step 5 — Bring Turbine Online**

At ≥15% thermal power with Tavg and pressure stable:

```
POST /api/turbine/online          { "enabled": true }
POST /api/turbine/governorauto    { "enabled": true }
POST /api/sg/fwauto               { "enabled": true }
```

**Step 6 — Raise to Full Power**

```
POST /api/turbine/load    { "mwe": 1150 }
POST /api/autotick        { "enabled": true, "speed": 10 }
```

The turbine governor and auto-rod will raise load to target. Monitor for
xenon buildup during power ascension — xenon concentration will initially
rise before equilibrating at the new power level. Auto-rod will compensate;
do not withdraw rods manually unless auto-rod is disabled.

Full power is achieved when `electrical.generatorMW` ≈ 1150 MWe and
`neutronics.thermalPowerFrac` ≈ 1.0.

---

### 4.3 Initialization at Full Power Stable

**Initial Conditions:** Reactor at 100% power, all systems at steady-state
equilibrium. Xenon at equilibrium. All auto-controls active.

This state is the default initialization for the web server and is the
recommended starting point for transient and accident scenario studies.

**Step 1 — Initialize State**

Via web UI: Select **INIT → FULL POWER**

Via API:
```
POST /api/init/full
```

**Step 2 — Verify Full Power Conditions**

Confirm via `/api/state` or `/api/diagnostics`:

| Parameter                        | Expected Value     |
|----------------------------------|--------------------|
| `neutronics.thermalPowerFrac`    | ~1.0               |
| `electrical.generatorMW`         | ~1150 MWe          |
| `primary.tavg`                   | ~308.2 °C          |
| `pressurizer.pressureMPa`        | ~15.5 MPa          |
| `primary.flowFraction`           | ~1.0               |
| `turbine.online`                 | true               |
| `scram`                          | Normal             |
| `trips`                          | None               |

**Step 3 — Enable Auto-Advance**

```
POST /api/autotick    { "enabled": true, "speed": 1 }
```

Simulation speed may be set from 0.1× to 100×. For transient studies, 1×
or slower is recommended. For steady-state monitoring or xenon transient
observation, 10× or higher is appropriate.

**Step 4 — Begin Operations**

The plant is ready for operator-directed activities: load following, boron
management, xenon transient observation, or accident scenario initiation
(LOCA, LOOP, turbine trip, etc.).

To take a snapshot of the current state for later restoration:

```
GET /api/snapshot   → downloads reactor_state.bin
POST /api/restore   → upload reactor_state.bin to restore
```

---

## 5. CONTROL INTERFACE REFERENCE

The web interface at `http://localhost:5000` provides a retro-CRT control room
panel with real-time display of all plant parameters. The panel polls
`GET /api/state` every 250 ms.

Key display sections:

| Panel             | Parameters Displayed                                        |
|-------------------|-------------------------------------------------------------|
| NEUTRONICS        | Power %, thermal power MW, decay heat, startup rate DPM     |
| REACTIVITY        | Total ρ (pcm), rod/Doppler/moderator/boron/xenon/samarium   |
| FUEL              | Centerline temp, DNBR, cladding oxide, burnup               |
| PRIMARY COOLANT   | Pressure, flow, Tin, Tout, Tavg, void fraction              |
| PRESSURIZER       | Pressure, level, heater kW, spray status                    |
| RODS              | Bank positions (steps), auto mode, Tavg setpoint            |
| STEAM GENERATOR   | Secondary pressure, steam flow, feedwater flow, SG level    |
| TURBINE           | Online status, load MW, throttle, governor mode             |
| ELECTRICAL        | Generator MW, net MW, offsite power, diesel status          |
| ECCS              | SI signal, accumulator status, HPI/LPI flow                 |
| CONTAINMENT       | Pressure kPa, temperature, H₂ fraction                     |
| CORE DAMAGE       | Damage state, Zr oxidation, fission product release         |

---

## 6. REACTOR PROTECTION SYSTEM

The Reactor Protection System (RPS) continuously evaluates 12+ trip conditions.
Any trip automatically executes a reactor scram (all rods fully inserted).

| Trip Signal              | Setpoint / Condition                                      |
|--------------------------|-----------------------------------------------------------|
| High Neutron Flux        | Total power > overpower trip setpoint                     |
| High Flux Rate           | Rate > design limit AND power > 5%                        |
| High Pressure            | Primary pressure > high pressure trip setpoint            |
| Low Pressure             | Primary pressure < low pressure trip AND power > 2%       |
| High Temperature         | Tavg > design trip temperature                            |
| Low Flow                 | Primary flow < low flow trip fraction AND power > 5%      |
| High Containment Pressure| Containment pressure > design trip setpoint               |
| Low SG Level             | SG level fraction < design limit AND power > 2%           |
| High SG Level            | SG level fraction > design limit AND power > 2%           |
| High Startup Rate        | Startup rate > 5 DPM AND 0.0001% < power < 5%            |
| Low DNBR                 | DNBR < design limit AND flow > 10% AND power > 5%         |
| Turbine Trip             | Turbine trip flag AND power > 15%                         |
| Loss of Offsite Power    | Offsite power lost AND power > 2%                         |

**Manual trip:**
```
POST /api/trip
```

**Reset trip (after resolving cause):**
```
POST /api/trip/reset
```

**Disable/enable RPS** (for scenario studies only — not a normal operating mode):
```
POST /api/rps    { "enabled": false }
```

**Emergency boration:**
```
POST /api/boron/emergency
```

**Manual safety injection:**
```
POST /api/eccs/si
```

The SI signal latches once activated and must be manually reset by the operator
after verifying plant conditions are safe.

---

## 7. AUTOMATED CONTROL SYSTEMS

The following automatic control loops are available. All may be individually
enabled or disabled.

### Pressurizer Pressure Control

Maintains primary pressure at the configured setpoint via heater and spray
proportional control. Deadband and gain are adjustable.

```
POST /api/pressurizer/heaterauto      { "enabled": true }
POST /api/pressurizer/sprayauto       { "enabled": true }
POST /api/pressurizer/setpoint        { "value": 15.51 }   ← MPa
POST /api/pressurizer/autogain        { "value": 10.0 }
POST /api/pressurizer/autodeadband    { "value": 0.05 }    ← MPa
```

### Automatic Rod Control (Tavg Program)

Bank D position is driven by proportional control on Tavg error relative to
the Tavg program (linear ramp from 291.7 °C at 0% power to 308.2 °C at 100%
power).

```
POST /api/rod/auto             { "enabled": true }
POST /api/rod/autogain         { "value": 3.0 }    ← steps/min per °C
POST /api/rod/autodeadband     { "value": 0.5 }    ← °C
```

### Turbine Governor

Throttle position tracks the turbine load setpoint based on available steam flow.

```
POST /api/turbine/governorauto    { "enabled": true }
POST /api/turbine/load            { "mwe": 1150 }
```

### Feedwater Control (3-Element)

Steam generator level is maintained by feedwater flow, using a 3-element
controller (level error + steam/feed flow mismatch).

```
POST /api/sg/fwauto              { "enabled": true }
POST /api/sg/fwlevelsetpoint     { "value": 0.5 }
POST /api/sg/fwlevelgain         { "value": 500 }
POST /api/sg/fwflowgain          { "value": 1.0 }
```

---

## 8. API REFERENCE

All endpoints accept and return JSON. Numeric values that are NaN or Infinity
are serialized as strings — clients must handle this.

### Initialization

| Method | Endpoint          | Description                          |
|--------|-------------------|--------------------------------------|
| POST   | `/api/init/cold`  | Initialize to Cold Shutdown state    |
| POST   | `/api/init/hzp`   | Initialize to Hot Zero Power state   |
| POST   | `/api/init/full`  | Initialize to Full Power Stable state|

### Simulation Control

| Method | Endpoint        | Body                                   | Description               |
|--------|-----------------|----------------------------------------|---------------------------|
| POST   | `/api/tick`     | `{ "dt": 1.0 }`                        | Advance by dt seconds     |
| POST   | `/api/autotick` | `{ "enabled": true, "speed": 1.0 }`   | Auto-advance control      |

### State & Diagnostics

| Method | Endpoint           | Description                              |
|--------|--------------------|------------------------------------------|
| GET    | `/api/state`       | Full plant state (all parameters)        |
| GET    | `/api/diagnostics` | Summary diagnostics                      |
| GET    | `/api/reactivity`  | Reactivity breakdown in pcm              |
| GET    | `/api/events`      | Last 200 event log entries               |
| GET    | `/api/snapshot`    | Download binary state file               |
| POST   | `/api/restore`     | Upload binary state file to restore      |

### Control Rods

| Method | Endpoint             | Body                              | Description                |
|--------|----------------------|-----------------------------------|----------------------------|
| POST   | `/api/rod/bank`      | `{ "bank": 3, "target": 200 }`   | Set bank target position   |
| POST   | `/api/rod/speed`     | `{ "speed": 48 }`                | Set rod speed (steps/min)  |
| POST   | `/api/rod/withdraw`  | `{ "bank": 3, "steps": 10 }`    | Withdraw N steps           |
| POST   | `/api/rod/insert`    | `{ "bank": 3, "steps": 10 }`    | Insert N steps             |
| POST   | `/api/rod/auto`      | `{ "enabled": true }`            | Enable auto-rod control    |

### Primary System

| Method | Endpoint        | Body                       | Description               |
|--------|-----------------|----------------------------|---------------------------|
| POST   | `/api/rcp`      | `{ "pump": 0, "start": true }` | Start/stop RCP         |
| POST   | `/api/boron`    | `{ "ppm": 900 }`           | Set boron target (ppm)    |
| POST   | `/api/boron/emergency` | (none)              | Emergency boration        |

### Protection & Safety

| Method | Endpoint          | Description                              |
|--------|-------------------|------------------------------------------|
| POST   | `/api/trip`       | Manual reactor trip (scram)              |
| POST   | `/api/trip/reset` | Reset scram (after resolving cause)      |
| POST   | `/api/rps`        | Enable/disable Reactor Protection System |
| POST   | `/api/eccs/si`    | Initiate Safety Injection                |
| POST   | `/api/eccs/accumulators` | Isolate/restore accumulators     |

### Secondary System

| Method | Endpoint                 | Body                        | Description                  |
|--------|--------------------------|-----------------------------|------------------------------|
| POST   | `/api/turbine/online`    | `{ "enabled": true }`       | Bring turbine online/offline |
| POST   | `/api/turbine/load`      | `{ "mwe": 1150 }`           | Set turbine load target      |
| POST   | `/api/sg/feedtemp`       | `{ "tempC": 227 }`          | Set feedwater temperature    |
| POST   | `/api/sg/fouling`        | `{ "factor": 1.0 }`         | Set SG tube fouling factor   |

### Electrical

| Method | Endpoint               | Description                |
|--------|------------------------|----------------------------|
| POST   | `/api/power/offsite`   | Enable/disable offsite power|
| POST   | `/api/power/diesel`    | Start/stop diesel generator |

---

## 9. WARNINGS & LIMITATIONS

**THIS IS A SIMULATION SYSTEM.** It is provided for educational and engineering
study purposes only. It does not represent the actual design, procedures, or
safety systems of any licensed nuclear power facility.

- Reactivity units are Δk/k internally; displayed as pcm (×10⁵) in the API.
- All temperatures are in degrees Celsius (°C) unless the field name specifies
  otherwise.
- All pressures are in MPa unless the field name specifies otherwise.
- Fractions are in the range 0.0–1.0 (e.g., `thermalPowerFrac`, `flowFraction`).
- The simulation state struct is binary-serializable; snapshot files are
  platform-specific and may not be portable across architectures.
- Simulation speed above 10× may reduce accuracy of transient phenomena.
- The RPS and ECCS are modeled but simplified relative to actual plant logic.
  Do not use this simulation for safety analysis or regulatory purposes.

---

*SIMREACTOR is an open-source project. Zero external dependencies beyond the
.NET 8 SDK and ASP.NET Core.*
