# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Run

```bash
# Build everything
dotnet build src/

# Run the web UI (then open http://localhost:5000)
cd src/PwrSimWeb && dotnet run

# Run the console demo (8 scenarios)
cd src/PwrSimDemo && dotnet run
```

All projects target .NET 8.0 with `AllowUnsafeBlocks` enabled (needed for binary state serialization).

## Architecture

PWR (Pressurized Water Reactor) simulator modeled on a Westinghouse 4-loop design (3411 MWt / 1150 MWe). The reactor engine aims to be comprehensive, precise, and physically accurate — models should reflect real PWR behavior, use correct engineering units and correlations, and cover the full operational envelope from cold shutdown through full power and transient/accident scenarios.

### PwrSimEngine — Core Physics + Controller

Two key files in `src/PwrSimEngine/`:

**`PwrSimEngine.cs`** (~2200 lines) — Pure physics simulation. Key types:

- **`ReactorState`** — monolithic value-type struct holding the entire simulation state. Binary-serializable via `Marshal.StructureToPtr` for snapshot/restore. Uses fixed-size array structs (`Array4`, `Array6`, `Array23`) to avoid heap allocation. Includes controller-written fields (`EccsSiSignal`, `PreviousTotalPowerFraction`) read by physics.
- **`ReactorDesign`** — sealed class with immutable plant design parameters (coefficients, setpoints, geometry). Units are embedded in field names (`MWt`, `MPa`, `C`, `ppm`, `DPM`).
- **`ReactorEngine`** — sealed simulation driver. `Tick(dt)` advances physics with automatic sub-stepping (max 0.05s). Reads control signals from state (throttle position, heater power, spray flow, SI signal) but does NOT set them. Exposes operator commands (`CommandRcp`, `CommandRodBank`, `ManualTrip`, etc.) and diagnostics.

Physics models in tick order: control rods → boron → reactivity budget → point-kinetics (6 delayed groups) → Xe/Sm poisons → ANS-5.1 decay heat (23 groups) → fuel rod thermal (radial conduction) → primary coolant → pressurizer → steam generator (LMTD) → turbine/generator → RCP model → ECCS → containment → electrical → fuel integrity (DNBR, oxidation) → burnup.

**`ReactorController.cs`** (~280 lines) — Automation/control layer wrapping the engine. `Update(dt)` sub-steps at 0.05s max, running for each sub-step: pre-tick controls → `Engine.Tick(subDt)` → post-tick protection/auto-rod. Contains:
- Pressurizer auto-control (heaters + spray)
- Turbine governor (throttle tracks load setpoint)
- 3-element feedwater control (level + flow + steam)
- ECCS SI signal determination
- Reactor Protection System (12+ trip evaluations → auto-scram)
- Automatic rod control (Tavg program, Bank D proportional)
- Station blackout alarm

**Design principle**: The engine handles pure physics; the controller sets control signals before each tick and evaluates protection logic after. The engine can be used standalone for manual-only scenarios; use the controller for full automation. All operator commands go directly to the engine; all ticking goes through the controller.

### PwrSimWeb — REST API + UI

`Program.cs`: ASP.NET Core minimal-API server. Uses `ReactorController.Update(dt)` for time-stepping (not raw `Engine.Tick`). Background ticker thread auto-advances the simulation at configurable speed (0.1x–100x). All engine access is synchronized via `lock(_lock)`. JSON uses `camelCase` naming and `AllowNamedFloatingPointLiterals` (NaN/Infinity serialize as strings — the frontend must handle this).

`wwwroot/index.html`: Vanilla JS retro-CRT control room UI. Polls `GET /api/state` every 250ms. No frameworks or build step.

### PwrSimDemo — Console Scenarios

Standalone demo exercising cold startup, load changes, xenon transients, LOCA, LOOP, RCP coast-down, and snapshot/restore.

## Key Conventions

- Units always in property names: `TavgC`, `PrimaryPressureMPa`, `BoronConcentrationPPM`, `LinearHeatRate` (W/m)
- Reactivity stored as Δk/k internally, displayed as pcm (×1e5) in the API
- `Rho` prefix = reactivity component; `Design` prefix = immutable parameter
- Fractions are 0.0–1.0 (e.g., `ThermalPowerFraction`, `PrimaryFlowFraction`)
- Zero external dependencies beyond the .NET SDK and ASP.NET Core
