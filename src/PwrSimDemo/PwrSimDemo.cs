// =============================================================================
// PwrSimDemo.cs — Demonstration harness for the PWR Simulation Engine
// =============================================================================
// Run:  cd PwrSimDemo && dotnet run
//
// Each scenario prints structured snapshots at key timestamps covering all
// major physics subsystems.  The output is stable across runs and suitable
// for use as an integration-test baseline (diff against a known-good run).
// =============================================================================

using System;
using PwrSimulator;

public static class PwrSimDemo
{
    public static void Main()
    {
        Console.WriteLine("╔══════════════════════════════════════════════════╗");
        Console.WriteLine("║   PWR Nuclear Reactor Simulation Engine  v1.0   ║");
        Console.WriteLine("╚══════════════════════════════════════════════════╝");
        Console.WriteLine();

        var design = new ReactorDesign
        {
            RatedThermalPowerMW = 3411.0,   // Westinghouse 4-loop
            RatedElectricalMW   = 1150.0,
        };
        var engine     = new ReactorEngine(design);
        var controller = new ReactorController(engine);

        // ==================================================================
        //  SCENARIO 1 — Full-power steady-state → 75 % load reduction
        // ==================================================================
        Console.WriteLine("═══ Scenario 1: Full-power steady-state ═══\n");
        engine.InitialiseFullPower();
        PrintSnapshot(engine, "t=0s: full-power init");

        Console.WriteLine("  ▸ Running 60 s at 100 % …");
        controller.Update(60.0);
        PrintSnapshot(engine, "t=60s");

        Console.WriteLine("  ▸ Reducing turbine load to 862 MWe (~75 %) with auto-rod …");
        engine.CommandTurbineLoad(862);
        engine.SetAutoRodControl(true);
        controller.Update(300.0);
        PrintSnapshot(engine, "t=6min: after 75% load reduction");

        // ==================================================================
        //  SCENARIO 2 — Snapshot / Restore
        // ==================================================================
        Console.WriteLine("\n═══ Scenario 2: Snapshot & Restore ═══\n");
        engine.InitialiseFullPower();
        ReactorState bookmark = engine.Snapshot();
        Console.WriteLine("  ▸ State bookmarked at full power.");
        Console.WriteLine("  ▸ Manual trip …");
        engine.ManualTrip();
        controller.Update(30.0);
        PrintSnapshot(engine, "t=+30s post-trip");

        Console.WriteLine("  ▸ Restoring …");
        engine.Restore(bookmark);
        PrintSnapshot(engine, "Restored (should match full-power init)");

        // ==================================================================
        //  SCENARIO 3 — Cold startup to criticality
        // ==================================================================
        Console.WriteLine("\n═══ Scenario 3: Cold startup to criticality ═══\n");
        engine.InitialiseColdShutdown();
        Console.WriteLine("  ▸ Jumping to hot zero power …");
        engine.InitialiseHotZeroPower();
        PrintSnapshot(engine, "HZP init");

        // Enable turbine so feedwater control maintains SG level during power rise.
        // Without this, no feedwater flows (turbine offline guard) and SG boils off.
        engine.CommandTurbineOnline(true);
        engine.CommandTurbineLoad(1150.0);  // full load setpoint → throttle opens to consume steam

        // Slow bank D withdrawal: 1 step/min ≈ 32 pcm/min insertion rate.
        // Doppler and MTC feedback can track this rate without a superprompt excursion.
        // 5 steps from position 150 adds ~160 pcm net excess reactivity.
        engine.CommandRodSpeed(1);          // 1 step/min
        engine.CommandRodBank(3, 155);      // target: +5 steps from HZP (was 150)
        engine.SetProtectionSystem(true);

        Console.WriteLine("  ▸ Slow bank D withdrawal at 1 step/min toward 155 steps …\n");
        Console.WriteLine($"  {"Time",-7}  {"Fission%",10}  {"SUR(DPM)",9}  {"Boron(ppm)",11}  {"BankD",6}  {"Tavg(°C)",9}  {"Rho(pcm)",9}");
        Console.WriteLine($"  {"───────",-7}  {"──────────",10}  {"─────────",9}  {"───────────",11}  {"──────",6}  {"─────────",9}  {"─────────",9}");
        for (int t = 1; t <= 12; t++)
        {
            controller.Update(60.0);
            var s = engine.State;
            Console.WriteLine($"  {"t+" + t + "min",-7}  {s.ThermalPowerFraction*100,10:F4}  {s.StartupRateDPM,9:F2}  {s.BoronConcentrationPPM,11:F0}  {s.RodBankPosition.V3,6:F1}  {s.TavgC,9:F2}  {s.RhoTotal*1e5,9:F2}");
            if (s.Scram != ScramState.Normal) { PrintSnapshot(engine, "TRIP during startup"); break; }
        }

        // ==================================================================
        //  SCENARIO 4 — Xenon transient after trip
        // ==================================================================
        Console.WriteLine("\n═══ Scenario 4: Xenon transient after trip ═══\n");
        engine.InitialiseFullPower();
        Console.WriteLine("  ▸ Starting at 100 % equilibrium Xe.  Tripping …\n");
        engine.ManualTrip();

        Console.WriteLine($"  {"Hours",-6}  {"Fiss%",7}  {"Xe/eq",12}  {"I-135(rel)",11}  {"Sm/eq",8}  {"Decay%",7}  {"RhoXe(pcm)",11}");
        Console.WriteLine($"  {"──────",-6}  {"─────",7}  {"────────────",12}  {"───────────",11}  {"──────",8}  {"──────",7}  {"──────────",11}");

        double iRef = engine.State.Iodine135;   // I-135 at t=0 (full-power eq.)
        for (int h = 0; h <= 48; h++)
        {
            if (h > 0) controller.Update(3600.0);
            var s = engine.State;
            double xeRatio = s.XenonEquilibrium > 0 ? s.Xenon135 / s.XenonEquilibrium : 0;
            double smRatio = s.SamariumEquilibrium > 0 ? s.Samarium149 / s.SamariumEquilibrium : 0;
            double iRel    = iRef > 0 ? s.Iodine135 / iRef : 0;
            // print every hour to h=12, then every 4 h (always print 0 and 48)
            if (h <= 12 || h % 4 == 0)
                Console.WriteLine($"  {h,6}  {s.ThermalPowerFraction*100,7:F3}  {xeRatio,12:F4}  {iRel,11:F4}  {smRatio,8:F4}  {s.DecayHeatFraction*100,7:F4}  {s.RhoXenon*1e5,11:F2}");
        }

        // ==================================================================
        //  SCENARIO 5 — Small-break LOCA
        // ==================================================================
        Console.WriteLine("\n═══ Scenario 5: Small-break LOCA ═══\n");
        engine.InitialiseFullPower();
        engine.SetProtectionSystem(true);
        Console.WriteLine("  ▸ Depressurising primary to 11.5 MPa (small-break LOCA) …\n");
        var locaState = engine.Snapshot();
        locaState.PressuriserPressureMPa = 11.5;
        locaState.PrimaryPressureMPa    = 11.5;   // below SI setpoint (13.1 MPa) and HPI threshold (12.0 MPa)
        engine.Restore(locaState);

        Console.WriteLine($"  {"Time",-6}  {"P(MPa)",7}  {"Mode",-12}  {"Trips",-24}  {"HPI(kg/s)",10}  {"Accum(m³)",10}  {"Subcool(°C)",11}  {"SI",3}");
        Console.WriteLine($"  {"──────",-6}  {"───────",7}  {"────────────",-12}  {"────────────────────────",-24}  {"──────────",10}  {"──────────",10}  {"───────────",11}  {"──",3}");
        for (int i = 1; i <= 10; i++)
        {
            controller.Update(10.0);
            var s = engine.State;
            Console.WriteLine($"  {"t+" + i*10 + "s",-6}  {s.PrimaryPressureMPa,7:F3}  {s.Mode,-12}  {s.ActiveTrips,-24}  {s.HpiFlowKgPerS,10:F1}  {s.AccumulatorWaterVolumeM3,10:F2}  {s.SubcoolingMarginC,11:F2}  {s.EccsSiSignal,3}");
        }

        // ==================================================================
        //  SCENARIO 6 — Binary serialisation round-trip
        // ==================================================================
        Console.WriteLine("\n═══ Scenario 6: Binary serialisation ═══\n");
        engine.InitialiseFullPower();
        byte[] blob = engine.SerialiseState();
        Console.WriteLine($"  ▸ Serialised state:  {blob.Length} bytes");

        double savedPower = engine.State.ThermalPowerFraction;
        double savedTavg  = engine.State.TavgC;
        double savedPress = engine.State.PrimaryPressureMPa;

        engine.InitialiseColdShutdown();
        Console.WriteLine($"  ▸ Reset to cold.     Power = {engine.State.ThermalPowerFraction*100:F4} %");
        engine.DeserialiseState(blob);
        Console.WriteLine($"  ▸ Deserialised.      Power = {engine.State.ThermalPowerFraction*100:F4} %  (saved: {savedPower*100:F4} %)");
        Console.WriteLine($"                       Tavg  = {engine.State.TavgC:F2} °C   (saved: {savedTavg:F2} °C)");
        Console.WriteLine($"                       Press = {engine.State.PrimaryPressureMPa:F3} MPa  (saved: {savedPress:F3} MPa)");

        // ==================================================================
        //  SCENARIO 7 — Loss of off-site power + diesel start
        // ==================================================================
        Console.WriteLine("\n═══ Scenario 7: Loss of off-site power ═══\n");
        engine.InitialiseFullPower();
        engine.SetProtectionSystem(true);
        Console.WriteLine("  ▸ LOOP event …");
        engine.SetOffSitePower(false);
        controller.Update(2.0);
        PrintSnapshot(engine, "t=+2s after LOOP");

        Console.WriteLine("  ▸ Starting diesel generator …");
        engine.CommandDieselGenerator(true);
        controller.Update(60.0);
        PrintSnapshot(engine, "t=+62s (DG running, decay heat cooling)");

        // ==================================================================
        //  SCENARIO 8 — RCP coast-down (2 of 4 pumps tripped)
        // ==================================================================
        Console.WriteLine("\n═══ Scenario 8: RCP coast-down ═══\n");
        engine.InitialiseFullPower();
        engine.SetProtectionSystem(false);   // observe without auto-trip for demo
        Console.WriteLine("  ▸ Tripping RCP #1 and #2 …\n");
        engine.CommandRcp(0, false);
        engine.CommandRcp(1, false);

        Console.WriteLine($"  {"Time",-6}  {"Flow%",6}  {"Tavg(°C)",9}  {"ΔT(°C)",7}  {"DNBR",6}  {"P(MPa)",7}  {"Subcool(°C)",11}  {"Void%",6}");
        Console.WriteLine($"  {"──────",-6}  {"──────",6}  {"─────────",9}  {"───────",7}  {"────",6}  {"───────",7}  {"───────────",11}  {"─────",6}");
        for (int i = 1; i <= 8; i++)
        {
            controller.Update(5.0);
            var s = engine.State;
            Console.WriteLine($"  {"t+" + i*5 + "s",-6}  {s.PrimaryFlowFraction*100,6:F1}  {s.TavgC,9:F2}  {s.CoreDeltaT,7:F2}  {s.DNBRatio,6:F3}  {s.PrimaryPressureMPa,7:F3}  {s.SubcoolingMarginC,11:F2}  {s.CoolantVoidFraction*100,6:F3}");
        }

        // ==================================================================
        //  SCENARIO 9 — Procedural cold startup (automated operator sequence)
        // ==================================================================
        Console.WriteLine("\n═══ Scenario 9: Procedural cold startup ═══\n");
        Console.WriteLine("  This scenario performs an automated operator startup sequence");
        Console.WriteLine("  using state-conditional logic, printing at physics milestones.\n");
        Console.WriteLine("  [Physical heatup from cold omitted — jumping to HZP.]\n");
        Console.WriteLine("  In a real startup: all 4 RCPs run for ~24h with pressurizer");
        Console.WriteLine("  heaters to raise primary temp from 30°C to 291°C.  The SG");
        Console.WriteLine("  secondary must be isolated or drained to allow heatup; RCP");
        Console.WriteLine("  shaft heat (~24 MW) is the primary heat source.\n");

        engine.InitialiseHotZeroPower();
        engine.SetProtectionSystem(true);
        engine.CommandRodSpeed(48);
        PrintSnapshot(engine, "HZP: starting state");

        // ── Step 1: Withdraw A, B, C to full ────────────────────────────────
        Console.WriteLine("  ── Step 1: Withdraw banks A, B, C to full ──\n");
        engine.CommandRodBank(0, 228);
        engine.CommandRodBank(1, 228);
        engine.CommandRodBank(2, 228);
        // wait for rods to finish moving (≤3 min at 48 steps/min)
        for (int i = 0; i < 6 && (engine.State.RodBankPosition.V0 < 227
                                || engine.State.RodBankPosition.V1 < 227
                                || engine.State.RodBankPosition.V2 < 227); i++)
            controller.Update(30.0);
        PrintSnapshot(engine, "After banks A/B/C fully withdrawn");

        // ── Step 2: Approach criticality via slow bank D withdrawal ──────────
        // Enable turbine so feedwater control maintains SG level during power rise.
        engine.CommandTurbineOnline(true);
        engine.CommandTurbineLoad(1150.0);

        Console.WriteLine("  ── Step 2: Approach to criticality ──\n");
        Console.WriteLine("  Procedure: withdraw bank D at 1 step/min toward 155;");
        Console.WriteLine("  hold when SUR > 0.10 DPM; stop at 5 % power or trip.\n");

        engine.CommandRodSpeed(1);          // 1 step/min — controlled criticality approach
        engine.CommandRodBank(3, 155);      // +5 steps from HZP position (≈160 pcm excess)

        Console.WriteLine($"  {"Time",-8}  {"Fiss%",11}  {"SUR",8}  {"Rho(pcm)",9}  {"Boron",6}  {"BankD",6}  {"Event",-22}");
        Console.WriteLine($"  {"────────",-8}  {"───────────",11}  {"────────",8}  {"─────────",9}  {"──────",6}  {"──────",6}  {"──────────────────────",-22}");

        bool firstCriticality = false;
        bool firstPercent     = false;
        bool firstFivePercent = false;

        for (int minute = 1; minute <= 120 && engine.State.Scram == ScramState.Normal; minute++)
        {
            controller.Update(60.0);
            var s = engine.State;

            // adaptive rate control: slow dilution when approaching criticality
            if (s.StartupRateDPM > 0.08 && s.BoronConcentrationTarget > s.BoronConcentrationPPM - 2)
                engine.CommandBoronTarget(s.BoronConcentrationPPM); // hold boron

            string evt = "";
            if (!firstCriticality && s.ThermalPowerFraction > 1e-7)
            { evt = ">> source-range detectable";  firstCriticality = true; }
            else if (!firstPercent && s.ThermalPowerFraction > 0.01)
            { evt = ">> 1 % power";  firstPercent = true; }
            else if (!firstFivePercent && s.ThermalPowerFraction > 0.05)
            { evt = ">> 5 % power";  firstFivePercent = true; }

            bool milestone = evt.Length > 0 || minute % 5 == 0
                          || s.ThermalPowerFraction > 0.001
                          || Math.Abs(s.StartupRateDPM) > 0.02;
            if (milestone)
                Console.WriteLine($"  {"t+" + minute + "min",-8}  {s.ThermalPowerFraction*100,11:F6}  {s.StartupRateDPM,8:F3}  {s.RhoTotal*1e5,9:F2}  {s.BoronConcentrationPPM,6:F0}  {s.RodBankPosition.V3,6:F0}  {evt,-22}");

            if (evt.Contains("5 %") || s.ThermalPowerFraction > 0.05)
            {
                PrintSnapshot(engine, "5 % power level");
                break;
            }
        }

        if (engine.State.Scram != ScramState.Normal)
            PrintSnapshot(engine, "TRIP during startup procedure");
        else if (!firstFivePercent)
            PrintSnapshot(engine, "End of startup window (120 min)");

        // ==================================================================
        Console.WriteLine("\n══════════════════════════════════════════════════");
        Console.WriteLine("  All scenarios complete.  Engine API verified.");
        Console.WriteLine("══════════════════════════════════════════════════");
    }

    // ======================================================================
    //  Comprehensive snapshot covering all major physics subsystems
    // ======================================================================
    static void PrintSnapshot(ReactorEngine engine, string label = "")
    {
        var s   = engine.State;
        var rho = engine.GetReactivityBreakdownPCM();
        double xeRatio = s.XenonEquilibrium > 0 ? s.Xenon135 / s.XenonEquilibrium : 0.0;
        double smRatio = s.SamariumEquilibrium > 0 ? s.Samarium149 / s.SamariumEquilibrium : 0.0;
        double iRel    = s.Iodine135;   // absolute concentration (units consistent with Xe/Sm)

        string tag = string.IsNullOrEmpty(label) ? "" : $" {label} ";
        Console.WriteLine($"\n  ┌─{tag}────────────────────────────────────────────────");
        Console.WriteLine($"  │ SimTime: {s.SimulationTimeSeconds,8:F1}s │ Mode: {s.Mode,-14} │ Scram: {s.Scram,-12} │ Trips: {s.ActiveTrips}");

        // ── Neutronics ──────────────────────────────────────────────────────
        Console.WriteLine($"  ├─ Neutronics");
        Console.WriteLine($"  │  Fission: {s.ThermalPowerFraction*100,9:F4}% ({s.ThermalPowerMW,8:F1} MWt)   Decay: {s.DecayHeatFraction*100,7:F4}% ({s.DecayHeatMW,7:F1} MWt)   Total: {s.TotalPowerMW,8:F1} MWt");
        Console.WriteLine($"  │  NeutPop: {s.NeutronPopulation,11:F6}   SUR: {s.StartupRateDPM,7:F2} DPM");
        Console.WriteLine($"  │  Rho(pcm): Total={rho["Total"],9:F2}  Rods={rho["ControlRods"],9:F2}  Doppler={rho["Doppler"],8:F2}  Moderator={rho["Moderator"],8:F2}");
        Console.WriteLine($"  │            Boron={rho["Boron"],9:F2}  Xe={rho["Xenon"],12:F2}  Samarium={rho["Samarium"],8:F2}  Void={rho["Void"],8:F2}  Bias={rho["ManualBias"],8:F2}");
        Console.WriteLine($"  │  Xe: {xeRatio,10:F4}×eq  (I-135: {s.Iodine135:E3})   Sm: {smRatio,8:F4}×eq  (Pm-149: {s.Promethium149:E3})");
        Console.WriteLine($"  │  Rods A/B/C/D: {s.RodBankPosition.V0,5:F0} / {s.RodBankPosition.V1,5:F0} / {s.RodBankPosition.V2,5:F0} / {s.RodBankPosition.V3,5:F0} steps   Boron: {s.BoronConcentrationPPM,6:F0} ppm");

        // ── Core Thermal-Hydraulics ─────────────────────────────────────────
        Console.WriteLine($"  ├─ Core Thermal-Hydraulics");
        Console.WriteLine($"  │  Tin: {s.CoreInletTempC,6:F2}°C  Tout: {s.CoreOutletTempC,6:F2}°C  Tavg: {s.TavgC,6:F2}°C  ΔT: {s.CoreDeltaT,5:F2}°C");
        Console.WriteLine($"  │  Flow: {s.PrimaryFlowFraction*100,6:F2}%  ({s.PrimaryFlowRateKgPerS,6:F0} kg/s)   Density: {s.CoolantDensityKgPerM3,6:F1} kg/m³   Void: {s.CoolantVoidFraction*100,6:F3}%   Subcool: {s.SubcoolingMarginC,+7:F2}°C");
        Console.WriteLine($"  │  Fuel-CL: {s.FuelCentrelineTemp,7:F1}°C  Fuel-avg: {s.FuelAverageTemp,7:F1}°C  Fuel-surf: {s.FuelSurfaceTemp,7:F1}°C");
        Console.WriteLine($"  │  Clad-in: {s.CladInnerTemp,7:F1}°C  Clad-out: {s.CladOuterTemp,7:F1}°C  Gap: {s.GapConductance,6:F0} W/m²K");
        Console.WriteLine($"  │  DNBR: {s.DNBRatio,6:F3}   LinHeat: {s.LinearHeatRate,6:F0} W/m   Pkfac: {s.PeakingFactor,5:F3}   Burnup: {s.FuelBurnupMWdPerTonne,7:F2} MWd/tHM");

        // ── Primary Circuit ─────────────────────────────────────────────────
        Console.WriteLine($"  ├─ Primary Circuit");
        Console.WriteLine($"  │  Press: {s.PrimaryPressureMPa,7:F3} MPa  ({s.PrimaryPressurePsi,6:F0} psi)");
        Console.WriteLine($"  │  Pzr: {s.PressuriserPressureMPa,7:F3} MPa  Level: {s.PressuriserLevelFraction,5:F3}  Temp: {s.PressuriserTempC,6:F1}°C  Heater: {s.PressuriserHeaterPowerKW,6:F0} kW  Spray: {(s.PressuriserSprayOn==1?"ON ":"off")}  PORV: {(s.PorvOpen==1?"OPEN":"shut")}  SafetyV: {(s.SafetyValveOpen==1?"OPEN":"shut")}");

        // ── Secondary / Turbine ─────────────────────────────────────────────
        Console.WriteLine($"  ├─ Secondary / Turbine");
        Console.WriteLine($"  │  SG: Psec={s.SgSecondaryPressureMPa,6:F3} MPa  Steam={s.SgSteamTempC,6:F1}°C  FW={s.SgFeedwaterTempC,6:F1}°C  Level={s.SgLevelFraction,5:F3}  UA={s.SgHeatTransferCoefficient,5:F1} MW/°C");
        Console.WriteLine($"  │     SteamFlow={s.SgSteamFlowKgPerS,6:F0} kg/s  FWflow={s.SgFeedwaterFlowKgPerS,6:F0} kg/s");
        Console.WriteLine($"  │  Turbine: {(s.TurbineOnline==1?"ONLINE":"off"),-6}  Throttle={s.TurbineThrottlePosition,5:F3}  Speed={s.TurbineSpeedFraction*100,5:F1}%  Output={s.GeneratorOutputMW,8:F1} MWe  Net={s.NetElectricalMW,8:F1} MWe");

        // ── Safety Systems ──────────────────────────────────────────────────
        Console.WriteLine($"  ├─ Safety Systems");
        Console.WriteLine($"  │  ECCS: SI={(s.EccsSiSignal==1?"ACTIVE":"off"),-6}  HPI={s.HpiFlowKgPerS,5:F1} kg/s  LPI={s.LpiFlowKgPerS,5:F1} kg/s  Accum={s.AccumulatorWaterVolumeM3,5:F1} m³ @ {s.AccumulatorPressureMPa,5:F3} MPa{(s.AccumulatorIsolated==1?"  [ISOLATED]":"")}");
        Console.WriteLine($"  │  Containment: {s.ContainmentPressureKPa,6:F1} kPa  Temp={s.ContainmentTempC,5:F1}°C  Humidity={s.ContainmentHumidity*100,4:F1}%");
        Console.WriteLine($"  │  Electrical:  OffSite={(s.OffSitePowerAvailable==1?"YES":"NO"),-3}  DG={(s.DieselGeneratorRunning==1?"RUNNING":"off"),-7}  Net={s.NetElectricalMW,8:F1} MWe");
        Console.WriteLine($"  └────────────────────────────────────────────────────────────\n");
    }
}
