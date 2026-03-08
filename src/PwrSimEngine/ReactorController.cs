// =============================================================================
// ReactorController.cs — Automation / Control Layer for PWR Simulation
// =============================================================================
// Wraps ReactorEngine to add automatic control loops, reactor protection
// system evaluation, and alarm monitoring.  The engine handles pure physics;
// this layer sets control signals before each tick and evaluates protection
// logic after each tick.
//
// Usage:
//   var engine     = new ReactorEngine();
//   var controller = new ReactorController(engine);
//   controller.Update(dt);   // runs control + physics + protection
//
// The engine can still be used standalone for manual-only control scenarios.
// All auto-enable flags, gains, setpoints, and override flags are stored in
// ReactorState so they survive snapshot/restore and are visible in the API.
// =============================================================================

using System;

namespace PwrSimulator
{
    public sealed class ReactorController
    {
        public ReactorEngine Engine { get; }
        private readonly ReactorDesign _design;

        public ReactorController(ReactorEngine engine)
        {
            Engine = engine ?? throw new ArgumentNullException(nameof(engine));
            _design = engine.Design;
        }

        /// <summary>
        /// Advance the simulation by dt seconds with full automation.
        /// Subdivides into the same 0.05 s max sub-steps as the engine so
        /// that control loops run at the same frequency as the physics.
        /// </summary>
        public void Update(double dt)
        {
            if (dt <= 0) return;

            const double maxSubStep = 0.05;
            int steps = (int)Math.Ceiling(dt / maxSubStep);
            double subDt = dt / steps;

            ref var s = ref Engine.State;

            for (int i = 0; i < steps; i++)
            {
                // Pre-substep: set control signals that physics will read
                UpdatePressurizerControl(ref s, subDt);
                UpdateTurbineGovernor(ref s, subDt);
                UpdateFeedwaterControl(ref s, subDt);
                UpdateEccsSiSignal(ref s);

                // Physics (single substep — won't subdivide further)
                Engine.Tick(subDt);

                // Post-substep: protection & auto-rod
                EvaluateProtectionSystem(ref s);
                UpdateAutoRodControl(ref s, subDt);
            }

            // Alarms only need to run once per update
            UpdateAlarms(ref s);
        }

        // =================================================================
        //  PRESSURIZER CONTROL  (heater auto + spray auto)
        // =================================================================

        private void UpdatePressurizerControl(ref ReactorState s, double dt)
        {
            double P    = s.PressuriserPressureMPa;
            double Pref = s.PrzPressureSetpointMPa;
            double dead = s.PrzAutoDeadbandMPa;
            double gain = s.PrzAutoGain;
            bool pressuriserActive = s.TotalRcpFlowFraction > 0.05;

            // ---- heaters ----
            // Operator command is the floor; auto tops up when below setpoint.
            double heaterPower = s.PressuriserHeaterCommandKW;

            if (s.PrzHeaterAutoEnabled == 1 && pressuriserActive && P < Pref - dead)
            {
                double err = (Pref - P) / Pref;
                double autoKW = Math.Min(
                    s.PressuriserHeaterMaxKW,
                    s.PressuriserHeaterMaxKW * err * gain);
                heaterPower = Math.Max(heaterPower, autoKW);
            }

            s.PressuriserHeaterPowerKW = heaterPower;

            // ---- spray (on if P > setpoint + deadband) ----
            if (s.PrzSprayAutoEnabled == 1 && pressuriserActive && P > Pref + dead)
            {
                s.PressuriserSprayOn = 1;
                double err = (P - Pref) / Pref;
                s.PressuriserSprayFlowKgPerS = Math.Min(
                    _design.PressuriserSprayMaxKgPerS,
                    _design.PressuriserSprayMaxKgPerS * err * gain);
            }
            else
            {
                s.PressuriserSprayOn = 0;
                s.PressuriserSprayFlowKgPerS = 0;
            }
        }

        // =================================================================
        //  TURBINE GOVERNOR  (throttle tracks load setpoint)
        // =================================================================

        private void UpdateTurbineGovernor(ref ReactorState s, double dt)
        {
            if (s.TurbineGovernorAutoEnabled == 0) return;
            if (s.TurbineOnline == 0 || s.TurbineTripFlag == 1) return;

            double steamAvailable = s.SgSteamFlowKgPerS;
            double steamForTarget = s.TurbinePowerTarget
                                    / (_design.TurbineEfficiency * _design.RatedThermalPowerMW + 1e-6)
                                    * steamAvailable;
            double throttleTarget = Math.Clamp(
                steamForTarget / (steamAvailable + 1e-6), 0, 1);

            double tauGov = _design.TurbineTimeConstantS;
            s.TurbineThrottlePosition += (throttleTarget - s.TurbineThrottlePosition)
                                          * dt / tauGov;
            s.TurbineThrottlePosition = Math.Clamp(s.TurbineThrottlePosition, 0, 1);
        }

        // =================================================================
        //  FEEDWATER CONTROL  (3-element: level + flow + steam)
        // =================================================================

        private void UpdateFeedwaterControl(ref ReactorState s, double dt)
        {
            if (s.FeedwaterAutoEnabled == 0) return;
            if (s.TurbineOnline == 0 || s.TurbineTripFlag == 1) return;
            if (s.MainFeedPumpTripped == 1 && s.FeedwaterTripOverride == 0) return;

            double levelErr = s.FwLevelSetpoint - s.SgLevelFraction;
            double flowErr  = s.SgSteamFlowKgPerS - s.SgFeedwaterFlowKgPerS;
            double feedCmd  = s.SgSteamFlowKgPerS
                            + levelErr * s.FwLevelGain
                            + flowErr  * s.FwFlowGain;
            feedCmd = Math.Max(0, feedCmd);
            s.SgFeedwaterFlowKgPerS += (feedCmd - s.SgFeedwaterFlowKgPerS)
                                       * dt / s.FwTimeConstantS;
        }

        // =================================================================
        //  ECCS SI SIGNAL  (determines whether SI is active)
        // =================================================================

        private void UpdateEccsSiSignal(ref ReactorState s)
        {
            bool hasSiTrip = s.ActiveTrips.HasFlag(TripSignal.SafetyInjection);
            bool pressureLoca = s.PrimaryPressureMPa < _design.TripLowPressureMPa
                             && s.Mode != ReactorMode.Shutdown
                             && s.Mode != ReactorMode.Refueling;
            // SI latches once active — must be manually reset by the operator
            s.EccsSiSignal = (byte)((hasSiTrip || pressureLoca || s.EccsSiSignal == 1) ? 1 : 0);
        }

        // =================================================================
        //  REACTOR PROTECTION SYSTEM
        // =================================================================

        private void EvaluateProtectionSystem(ref ReactorState s)
        {
            if (s.ProtectionSystemEnabled == 0) return;
            TripSignal trips = TripSignal.None;

            double totalPower = s.ThermalPowerFraction + s.DecayHeatFraction;

            // high neutron flux (total thermal power vs trip setpoint)
            if (totalPower > s.OverpowerTripSetpoint)
                trips |= TripSignal.HighNeutronFlux;

            // high flux rate
            double rate = Math.Abs(totalPower - s.PreviousTotalPowerFraction)
                          / (s.TimeStepSeconds + 1e-30);
            if (rate > _design.TripHighFluxRate && totalPower > 0.05)
                trips |= TripSignal.HighNeutronFluxRate;

            // high pressure
            if (s.PrimaryPressureMPa > s.HighPressureTripMPa)
                trips |= TripSignal.HighPressure;

            // low pressure (only armed above 2% power — not at cold shutdown)
            if (s.PrimaryPressureMPa < s.LowPressureTripMPa && totalPower > 0.02)
                trips |= TripSignal.LowPressure;

            // high Tavg
            if (s.TavgC > _design.TripHighTavgC)
                trips |= TripSignal.HighTemperature;

            // low flow (only armed above 5% power)
            if (s.PrimaryFlowFraction < s.LowFlowTripFraction && totalPower > 0.05)
                trips |= TripSignal.LowFlow;

            // high containment pressure
            if (s.ContainmentPressureKPa > _design.TripHighContainmentKPa)
                trips |= TripSignal.HighContainmentPressure;

            // SG level (only armed above 2% power)
            if (totalPower > 0.02)
            {
                if (s.SgLevelFraction < _design.TripLowSgLevel)
                    trips |= TripSignal.LowSteamGeneratorLevel;
                if (s.SgLevelFraction > _design.TripHighSgLevel)
                    trips |= TripSignal.HighSteamGeneratorLevel;
            }

            // high startup rate (only in source/intermediate range)
            if (Math.Abs(s.StartupRateDPM) > _design.TripHighStartupRate
                && totalPower < 0.05 && totalPower > 1e-6)
                trips |= TripSignal.HighStartupRate;

            // low DNBR (only meaningful with flow and above 5% power)
            if (s.DNBRatio < _design.TripLowDNBR
                && s.PrimaryFlowFraction > 0.1
                && totalPower > 0.05)
                trips |= TripSignal.LowDNBRatio;

            // turbine trip (only meaningful when turbine was supplying load)
            if (s.TurbineTripFlag == 1 && totalPower > 0.15)
                trips |= TripSignal.TurbineTrip;

            // loss of off-site power (only trip if above 2% power)
            if (s.OffSitePowerAvailable == 0 && totalPower > 0.02)
                trips |= TripSignal.LossOfOffSitePower;

            s.ActiveTrips = trips;

            // Bypassed trips still show as active but are excluded from the scram decision.
            TripSignal scramTrips = trips & ~s.TripBypassFlags;
            if (scramTrips != TripSignal.None && s.Scram == ScramState.Normal)
            {
                Engine.ExecuteScram(ScramState.AutoScram);
                Engine.LogEvent("RPS", $"Automatic reactor trip: {scramTrips}");
            }
        }

        // =================================================================
        //  AUTOMATIC ROD CONTROL  (Tavg program)
        // =================================================================

        private void UpdateAutoRodControl(ref ReactorState s, double dt)
        {
            if (s.RodControlAutoMode == 0) return;
            // Block auto-rod during scram unless override active
            if (s.Scram != ScramState.Normal && s.RodScramOverride == 0) return;

            // Tavg program: linear from 291.7 °C at 0% to 308.2 °C at 100%
            double targetTavg = _design.DesignCoreInletTempC
                              + _design.DesignCoreDeltaT / 2.0 * s.ThermalPowerFraction;
            s.TavgProgramSetpointC = targetTavg;

            double error = s.TavgC - targetTavg; // positive = too hot
            double gain  = s.RodAutoGainStepsPerMinPerC;
            double dead  = s.RodAutoDeadbandC;

            double speed = 0;
            if (Math.Abs(error) > dead)
            {
                speed = -error * gain; // steps/min proportional
                speed = Math.Clamp(speed, -s.MaxRodSpeedStepsPerMin,
                                          s.MaxRodSpeedStepsPerMin);
            }

            double newTarget = s.RodBankPosition[3] + speed * dt / 60.0;
            newTarget = Math.Clamp(newTarget, 0, _design.MaxRodSteps);
            s.RodBankTargetPosition[3] = newTarget;
            s.RodSpeedStepsPerMin = Math.Abs(speed);
        }

        // =================================================================
        //  ALARMS
        // =================================================================

        private void UpdateAlarms(ref ReactorState s)
        {
            if (s.NetElectricalMW < 0 && s.OffSitePowerAvailable == 0
                && s.DieselGeneratorRunning == 0)
            {
                Engine.LogEvent("ELEC", "Station blackout condition!");
            }
        }
    }
}
