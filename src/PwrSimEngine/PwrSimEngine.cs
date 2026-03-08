// =============================================================================
// PwrSimEngine.cs — Pressurized Water Reactor Simulation Engine
// =============================================================================
// A high-fidelity, real-time PWR simulation engine suitable for training
// simulators, games, or control-system prototyping.  Every piece of mutable
// state lives inside the single value-type ReactorState so that the full
// simulation can be snapshot / restored / diffed trivially.
//
// Physics models included:
//   • Point-kinetics with six delayed-neutron groups (U-235 thermal data)
//   • Xenon-135 / Iodine-135 poison transient
//   • Samarium-149 / Promethium-149 poison transient
//   • Doppler (fuel temperature) reactivity feedback
//   • Moderator temperature reactivity feedback
//   • Coolant void-fraction reactivity feedback
//   • Boron-10 reactivity worth
//   • Control-rod bank reactivity (four banks, differential worth curves)
//   • Radial fuel-rod thermal model  (centreline → surface → gap → clad → coolant)
//   • Primary-loop hydraulics  (hot-leg, cold-leg, ΔT, flow, pressure)
//   • Pressuriser model  (heaters, spray, PORV / safety valves, level)
//   • U-tube steam-generator heat transfer
//   • Secondary-side steam drum  (pressure, level, quality)
//   • Simplified turbine-governor & condenser
//   • Main coolant pump model  (coast-down, locked-rotor)
//   • Chemical & Volume Control System  (boration / dilution)
//   • ANS-5.1-2005 decay-heat model  (23-group exponential fit)
//   • Fuel / cladding integrity tracking  (Zr-4 oxidation, DNB margin)
//   • Reactor Protection System  (multi-setpoint automatic trip logic)
//   • Emergency Core Cooling System  (HPI, LPI, accumulators)
// =============================================================================

using System;
using System.IO;
using System.Runtime.InteropServices;
using System.Runtime.Serialization.Formatters.Binary;
using System.Collections.Generic;

namespace PwrSimulator
{
    // =========================================================================
    //  Enumerations
    // =========================================================================

    public enum ReactorMode : byte
    {
        Shutdown,           // sub-critical, all rods in
        Startup,            // approaching criticality
        PowerAscension,     // increasing power
        SteadyState,        // at target power
        PowerDescension,    // decreasing power
        HotStandby,         // critical at zero power, Tavg maintained
        Tripped,            // post-trip
        Refueling           // vessel head removed
    }

    public enum ScramState : byte
    {
        Normal,
        ManualScram,
        AutoScram
    }

    [Flags]
    public enum TripSignal : uint
    {
        None                    = 0,
        HighNeutronFluxRate     = 1 << 0,
        HighNeutronFlux         = 1 << 1,
        LowNeutronFlux         = 1 << 2,
        HighPressure            = 1 << 3,
        LowPressure             = 1 << 4,
        HighTemperature         = 1 << 5,
        LowFlow                 = 1 << 6,
        HighContainmentPressure = 1 << 7,
        LowSteamGeneratorLevel  = 1 << 8,
        HighSteamGeneratorLevel = 1 << 9,
        TurbineTrip             = 1 << 10,
        LossOfOffSitePower      = 1 << 11,
        SafetyInjection         = 1 << 12,
        ManualTrip              = 1 << 13,
        HighStartupRate          = 1 << 14,
        LowDNBRatio              = 1 << 15,
        TurbineOverspeedWarning  = 1 << 16,  // speed 105–110% (warning before emergency OST)
        TurbineUnderspeed        = 1 << 17   // speed < 95% while online and at power
    }

    public enum PumpState : byte
    {
        Off,
        Starting,
        Running,
        CoastingDown,
        LockedRotor
    }

    public enum ValveState : byte
    {
        Closed,
        Opening,
        Open,
        Closing
    }

    public enum EccsMode : byte
    {
        Standby,
        HighPressureInjection,
        Accumulator,
        LowPressureInjection,
        Recirculation
    }

    /// <summary>Progressive core damage states — ratchet forward only.</summary>
    public enum CoreDamageState : byte
    {
        Normal        = 0,  // no damage
        CladFailure   = 1,  // Zr cladding > 1204 °C; oxidation runaway, FP release begins
        FuelMelting   = 2,  // UO₂ centreline > 2840 °C; liquefaction begins
        CoreDamage    = 3,  // >15 % fuel severely damaged; core geometry degrading
        Meltdown      = 4,  // >50 % core destroyed; corium pool forming
        VesselFailure = 5,  // vessel lower head fails; ex-vessel progression
    }

    // =========================================================================
    //  Fixed-size array helpers  (structs cannot contain managed arrays)
    // =========================================================================

    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct Array6
    {
        public double V0, V1, V2, V3, V4, V5;
        public double this[int i]
        {
            get => i switch { 0 => V0, 1 => V1, 2 => V2, 3 => V3, 4 => V4, 5 => V5,
                              _ => throw new IndexOutOfRangeException() };
            set { switch (i) { case 0: V0 = value; break; case 1: V1 = value; break;
                                case 2: V2 = value; break; case 3: V3 = value; break;
                                case 4: V4 = value; break; case 5: V5 = value; break;
                                default: throw new IndexOutOfRangeException(); } }
        }
        public const int Length = 6;
    }

    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct Array4
    {
        public double V0, V1, V2, V3;
        public double this[int i]
        {
            get => i switch { 0 => V0, 1 => V1, 2 => V2, 3 => V3,
                              _ => throw new IndexOutOfRangeException() };
            set { switch (i) { case 0: V0 = value; break; case 1: V1 = value; break;
                                case 2: V2 = value; break; case 3: V3 = value; break;
                                default: throw new IndexOutOfRangeException(); } }
        }
        public const int Length = 4;
    }

    [Serializable]
    [StructLayout(LayoutKind.Sequential)]
    public struct Array23
    {
        public double V00, V01, V02, V03, V04, V05, V06, V07, V08, V09;
        public double V10, V11, V12, V13, V14, V15, V16, V17, V18, V19;
        public double V20, V21, V22;
        public double this[int i]
        {
            get => i switch {
                0=>V00,1=>V01,2=>V02,3=>V03,4=>V04,5=>V05,6=>V06,7=>V07,8=>V08,9=>V09,
                10=>V10,11=>V11,12=>V12,13=>V13,14=>V14,15=>V15,16=>V16,17=>V17,18=>V18,19=>V19,
                20=>V20,21=>V21,22=>V22,
                _ => throw new IndexOutOfRangeException() };
            set { switch(i) {
                case 0:V00=value;break;case 1:V01=value;break;case 2:V02=value;break;
                case 3:V03=value;break;case 4:V04=value;break;case 5:V05=value;break;
                case 6:V06=value;break;case 7:V07=value;break;case 8:V08=value;break;
                case 9:V09=value;break;case 10:V10=value;break;case 11:V11=value;break;
                case 12:V12=value;break;case 13:V13=value;break;case 14:V14=value;break;
                case 15:V15=value;break;case 16:V16=value;break;case 17:V17=value;break;
                case 18:V18=value;break;case 19:V19=value;break;case 20:V20=value;break;
                case 21:V21=value;break;case 22:V22=value;break;
                default:throw new IndexOutOfRangeException(); } }
        }
        public const int Length = 23;
    }

    // =========================================================================
    //  ReactorState  —  the single monolithic, serialisable value type
    // =========================================================================

    [Serializable]
    [StructLayout(LayoutKind.Sequential, Pack = 8)]
    public struct ReactorState
    {
        // ---- simulation bookkeeping ----------------------------------------
        public double SimulationTimeSeconds;        // elapsed seconds
        public double TimeStepSeconds;              // last dt used
        public long   TickCount;                    // total ticks

        // ---- reactor mode & protection -------------------------------------
        public ReactorMode Mode;
        public ScramState   Scram;
        public TripSignal   ActiveTrips;
        public TripSignal   TripBypassFlags;         // trips bypassed from causing a scram (still shown as active)
        public double       TripTimestamp;          // sim-time of last trip
        public byte         ProtectionSystemEnabled; // 1 = enabled

        // ---- neutronics (point-kinetics) -----------------------------------
        public double NeutronPopulation;            // normalised: 1.0 = rated
        public double ThermalPowerFraction;         // fraction of rated (0–~1.2)
        public double ThermalPowerMW;               // absolute MW-thermal
        public double DecayHeatFraction;            // fraction of rated
        public double DecayHeatMW;
        public double TotalPowerMW;                 // fission + decay
        public double StartupRateDPM;               // decades per minute (SUR)

        public Array6 PrecursorConcentrations;      // Ci, normalised

        // ---- reactivity budget (all in Δk/k) ------------------------------
        public double RhoTotal;                     // net reactivity
        public double RhoControlRods;
        public double RhoDoppler;
        public double RhoModerator;
        public double RhoBoron;
        public double RhoXenon;
        public double RhoSamarium;
        public double RhoVoid;
        public double RhoManualBias;                // operator "shim" knob

        // ---- delayed-neutron data (held per-state for moddability) --------
        public double BetaTotal;                    // total β
        public double PromptNeutronLifetime;        // Λ (seconds)

        // ---- fission-product poisons (atoms / cm³ in fuel) ----------------
        public double Iodine135;                    // I-135 concentration
        public double Xenon135;                     // Xe-135 concentration
        public double Promethium149;                // Pm-149 concentration
        public double Samarium149;                  // Sm-149 concentration
        public double XenonEquilibrium;             // tracking target
        public double SamariumEquilibrium;

        // ---- decay-heat model (ANS-5.1 23-group) --------------------------
        public Array23 DecayHeatGroupEnergy;        // accumulated E per group

        // ---- fuel rod thermal model ----------------------------------------
        public double FuelCentrelineTemp;           // °C
        public double FuelAverageTemp;              // °C
        public double FuelSurfaceTemp;              // °C
        public double GapConductance;               // W/m²·K
        public double CladInnerTemp;                // °C
        public double CladOuterTemp;                // °C
        public double CoolantBulkTemp;              // °C  (core-average)
        public double LinearHeatRate;               // W/m  (peak rod)
        public double PeakingFactor;                // radial × axial

        // ---- cladding & fuel integrity -------------------------------------
        public double CladOxideThickness;           // µm  Zr-4 oxide layer
        public double CladHoopStress;               // MPa
        public double CladTemperatureLimit;         // °C  (design: 1204)
        public double FuelMeltFraction;             // 0 – 1

        // ---- core damage / meltdown tracking ----------------------------------
        public CoreDamageState CoreDamageState;     // progressive damage level
        public double CoreDamageFraction;           // 0–1, integrated; ratchet-forward
        public double ZirconiumOxidizedFraction;    // 0–1 (high-T Zr-steam rxn)
        public double CoreExothermicHeatMW;         // MW from Zr-steam reaction
        public double HydrogenGenerationKg;         // cumulative H₂ produced
        public double FissionProductReleaseFraction;// 0–1, fraction of FP in primary

        public double DNBRatio;                     // departure-from-nucleate-boiling ratio
        public double FuelBurnupMWdPerTonne;        // MWd/tHM

        // ---- primary coolant loop ------------------------------------------
        public double PrimaryPressureMPa;           // MPa (nominal ~15.5)
        public double PrimaryPressurePsi;           // convenience
        public double PrimaryFlowRateKgPerS;        // total loop
        public double PrimaryFlowFraction;          // fraction of design
        public double CoreInletTempC;               // °C
        public double CoreOutletTempC;              // °C
        public double CoreDeltaT;                   // °C
        public double TavgC;                        // (Tin+Tout)/2
        public double TrefC;                        // reference T for MTC calc
        public double CoolantDensityKgPerM3;        // core-average
        public double CoolantVoidFraction;          // 0 = all liquid
        public double SubcoolingMarginC;            // Tsat - Tout

        // ---- pressuriser ---------------------------------------------------
        public double PressuriserPressureMPa;
        public double PressuriserTempC;
        public double PressuriserLevelFraction;     // 0 – 1
        public double PressuriserHeaterPowerKW;
        public double PressuriserHeaterCommandKW;   // operator-commanded setpoint
        public double PressuriserHeaterMaxKW;
        public byte   PressuriserSprayOn;           // 0/1
        public double PressuriserSprayFlowKgPerS;
        public byte   PorvOpen;                     // pilot-operated relief valve
        public byte   SafetyValveOpen;              // code safety valve
        public double PressuriserVolumeM3;          // total volume
        public double PressuriserSteamFraction;     // steam/water in press.

        // ---- reactor coolant pumps (4 loops) --------------------------------
        public Array4 RcpSpeed;                     // fraction of rated
        public Array4 RcpState;                     // cast to PumpState
        public Array4 RcpFlowFraction;              // per-loop fraction
        public Array4 RcpCurrentAmps;               // electrical draw
        public double TotalRcpFlowFraction;

        // ---- chemical & volume control system (CVCS) -----------------------
        public double BoronConcentrationPPM;        // soluble boron in coolant
        public double BoronConcentrationTarget;     // operator setpoint
        public double LetdownFlowKgPerS;
        public double ChargingFlowKgPerS;
        public double BoricAcidTankConcentration;   // ppm of makeup
        public double CvcsOperatingMode;            // 0=manual, 1=auto dilute, 2=auto borate

        // ---- control rods (4 banks: A, B, C, D) ----------------------------
        //       position in "steps withdrawn": 0 = fully inserted, 228 = fully out
        public Array4 RodBankPosition;              // steps (0-228)
        public Array4 RodBankTargetPosition;        // commanded target
        public double RodSpeedStepsPerMin;          // current stepping speed
        public double MaxRodSpeedStepsPerMin;       // limit (typ 72 in, 48 out PWR)
        public byte   RodControlAutoMode;           // 0=manual, 1=auto (Tavg program)
        public double TavgProgramSetpointC;         // linear Tavg vs power
        // Rod auto-control parameters
        public double RodAutoGainStepsPerMinPerC;   // proportional gain (default 2.0)
        public double RodAutoDeadbandC;             // deadband (default 0.5 °C)
        public byte   RodScramOverride;             // 1 = allow rod ops while scrammed

        // ---- auto-control enable flags ----
        public byte   PrzHeaterAutoEnabled;         // 0=manual only, 1=auto top-up (default 1)
        public byte   PrzSprayAutoEnabled;          // 0=manual only, 1=auto spray (default 1)
        public byte   TurbineGovernorAutoEnabled;   // 0=manual throttle, 1=auto governor (default 1)
        public byte   FeedwaterAutoEnabled;         // 0=manual, 1=3-element control (default 1)
        public byte   FeedwaterTripOverride;        // 1 = ignore feed-pump trip flag

        // ---- configurable setpoints / gains ----
        public double PrzPressureSetpointMPa;       // pressurizer pressure target (default design)
        public double PrzAutoGain;                  // heater/spray P-gain (default 10.0)
        public double PrzAutoDeadbandMPa;           // heater/spray deadband (default 0.07 MPa)
        public double FwLevelSetpoint;              // SG level target (fraction, default 0.50)
        public double FwLevelGain;                  // level error gain kg/s per fraction (default 50)
        public double FwFlowGain;                   // flow mismatch gain (default 0.5)
        public double FwTimeConstantS;              // FW response lag (default 4.0 s)

        // ---- steam generator (single effective SG representing all loops) ---
        public double SgPrimaryInletTempC;
        public double SgPrimaryOutletTempC;
        public double SgSecondaryPressureMPa;
        public double SgSteamTempC;
        public double SgFeedwaterTempC;
        public double SgLevelFraction;              // narrow-range
        public double SgLevelFractionWide;          // wide-range
        public double SgHeatTransferCoefficient;    // overall UA (MW/°C)
        public double SgSteamFlowKgPerS;
        public double SgFeedwaterFlowKgPerS;
        public double SgTubeFoulingFactor;          // 1.0 = clean
        public double SgBlowdownFlowKgPerS;

        // ---- secondary side / turbine --------------------------------------
        public double MainSteamPressureMPa;
        public double MainSteamTempC;
        public double TurbineLoadMW;                // electrical output
        public double TurbineSpeedRPM;
        public double TurbineSpeedFraction;         // fraction of rated
        public byte   TurbineTripFlag;
        public double CondenserVacuumKPa;
        public double CondenserTempC;
        public double TurbineThrottlePosition;      // 0-1 (governor valve)
        public double TurbinePowerTarget;           // MW-e setpoint
        public byte   TurbineOnline;
        public byte   GeneratorBreakerClosed;        // 1 = synchronized & connected to grid
        public byte   GridTiedMode;                  // 1 = infinite-bus (grid-tied), 0 = islanded (isochronous)
        public double GridFrequencyHz;               // grid frequency (Hz), nominal 60.0
        public double TurbineMechanicalPowerMW;      // shaft mechanical power (before generator losses)
        public double TurbineInletEnthalpyKJPerKg;   // h at HP turbine admission (kJ/kg)
        public double TurbineExhaustEnthalpyKJPerKg; // h at LP exhaust / condenser inlet (kJ/kg)
        public double TurbineSteamConsumptionKgPerS; // steam mass flow through turbine (kg/s)
        public double CondenserHeatRejectionMW;      // heat duty to condenser cooling water (MW)
        // ---- turbine annunciator alarms ------------------------------------
        public byte   TurbineUnderspeedAlarm;        // speed < 95% rated while running
        public byte   TurbineOverspeedWarn;          // speed 105–110% (pre-trip warning)
        public byte   TurbineBreakerAlarm;           // breaker open unexpectedly at power
        // ---- islanded governor integrator ----------------------------------
        public double IsochrIntegrator;              // PI integrator state for isochronous governor

        // ---- feed-water system ---------------------------------------------
        public double MainFeedPumpSpeedFraction;
        public double MainFeedPumpFlowKgPerS;
        public double FeedwaterTempC;               // after FW heaters
        public byte   MainFeedPumpTripped;

        // ---- emergency core cooling system ---------------------------------
        public EccsMode EccsStatus;
        public double AccumulatorPressureMPa;
        public double AccumulatorWaterVolumeM3;
        public byte   AccumulatorIsolated;          // 1 = isolated
        public double HpiFlowKgPerS;
        public double LpiFlowKgPerS;
        public double SitWaterTempC;                // safety-injection tank

        // ---- containment (simplified) --------------------------------------
        public double ContainmentPressureKPa;
        public double ContainmentTempC;
        public double ContainmentHumidity;          // 0-1
        public double ContainmentH2FractionPct;     // vol% hydrogen (flammable >4%, detonable 8-18%)

        // ---- electrical (simplified) ---------------------------------------
        public double GeneratorOutputMW;
        public double HousekeepingLoadMW;
        public double NetElectricalMW;
        public byte   OffSitePowerAvailable;
        public byte   DieselGeneratorRunning;

        // ---- operator-settable limiters ------------------------------------
        public double OverpowerTripSetpoint;        // fraction (e.g. 1.09)
        public double OvertemperatureTripDeltaT;    // °C
        public double HighPressureTripMPa;
        public double LowPressureTripMPa;
        public double LowFlowTripFraction;

        // ---- controller state (set by ReactorController, read by physics) --
        public double PreviousTotalPowerFraction;   // for rate-of-change calcs
        public byte   EccsSiSignal;                 // 1 = SI active
    }

    // =========================================================================
    //  Reactor design parameters  (immutable for a given plant design)
    // =========================================================================

    public sealed class ReactorDesign
    {
        // ---- core ----------------------------------------------------------
        public double RatedThermalPowerMW   = 3411.0;   // Westinghouse 4-loop
        public double RatedElectricalMW     = 1150.0;
        public int    FuelAssemblies        = 193;
        public int    RodsPerAssembly       = 264;
        public double ActiveFuelLengthM     = 3.66;
        public double FuelPelletRadiusM     = 0.004096;
        public double CladInnerRadiusM      = 0.004178;
        public double CladOuterRadiusM      = 0.004750;
        public double FuelRodPitchM         = 0.01260;

        // ---- neutronics constants ------------------------------------------
        public double BetaTotal             = 0.0065;    // delayed neutron fraction
        public double PromptNeutronLifetime = 2.0e-5;    // seconds
        // six-group delayed neutron data  (U-235 thermal fission)
        public double[] Beta_i  = { 0.000215, 0.001424, 0.001274, 0.002568, 0.000748, 0.000273 };
        public double[] Lambda_i = { 0.0124, 0.0305, 0.111, 0.301, 1.14, 3.01 }; // s⁻¹

        // ---- reactivity coefficients --------------------------------------
        public double DopplerCoefficient    = -2.8e-5;   // Δk/k per °C  (fuel avg)
        public double ModeratorTempCoeff    = -3.5e-4;   // Δk/k per °C  (at BOL, depends on boron)
        public double BoronWorth            = -1.0e-4;   // Δk/k per ppm
        public double VoidCoefficient       = -1.5e-3;   // Δk/k per % void
        public double TotalRodWorthPCM      = 8000.0;    // total all banks (pcm)
        public double[] BankWorthFraction   = { 0.08, 0.12, 0.20, 0.60 }; // A,B,C,D
        public int    MaxRodSteps           = 228;

        // ---- fission-product data ------------------------------------------
        public double GammaIodine           = 0.0639;    // fission yield I-135
        public double GammaXenon            = 0.00237;   // direct Xe-135 yield
        public double LambdaIodine          = 2.87e-5;   // s⁻¹  (t½ ≈ 6.7 h)
        public double LambdaXenon           = 2.09e-5;   // s⁻¹  (t½ ≈ 9.2 h)
        public double SigmaXenon            = 2.65e-18;  // cm²  microscopic σ_a
        public double GammaPromethium       = 0.0113;    // Pm-149 yield
        public double LambdaPromethium      = 3.63e-6;   // s⁻¹  (t½ ≈ 53 h)
        public double SigmaSamarium         = 4.02e-20;  // cm²

        // macroscopic reference: Σf·Φ at rated power  (normalised so NP = 1)
        public double SigmaFPhiRated        = 3.20e13;   // fissions/cm³/s representative

        // ---- thermal-hydraulic design --------------------------------------
        public double DesignPrimaryFlowKgPerS   = 18630.0;   // 4-loop total
        public double DesignCoreDeltaT          = 33.0;       // °C
        public double DesignCoreInletTempC      = 291.7;
        public double DesignCoreOutletTempC     = 324.7;
        public double DesignTavgC               = 308.2;
        public double DesignPrimaryPressureMPa  = 15.51;
        public double CoolantSpecificHeatJPerKgC= 5600.0;     // avg for subcooled ~315 °C
        public double CoolantDensityNominal     = 720.0;      // kg/m³ nominal
        public double FuelThermalConductivity   = 3.5;        // W/m·K (UO₂ ≈ 3-5)
        public double GapConductanceNominal     = 5700.0;     // W/m²·K
        public double CladConductivity          = 21.5;       // W/m·K  Zr-4
        public double CoolantHTCNominal         = 34000.0;    // W/m²·K (Dittus-Boelter)
        public double SaturationTempAtDesignP   = 344.8;      // °C at 15.51 MPa

        // ---- reference state for reactivity deviations --------------------
        public double DesignFuelAverageTempC    = 923.0;      // °C  at rated power
        public double DesignBoronPPM            = 800.0;      // ppm at rated power, equil Xe

        // ---- pressuriser ---------------------------------------------------
        public double PressuriserVolumeM3       = 51.0;
        public double PressuriserHeaterMaxKW    = 1800.0;
        public double PressuriserSprayMaxKgPerS = 28.0;
        public double PorvSetpointMPa           = 15.72;      // open
        public double PorvResetMPa              = 15.51;
        public double SafetyValveSetpointMPa    = 17.24;
        public double PressuriserHeaterDeadband  = 0.07;       // MPa

        // ---- primary system thermal mass -----------------------------------
        public double RcsWaterMassKg            = 250_000.0;  // total RCS water inventory
        public double RcpRatedPowerMW           = 6.0;        // per-pump shaft heat input at full speed

        // ---- steam generator -----------------------------------------------
        public double SgDesignUAMwPerC          = 66.0;       // overall heat-xfer × area
        public double SgDesignSecondaryPressure  = 6.90;       // MPa  (≈1000 psia)
        public double SgDesignSteamTempC         = 284.0;
        public double SgDesignFeedTempC          = 226.7;
        public double SgWaterMassKg             = 52000.0;

        // ---- turbine -------------------------------------------------------
        public double TurbineRatedSpeedRPM       = 1800.0;    // 4-pole, 60 Hz
        public double TurbineTimeConstantS       = 5.0;       // governor servo time constant (s)
        public double TurbineEfficiency          = 0.335;     // gross cycle thermal efficiency ηth
        public double CondenserDesignTempC        = 33.0;     // °C at rated heat rejection
        public double CondenserDesignVacuumKPa    = 5.0;      // kPa saturation pressure at design Tcond
        public double TurbineInertiaConstantH    = 9.0;       // H constant (MWs/MVA) — combined rotor inertia
                                                              // 4-loop PWR: HP+3×LP cylinders + generator ≈ 8–10 s
        public double TurbineGovernorDroop       = 0.05;      // 5 % speed droop (typical utility governor)
        public double TurbineDampingD            = 2.0;       // per-unit mechanical damping coefficient D
        public double TurbineOverspeedTripFraction   = 1.10;  // emergency overspeed trip at 110 %
        public double TurbineOverspeedWarnFraction  = 1.05;  // RPS trip warning at 105 %
        public double TurbineUnderspeedAlarmFraction = 0.95; // RPS trip / alarm at 95 % rated
        public double TurbineSyncKpow            = 20.0;     // grid synchronising torque coefficient
        public double GeneratorEfficiency        = 0.985;    // η_gen (copper + iron losses)

        // ---- protection setpoints ------------------------------------------
        public double TripHighFlux              = 1.09;        // fraction of rated
        public double TripHighFluxRate          = 0.05;        // Δ(fraction)/s
        public double TripHighPressureMPa       = 16.20;
        public double TripLowPressureMPa        = 13.10;
        public double TripLowFlow               = 0.87;        // fraction
        public double TripHighTavgC             = 327.0;
        public double TripHighContainmentKPa    = 115.0;
        public double TripLowSgLevel            = 0.15;
        public double TripHighSgLevel           = 0.90;
        public double TripHighStartupRate       = 5.0;         // DPM
        public double TripLowDNBR              = 1.30;

        // ---- ECCS ----------------------------------------------------------
        public double AccumulatorDesignPressureMPa = 4.24;
        public double AccumulatorDesignVolumeM3    = 28.3;  // per accumulator (×4)
        public double HpiDesignFlowKgPerS          = 63.0;
        public double LpiDesignFlowKgPerS          = 630.0;
        public double SitWaterBoronPPM             = 2400.0;

        // ---- decay-heat  (ANS-5.1-2005 23 groups)  α·exp(-λt) -------------
        //       after shutdown: Pd/P0 = Σ αi · exp(-λi · t)
        //       where t is seconds since shutdown
        //  Corrected ANS-5.1-2005 23-group decay heat fit for U-235.
        //  Σ αi ≈ 0.0662 — i.e. ~6.6 % of operating power at instant of
        //  shutdown after infinite irradiation.  (The lambdas below give the
        //  time-dependent shape; the alphas give the magnitude.)
        public double[] DecayAlpha = {
            4.886e-02, 1.105e-02, 1.894e-03, 2.396e-03, 8.610e-04,
            3.964e-04, 1.095e-04, 2.066e-04, 1.876e-04, 1.335e-04,
            6.260e-05, 2.603e-05, 1.832e-05, 1.172e-05, 3.842e-06,
            1.988e-06, 1.105e-06, 2.470e-07, 8.530e-08, 3.597e-08,
            1.765e-08, 4.554e-09, 7.639e-10 };
        public double[] DecayLambda = {
            2.289e+01, 5.667e-01, 4.632e-01, 6.321e-02, 1.823e-02,
            4.122e-03, 1.474e-03, 6.314e-04, 2.092e-04, 1.008e-04,
            4.044e-05, 1.649e-05, 7.817e-06, 3.512e-06, 1.067e-06,
            5.180e-07, 2.480e-07, 6.520e-08, 2.020e-08, 6.690e-09,
            2.470e-09, 5.490e-10, 6.380e-11 };
    }

    // =========================================================================
    //  Simulation event system
    // =========================================================================

    public readonly struct SimulationEvent
    {
        public readonly double Timestamp;
        public readonly string Category;
        public readonly string Message;
        public SimulationEvent(double t, string cat, string msg)
        { Timestamp = t; Category = cat; Message = msg; }
    }

    // =========================================================================
    //  ReactorEngine  —  the main simulation driver
    // =========================================================================

    public sealed class ReactorEngine
    {
        // -- public accessors ------------------------------------------------
        public ref ReactorState State => ref _state;
        public ReactorDesign Design { get; }
        public IReadOnlyList<SimulationEvent> EventLog => _events;

        // -- internals -------------------------------------------------------
        private ReactorState _state;
        private readonly ReactorDesign _design;
        private readonly List<SimulationEvent> _events = new();

        // =====================================================================
        //  Construction
        // =====================================================================

        public ReactorEngine() : this(new ReactorDesign()) { }

        public ReactorEngine(ReactorDesign design)
        {
            _design = design ?? throw new ArgumentNullException(nameof(design));
            Design = _design;
            _state = new ReactorState();
            InitialiseColdShutdown();
        }

        // =====================================================================
        //  State snapshot / restore
        // =====================================================================

        /// <summary>Return a by-value copy of the entire simulation state.</summary>
        public ReactorState Snapshot() => _state;            // struct copy

        /// <summary>Overwrite the simulation state with a previously saved copy.</summary>
        public void Restore(ReactorState saved) => _state = saved;

        /// <summary>Serialise state to a byte array for disk / network.</summary>
        public byte[] SerialiseState()
        {
            int size = Marshal.SizeOf<ReactorState>();
            byte[] buf = new byte[size];
            GCHandle h = GCHandle.Alloc(buf, GCHandleType.Pinned);
            try { Marshal.StructureToPtr(_state, h.AddrOfPinnedObject(), false); }
            finally { h.Free(); }
            return buf;
        }

        /// <summary>Deserialise state from a byte array.</summary>
        public void DeserialiseState(byte[] data)
        {
            int size = Marshal.SizeOf<ReactorState>();
            if (data.Length < size)
                throw new ArgumentException($"Buffer too small: need {size}, got {data.Length}");
            GCHandle h = GCHandle.Alloc(data, GCHandleType.Pinned);
            try { _state = Marshal.PtrToStructure<ReactorState>(h.AddrOfPinnedObject()); }
            finally { h.Free(); }
        }

        // =====================================================================
        //  Cold-shutdown initialisation
        // =====================================================================

        public void InitialiseColdShutdown()
        {
            _state = default;

            _state.SimulationTimeSeconds = 0;
            _state.TickCount = 0;

            _state.Mode = ReactorMode.Shutdown;
            _state.Scram = ScramState.Normal;
            _state.ProtectionSystemEnabled = 1;

            // neutronics at source level
            _state.NeutronPopulation = 1e-10;
            _state.BetaTotal = _design.BetaTotal;
            _state.PromptNeutronLifetime = _design.PromptNeutronLifetime;

            // precursors at equilibrium with source
            for (int i = 0; i < Array6.Length; i++)
                _state.PrecursorConcentrations[i] =
                    _design.Beta_i[i] / (_design.PromptNeutronLifetime * _design.Lambda_i[i])
                    * _state.NeutronPopulation;

            // zero poisons at cold start (fresh core)
            _state.Iodine135 = 0.0;
            _state.Xenon135 = 0.0;
            _state.Promethium149 = 0.0;
            _state.Samarium149 = 0.0;

            // all rods in
            for (int i = 0; i < Array4.Length; i++)
            {
                _state.RodBankPosition[i] = 0;
                _state.RodBankTargetPosition[i] = 0;
            }
            _state.RodSpeedStepsPerMin = 0;
            _state.MaxRodSpeedStepsPerMin = 72;
            _state.RodAutoGainStepsPerMinPerC = 2.0;
            _state.RodAutoDeadbandC          = 0.5;
            _state.RodScramOverride          = 0;

            // auto-control defaults — all enabled, gains at calibrated values
            _state.PrzHeaterAutoEnabled      = 1;
            _state.PrzSprayAutoEnabled       = 1;
            _state.TurbineGovernorAutoEnabled = 1;
            _state.FeedwaterAutoEnabled      = 1;
            _state.FeedwaterTripOverride     = 0;
            _state.PrzPressureSetpointMPa    = _design.DesignPrimaryPressureMPa;
            _state.PrzAutoGain               = 10.0;
            _state.PrzAutoDeadbandMPa        = _design.PressuriserHeaterDeadband;
            _state.FwLevelSetpoint           = 0.50;
            _state.FwLevelGain               = 50.0;
            _state.FwFlowGain                = 0.5;
            _state.FwTimeConstantS           = 4.0;

            // primary at cold conditions
            _state.PrimaryPressureMPa = 0.1;          // atmospheric
            _state.CoreInletTempC = 30.0;
            _state.CoreOutletTempC = 30.0;
            _state.TavgC = 30.0;
            _state.TrefC = _design.DesignTavgC;
            _state.PrimaryFlowRateKgPerS = 0;
            _state.CoolantDensityKgPerM3 = 995.0;     // near room temp
            _state.CoolantVoidFraction = 0;
            _state.SubcoolingMarginC = SaturationTemperature(0.1) - 30.0;
            _state.PrimaryFlowFraction = 0;
            _state.PrimaryPressurePsi = 0.1 * 145.038;

            // pressuriser (cold, depressurised)
            _state.PressuriserPressureMPa = 0.1;
            _state.PressuriserTempC = 30.0;
            _state.PressuriserLevelFraction = 0.25;
            _state.PressuriserHeaterMaxKW = _design.PressuriserHeaterMaxKW;
            _state.PressuriserVolumeM3 = _design.PressuriserVolumeM3;

            // RCPs off
            for (int i = 0; i < 4; i++)
            {
                _state.RcpSpeed[i] = 0;
                _state.RcpState[i] = (double)PumpState.Off;
                _state.RcpFlowFraction[i] = 0;
            }

            // boron at high concentration for shutdown
            _state.BoronConcentrationPPM = 1800.0;
            _state.BoronConcentrationTarget = 1800.0;
            _state.BoricAcidTankConcentration = 7000.0;

            // SG secondary depressurised
            _state.SgSecondaryPressureMPa = 0.1;
            _state.SgLevelFraction = 0.50;
            _state.SgLevelFractionWide = 0.50;
            _state.SgTubeFoulingFactor = 1.0;
            _state.SgPrimaryInletTempC = 30.0;
            _state.SgPrimaryOutletTempC = 30.0;
            _state.SgFeedwaterTempC = 30.0;
            _state.SgSteamTempC = 30.0;
            _state.MainSteamPressureMPa = 0.1;
            _state.FeedwaterTempC = 30.0;

            // turbine off-line
            _state.TurbineOnline = 0;
            _state.GeneratorBreakerClosed = 0;
            _state.GridTiedMode = 1;  // default to grid-tied
            _state.TurbineSpeedRPM = 0;
            _state.GridFrequencyHz = 0;
            _state.TurbineMechanicalPowerMW = 0;
            _state.TurbineInletEnthalpyKJPerKg = 0;
            _state.TurbineExhaustEnthalpyKJPerKg = 0;
            _state.TurbineSteamConsumptionKgPerS = 0;
            _state.CondenserHeatRejectionMW = 0;
            _state.TurbineUnderspeedAlarm = 0;
            _state.TurbineOverspeedWarn = 0;
            _state.TurbineBreakerAlarm = 0;

            // ECCS standby
            _state.EccsStatus = EccsMode.Standby;
            _state.AccumulatorPressureMPa = _design.AccumulatorDesignPressureMPa;
            _state.AccumulatorWaterVolumeM3 = _design.AccumulatorDesignVolumeM3;

            // containment
            _state.ContainmentPressureKPa = 101.3;
            _state.ContainmentTempC = 30.0;

            // fuel integrity at BOL
            _state.CladOxideThickness = 0;
            _state.DNBRatio = 10.0;
            _state.CladTemperatureLimit = 1204.0;   // 10 CFR 50.46

            // core damage — reset on cold shutdown
            _state.CoreDamageState           = CoreDamageState.Normal;
            _state.CoreDamageFraction        = 0;
            _state.ZirconiumOxidizedFraction = 0;
            _state.CoreExothermicHeatMW      = 0;
            _state.HydrogenGenerationKg      = 0;
            _state.FissionProductReleaseFraction = 0;
            _state.ContainmentH2FractionPct  = 0;
            _state.FuelBurnupMWdPerTonne = 0;
            _state.PeakingFactor = 2.50;             // typical PWR total peaking

            // fuel temps at cold
            _state.FuelCentrelineTemp = 30.0;
            _state.FuelAverageTemp = 30.0;
            _state.FuelSurfaceTemp = 30.0;
            _state.CladInnerTemp = 30.0;
            _state.CladOuterTemp = 30.0;
            _state.CoolantBulkTemp = 30.0;
            _state.GapConductance = _design.GapConductanceNominal;

            // trip setpoints (operator-adjustable)
            _state.OverpowerTripSetpoint = _design.TripHighFlux;
            _state.HighPressureTripMPa = _design.TripHighPressureMPa;
            _state.LowPressureTripMPa = _design.TripLowPressureMPa;
            _state.LowFlowTripFraction = _design.TripLowFlow;

            // electrical
            _state.OffSitePowerAvailable = 1;
            _state.HousekeepingLoadMW = 60.0;

            _state.TimeStepSeconds = 0.05;

            // pre-compute reactivity budget so RhoTotal is correct from tick 0
            ComputeReactivityBudget();

            // set neutron population and precursors to subcritical equilibrium
            // n_eq = S · Λ / (β − ρ)     where ρ is deeply negative
            double rhoNow = _state.RhoTotal;
            double promptDamp = (_state.BetaTotal - rhoNow) / _state.PromptNeutronLifetime;
            double srcRate = 1e-11;
            double nEq = srcRate / promptDamp;
            _state.NeutronPopulation = Math.Max(nEq, 1e-15);

            // precursors at equilibrium for subcritical source-level n
            for (int i = 0; i < Array6.Length; i++)
            {
                double CiEq = (_design.Beta_i[i] / _state.PromptNeutronLifetime)
                             * _state.NeutronPopulation / _design.Lambda_i[i];
                _state.PrecursorConcentrations[i] = CiEq;
            }

            _events.Clear();
            LogEvent("INIT", "Cold shutdown state initialised.");
        }

        // =====================================================================
        //  Hot-standby initialisation  (Tavg at program, critical, 0 power)
        // =====================================================================

        public void InitialiseHotZeroPower()
        {
            InitialiseColdShutdown();
            _state.Mode = ReactorMode.HotStandby;

            // primary at normal operating pressure and temperature
            _state.PrimaryPressureMPa = _design.DesignPrimaryPressureMPa;
            _state.CoreInletTempC = _design.DesignCoreInletTempC;
            _state.CoreOutletTempC = _design.DesignCoreInletTempC; // zero ΔT
            _state.TavgC = _design.DesignCoreInletTempC;
            _state.CoolantDensityKgPerM3 = _design.CoolantDensityNominal;

            _state.PressuriserPressureMPa = _design.DesignPrimaryPressureMPa;
            _state.PressuriserTempC = _design.SaturationTempAtDesignP;
            _state.PressuriserLevelFraction = 0.60;

            // RCPs running
            for (int i = 0; i < 4; i++)
            {
                _state.RcpSpeed[i] = 1.0;
                _state.RcpState[i] = (double)PumpState.Running;
                _state.RcpFlowFraction[i] = 1.0;
            }
            _state.PrimaryFlowRateKgPerS = _design.DesignPrimaryFlowKgPerS;
            _state.PrimaryFlowFraction = 1.0;
            _state.TotalRcpFlowFraction = 1.0;

            // boron lowered for criticality at HZP
            _state.BoronConcentrationPPM = 1200.0;
            _state.BoronConcentrationTarget = 1200.0;

            // rods: banks A-C fully out, D at ~150 steps
            _state.RodBankPosition[0] = 228;
            _state.RodBankPosition[1] = 228;
            _state.RodBankPosition[2] = 228;
            _state.RodBankPosition[3] = 150;
            for (int i = 0; i < 4; i++)
                _state.RodBankTargetPosition[i] = _state.RodBankPosition[i];

            // bring to just-critical
            _state.NeutronPopulation = 1e-8;
            _state.RhoTotal = 0.0;

            // SG secondary at operating pressure
            _state.SgSecondaryPressureMPa = _design.SgDesignSecondaryPressure;
            _state.SgSteamTempC = _design.SgDesignSteamTempC;
            _state.SgFeedwaterTempC = _design.SgDesignFeedTempC;
            _state.SgLevelFraction = 0.50;

            // fuel at cold-leg temp
            _state.FuelCentrelineTemp = _state.TavgC;
            _state.FuelAverageTemp = _state.TavgC;
            _state.FuelSurfaceTemp = _state.TavgC;
            _state.CladInnerTemp = _state.TavgC;
            _state.CladOuterTemp = _state.TavgC;
            _state.CoolantBulkTemp = _state.TavgC;

            // equilibrium precursors for zero power
            for (int i = 0; i < Array6.Length; i++)
                _state.PrecursorConcentrations[i] =
                    _design.Beta_i[i] / (_design.PromptNeutronLifetime * _design.Lambda_i[i])
                    * _state.NeutronPopulation;

            // SG primary-side temperatures
            _state.SgPrimaryInletTempC = _state.CoreOutletTempC;
            _state.SgPrimaryOutletTempC = _state.CoreInletTempC;

            // subcooling margin
            double Tsat = SaturationTemperature(_state.PrimaryPressureMPa);
            _state.SubcoolingMarginC = Tsat - _state.CoreOutletTempC;

            // self-calibrate reactivity (just-critical at HZP)
            ComputeReactivityBudget();
            _state.RhoManualBias = -_state.RhoTotal;
            _state.RhoTotal = 0;

            LogEvent("INIT", "Hot zero-power state initialised (just-critical).");
        }

        // =====================================================================
        //  Full-power equilibrium initialisation
        // =====================================================================

        public void InitialiseFullPower()
        {
            InitialiseHotZeroPower();
            _state.Mode = ReactorMode.SteadyState;

            // At rated, total thermal power = fission + decay.
            // NeutronPopulation & ThermalPowerFraction represent the fission
            // contribution only, so that fission + DecayHeatFraction = 1.0.
            // Self-consistent equilibrium: f + f·Σα = 1  ⇒  f = 1/(1+Σα)
            double equilibDecay = 0;
            for (int g = 0; g < _design.DecayAlpha.Length; g++)
                equilibDecay += _design.DecayAlpha[g];
            double fissionFrac = 1.0 / (1.0 + equilibDecay);

            _state.NeutronPopulation = fissionFrac;
            _state.ThermalPowerFraction = fissionFrac;
            _state.ThermalPowerMW = fissionFrac * _design.RatedThermalPowerMW;

            // temperatures at rated
            _state.CoreInletTempC = _design.DesignCoreInletTempC;
            _state.CoreOutletTempC = _design.DesignCoreOutletTempC;
            _state.TavgC = _design.DesignTavgC;
            _state.CoreDeltaT = _design.DesignCoreDeltaT;

            // fuel temps at rated (approximate)
            _state.CoolantBulkTemp = _state.TavgC;
            _state.CladOuterTemp = _state.CoolantBulkTemp + 20.0;
            _state.CladInnerTemp = _state.CladOuterTemp + 15.0;
            _state.FuelSurfaceTemp = _state.CladInnerTemp + 180.0;
            _state.FuelAverageTemp = _state.FuelSurfaceTemp + 400.0;
            _state.FuelCentrelineTemp = _state.FuelAverageTemp + 350.0;

            // equilibrium poisons at actual fission power fraction
            double sfphi = _design.SigmaFPhiRated * fissionFrac;
            _state.Iodine135 = _design.GammaIodine * sfphi / _design.LambdaIodine;
            _state.Xenon135 = (_design.GammaXenon * sfphi + _design.LambdaIodine * _state.Iodine135)
                              / (_design.LambdaXenon + _design.SigmaXenon * sfphi);
            _state.XenonEquilibrium = _state.Xenon135;
            _state.Promethium149 = _design.GammaPromethium * sfphi / _design.LambdaPromethium;
            _state.Samarium149 = _design.GammaPromethium * sfphi / (_design.SigmaSamarium * sfphi);
            _state.SamariumEquilibrium = _state.Samarium149;

            // equilibrium precursors
            for (int i = 0; i < Array6.Length; i++)
                _state.PrecursorConcentrations[i] =
                    _design.Beta_i[i] / (_design.PromptNeutronLifetime * _design.Lambda_i[i])
                    * _state.NeutronPopulation;

            // boron lower for power defect
            _state.BoronConcentrationPPM = 800.0;
            _state.BoronConcentrationTarget = 800.0;

            // rod D at ~200 steps
            _state.RodBankPosition[3] = 200;
            _state.RodBankTargetPosition[3] = 200;

            // turbine on-line
            _state.TurbineOnline = 1;
            _state.GeneratorBreakerClosed = 1;
            _state.GridTiedMode = 1;  // default to grid-tied
            _state.TurbineUnderspeedAlarm = 0;
            _state.TurbineOverspeedWarn = 0;
            _state.TurbineBreakerAlarm = 0;
            _state.TurbineSpeedRPM = _design.TurbineRatedSpeedRPM;
            _state.TurbineSpeedFraction = 1.0;
            _state.TurbineThrottlePosition = 1.0;
            _state.TurbinePowerTarget = _design.RatedElectricalMW;
            _state.GridFrequencyHz = 60.0;

            // Steam thermodynamics at rated conditions (7 MPa saturated steam → condenser)
            // hInFP, dhFP now come from the corrected ln(P) polynomial fits.
            double hInFP = SaturatedSteamEnthalpyKJ(7.0);   // ≈ 2773 kJ/kg
            double sInFP = SaturatedSteamEntropyKJ(7.0);    // ≈ 5.77 kJ/kg·K
            double dhFP  = TurbineIsentropicDrop(hInFP, sInFP, _design.CondenserDesignTempC);
            // dhFP ≈ 697 kJ/kg ideal; actual = dhFP × ηIs (already applied inside helper)
            // Net: ~593 kJ/kg actual enthalpy drop at full steam admission

            // SG steam flow: SG model uses hfg=1500 kJ/kg (latent heat at 7 MPa — correct)
            double ratedSteamFlowKgPerS = _design.RatedThermalPowerMW * 1e6 / 1500e3; // ≈ 2274 kg/s

            // Throttle at rated: the machine produces MORE power than rated at 100% throttle
            // (~1347 MW mech / ~1327 MWe); governor sits at ~87% throttle for 1150 MWe.
            double maxMechMW    = ratedSteamFlowKgPerS * dhFP / 1000.0; // at 100% throttle
            double ratedMechMW  = _design.RatedElectricalMW / _design.GeneratorEfficiency;
            double ratedThrottle = Math.Clamp(ratedMechMW / (maxMechMW + 1e-6), 0, 1);

            _state.TurbineThrottlePosition = ratedThrottle;
            _state.TurbineInletEnthalpyKJPerKg  = hInFP;
            _state.TurbineExhaustEnthalpyKJPerKg = hInFP - dhFP;
            _state.TurbineSteamConsumptionKgPerS = ratedSteamFlowKgPerS * ratedThrottle;
            _state.TurbineMechanicalPowerMW = ratedMechMW;
            _state.TurbineLoadMW            = ratedMechMW;
            _state.GeneratorOutputMW        = _design.RatedElectricalMW;
            _state.NetElectricalMW          = _design.RatedElectricalMW - _state.HousekeepingLoadMW;

            // Condenser at design
            _state.CondenserTempC     = _design.CondenserDesignTempC;
            _state.CondenserVacuumKPa = _design.CondenserDesignVacuumKPa;
            double hf = 4.186 * _design.CondenserDesignTempC;
            _state.CondenserHeatRejectionMW = Math.Max(0,
                _state.TurbineSteamConsumptionKgPerS * (_state.TurbineExhaustEnthalpyKJPerKg - hf) / 1000.0);

            // SG at rated
            _state.SgSteamFlowKgPerS     = ratedSteamFlowKgPerS;
            _state.SgFeedwaterFlowKgPerS  = ratedSteamFlowKgPerS;
            _state.MainFeedPumpSpeedFraction = 1.0;
            _state.MainFeedPumpFlowKgPerS = ratedSteamFlowKgPerS;
            _state.FeedwaterTempC         = _design.SgDesignFeedTempC;

            // DNB
            _state.DNBRatio = 2.10;

            // decay-heat groups at equilibrium for the current fission power
            for (int g = 0; g < Array23.Length; g++)
                _state.DecayHeatGroupEnergy[g] = _design.DecayAlpha[g] * fissionFrac / _design.DecayLambda[g];

            // compute equilibrium decay-heat fraction so first tick isn't a discontinuity
            double dhSum = 0;
            for (int g = 0; g < Array23.Length; g++)
                dhSum += _design.DecayLambda[g] * _state.DecayHeatGroupEnergy[g];
            _state.DecayHeatFraction = dhSum;
            _state.DecayHeatMW = dhSum * _design.RatedThermalPowerMW;
            _state.TotalPowerMW = _state.ThermalPowerMW + _state.DecayHeatMW;

            // SG primary-side temperatures (must be initialised before first tick)
            _state.SgPrimaryInletTempC = _state.CoreOutletTempC;
            _state.SgPrimaryOutletTempC = _state.CoreInletTempC;

            // flow fraction
            _state.PrimaryFlowFraction = 1.0;

            // subcooling margin & equilibrium void fraction
            double Tsat = SaturationTemperature(_state.PrimaryPressureMPa);
            _state.SubcoolingMarginC = Tsat - _state.CoreOutletTempC;
            if (_state.SubcoolingMarginC < 0)
                _state.CoolantVoidFraction = Math.Min(
                    (-_state.SubcoolingMarginC) / 50.0, 0.95);
            else
                _state.CoolantVoidFraction = 0;

            // coolant density at operating temperature
            _state.CoolantDensityKgPerM3 = Math.Max(100.0,
                1013.0 - 0.95 * _state.TavgC);

            // linear heat rate at rated (for consistent DNBR on first tick)
            double totalRods = _design.FuelAssemblies * _design.RodsPerAssembly;
            double avgLHR = _design.RatedThermalPowerMW * 1e6 / (totalRods * _design.ActiveFuelLengthM);
            _state.LinearHeatRate = avgLHR * _state.PeakingFactor;

            // self-calibrate: compute reactivity budget and null it out with bias
            // so the reactor starts exactly critical at the reference state.
            // Reset bias first so it doesn't carry over from HZP init.
            _state.RhoManualBias = 0;
            ComputeReactivityBudget();
            _state.RhoManualBias = -_state.RhoTotal;
            _state.RhoTotal = 0;

            // Settle fuel rod thermal model to true equilibrium.  The
            // approximate temperature profile above may not balance with the
            // fuel thermal-hydraulic model, causing transient drift on the
            // first real tick.  Iterate fuel thermal only, with primary coolant
            // temps held at design values, until fuel temps converge.
            for (int i = 0; i < 50; i++)
                UpdateFuelRodThermal(1.0);

            // recalibrate after thermal settling
            _state.RhoManualBias = 0;
            ComputeReactivityBudget();
            _state.RhoManualBias = -_state.RhoTotal;
            _state.RhoTotal = 0;

            // initialise previous power so rate-of-change is zero on first tick
            _state.PreviousTotalPowerFraction = _state.ThermalPowerFraction + _state.DecayHeatFraction;

            LogEvent("INIT", "Full-power equilibrium state initialised.");
        }

        // =====================================================================
        //  PRIMARY SIMULATION TICK
        // =====================================================================

        /// <summary>
        /// Advance the simulation by dt seconds.
        /// For stability, dt should be ≤ 0.10 s; the engine will subdivide
        /// automatically if dt > internalMaxStep.
        /// </summary>
        public void Tick(double dt)
        {
            if (dt <= 0) return;
            const double maxSubStep = 0.05;
            int steps = (int)Math.Ceiling(dt / maxSubStep);
            double subDt = dt / steps;

            for (int s = 0; s < steps; s++)
            {
                _state.PreviousTotalPowerFraction = _state.ThermalPowerFraction + _state.DecayHeatFraction;
                _state.TimeStepSeconds = subDt;

                UpdateControlRods(subDt);
                UpdateBoronConcentration(subDt);
                ComputeReactivityBudget();
                UpdateNeutronKinetics(subDt);
                UpdateFissionProductPoisons(subDt);
                UpdateDecayHeat(subDt);
                UpdateFuelRodThermal(subDt);
                UpdatePrimaryCoolant(subDt);
                UpdatePressuriser(subDt);
                UpdateSteamGenerator(subDt);
                UpdateSecondaryAndTurbine(subDt);
                UpdateRcpModel(subDt);
                UpdateEccs(subDt);
                UpdateContainment(subDt);
                UpdateElectrical(subDt);
                UpdateFuelIntegrity(subDt);
                UpdateCoreDestruction(subDt);
                UpdateBurnup(subDt);

                _state.SimulationTimeSeconds += subDt;
                _state.TickCount++;
            }
        }

        // =====================================================================
        //  NEUTRON KINETICS  (point kinetics equations, 6 delayed groups)
        // =====================================================================

        private void UpdateNeutronKinetics(double dt)
        {
            double n = _state.NeutronPopulation;
            double rho = _state.RhoTotal;
            double beta = _state.BetaTotal;
            double lambda_gen = _state.PromptNeutronLifetime;

            // source term (keeps neutron population alive at deep subcritical)
            double source = 1e-11;

            // precursor source sum
            double precursorSum = 0;
            for (int i = 0; i < Array6.Length; i++)
                precursorSum += _design.Lambda_i[i] * _state.PrecursorConcentrations[i];

            // ---- DEEPLY SUBCRITICAL (|ρ| > 1000 pcm) ----
            // When the reactor is far from critical, the prompt neutron
            // population is in quasi-static equilibrium with the source.
            // Using differential equations here is both unnecessary and
            // numerically dangerous (the prompt decay constant can be
            // ~10,000 s⁻¹, requiring sub-microsecond steps for explicit
            // methods).  Instead, compute the equilibrium directly:
            //   n_eq = (S + Σλᵢ·Cᵢ) · Λ / (β − ρ)
            // Then relax precursors toward their equilibrium values.

            if (rho < -0.01)   // more than -1000 pcm subcritical
            {
                double promptDamp = (beta - rho) / lambda_gen;   // s⁻¹
                n = (source + precursorSum) / promptDamp;
                n = Math.Max(n, 0);

                // relax precursors toward subcritical equilibrium
                for (int i = 0; i < Array6.Length; i++)
                {
                    double Ci = _state.PrecursorConcentrations[i];
                    double CiEq = (_design.Beta_i[i] / lambda_gen) * n / _design.Lambda_i[i];
                    double tauC = 1.0 / _design.Lambda_i[i]; // precursor time constant
                    double alag = dt / (tauC + dt);
                    _state.PrecursorConcentrations[i] = Math.Max(0,
                        Ci + (CiEq - Ci) * alag);
                }
            }
            // ---- NEAR-CRITICAL AND SUPERCRITICAL ----
            // Use implicit (backward) Euler for the prompt term:
            //   n_new = [n + dt·(Σλᵢ·Cᵢ + S)] / [1 − dt·(ρ−β)/Λ]
            else
            {
                double alpha = (rho - beta) / lambda_gen;
                double rhs = n + dt * (precursorSum + source);
                double denom = 1.0 - dt * alpha;

                if (denom > 0.01)
                    n = rhs / denom;
                else
                    n = n + dt * (alpha * n + precursorSum + source); // explicit fallback

                // update precursors with explicit Euler
                for (int i = 0; i < Array6.Length; i++)
                {
                    double Ci = _state.PrecursorConcentrations[i];
                    double dCdt = (_design.Beta_i[i] / lambda_gen) * n
                                  - _design.Lambda_i[i] * Ci;
                    _state.PrecursorConcentrations[i] = Math.Max(0, Ci + dCdt * dt);
                }
            }

            // clamp to physical range
            n = Math.Clamp(n, 0, 5.0);  // >500% would be unphysical prompt-crit

            _state.NeutronPopulation = n;

            // thermal power tracks neutron population with short time-constant
            // (accounts for heat-capacity lag of UO₂)
            double tau_fuel = 6.0; // seconds (UO₂ thermal time constant)
            double dPdt = (n - _state.ThermalPowerFraction) / tau_fuel;
            _state.ThermalPowerFraction = Math.Max(0,
                _state.ThermalPowerFraction + dPdt * dt);

            _state.ThermalPowerMW = _state.ThermalPowerFraction * _design.RatedThermalPowerMW;
            _state.TotalPowerMW = _state.ThermalPowerMW + _state.DecayHeatMW;

            // startup rate  (SUR = 26.06 / reactor period)
            double totalPowerNow = _state.ThermalPowerFraction + _state.DecayHeatFraction;
            double powerRate = (totalPowerNow - _state.PreviousTotalPowerFraction) / dt;
            if (totalPowerNow > 1e-9)
            {
                double period = totalPowerNow / (powerRate + 1e-30);
                _state.StartupRateDPM = 26.06 / (period + 1e-30);
            }
            else
            {
                _state.StartupRateDPM = 0;
            }
        }

        // =====================================================================
        //  REACTIVITY BUDGET
        // =====================================================================

        private void ComputeReactivityBudget()
        {
            // ---- control rods ----
            double rodRho = 0;
            for (int b = 0; b < 4; b++)
            {
                double frac = _state.RodBankPosition[b] / (double)_design.MaxRodSteps;
                // differential worth follows a roughly cosine-squared shape
                double integralWorth = IntegralRodWorth(frac);
                rodRho -= _design.TotalRodWorthPCM * 1e-5
                          * _design.BankWorthFraction[b]
                          * (1.0 - integralWorth);
            }
            _state.RhoControlRods = rodRho;

            // ---- Doppler  (fuel temperature feedback) ----
            // deviation from design full-power fuel temperature
            double dTfuel = _state.FuelAverageTemp - _design.DesignFuelAverageTempC;
            _state.RhoDoppler = _design.DopplerCoefficient * dTfuel;

            // ---- moderator temperature feedback ----
            // In a real PWR, MTC is near zero (slightly positive) at cold
            // conditions with high boron, and becomes more negative as
            // temperature increases toward operating conditions.
            // Model: MTC scales linearly from ~0 at 50°C to full design value
            // at DesignTavg.  This prevents the unrealistic +11,000 pcm
            // contribution that a constant MTC would give at cold conditions.
            double tavg = _state.TavgC;
            double mtcRef = _design.ModeratorTempCoeff
                          * (1.0 + 0.0001 * _state.BoronConcentrationPPM);
            double mtcScalar = Math.Clamp((tavg - 50.0)
                             / (_state.TrefC - 50.0), 0.0, 1.0);
            double mtcEffective = mtcRef * mtcScalar;
            double dTmod = tavg - _state.TrefC;
            _state.RhoModerator = mtcEffective * dTmod;

            // ---- boron  (deviation from design equilibrium concentration) ----
            _state.RhoBoron = _design.BoronWorth
                            * (_state.BoronConcentrationPPM - _design.DesignBoronPPM);

            // ---- xenon ----
            // Deviation from equilibrium.  Guard: if no Xe is present (fresh
            // startup, Xe135≈0) there is no reactivity effect — prevents a
            // spurious +3000 pcm injection when XeEq first becomes non-zero
            // while Xe135 hasn't had time to build up yet.
            if (_state.XenonEquilibrium > 0 && _state.Xenon135 > 0)
                // Clamp to ≤ 0: the HZP calibration bias already accounts for
                // zero xenon, so xenon can only ADD poison (negative reactivity)
                // as it builds up from zero.  The "xenon-free bonus" reactivity
                // (Xe < XeEq) is not credited here — operators compensate with
                // rods/boron when running a real startup to equilibrium.
                _state.RhoXenon = Math.Min(0.0, -0.03 * (_state.Xenon135 / (_state.XenonEquilibrium + 1e-30) - 1.0));
            else
                _state.RhoXenon = 0;

            // ---- samarium ----
            // Same guard and clamp as xenon.
            if (_state.SamariumEquilibrium > 0 && _state.Samarium149 > 0)
                _state.RhoSamarium = Math.Min(0.0, -0.007 * (_state.Samarium149 / (_state.SamariumEquilibrium + 1e-30) - 1.0));
            else
                _state.RhoSamarium = 0;

            // ---- void ----
            _state.RhoVoid = _design.VoidCoefficient * _state.CoolantVoidFraction * 100.0;

            // ---- total ----
            _state.RhoTotal = _state.RhoControlRods
                            + _state.RhoDoppler
                            + _state.RhoModerator
                            + _state.RhoBoron
                            + _state.RhoXenon
                            + _state.RhoSamarium
                            + _state.RhoVoid
                            + _state.RhoManualBias;

        }

        /// <summary>Normalised integral rod worth  (0 = fully in, 1 = fully out).</summary>
        private static double IntegralRodWorth(double fractionWithdrawn)
        {
            double x = Math.Clamp(fractionWithdrawn, 0, 1);
            // cosine-squared integral:  W(x) = x - sin(2πx)/(2π)
            return x - Math.Sin(2.0 * Math.PI * x) / (2.0 * Math.PI);
        }

        // =====================================================================
        //  FISSION-PRODUCT POISONS
        // =====================================================================

        private void UpdateFissionProductPoisons(double dt)
        {
            double phi = _design.SigmaFPhiRated * _state.ThermalPowerFraction;

            // ---- Iodine-135 ----
            // dI/dt = γI Σf φ  -  λI I
            double dIdt = _design.GammaIodine * phi
                        - _design.LambdaIodine * _state.Iodine135;
            _state.Iodine135 = Math.Max(0, _state.Iodine135 + dIdt * dt);

            // ---- Xenon-135 ----
            // dXe/dt = γXe Σf φ  +  λI I  -  λXe Xe  -  σXe φ Xe
            double dXedt = _design.GammaXenon * phi
                         + _design.LambdaIodine * _state.Iodine135
                         - _design.LambdaXenon * _state.Xenon135
                         - _design.SigmaXenon * phi * _state.Xenon135;
            _state.Xenon135 = Math.Max(0, _state.Xenon135 + dXedt * dt);

            // track equilibrium at current power
            // XeEq rises instantly with power (tracks the new target immediately)
            // but only falls at the physical Xe-135 decay rate — this keeps the
            // Xe/eq ratio bounded after a trip and prevents post-scram blow-up.
            if (phi > 1e5)
            {
                double Ieq    = _design.GammaIodine * phi / _design.LambdaIodine;
                double xeEqNow = (_design.GammaXenon * phi + _design.LambdaIodine * Ieq)
                                 / (_design.LambdaXenon + _design.SigmaXenon * phi);
                if (xeEqNow >= _state.XenonEquilibrium)
                    _state.XenonEquilibrium = xeEqNow;
                else
                    _state.XenonEquilibrium = Math.Max(xeEqNow,
                        _state.XenonEquilibrium * Math.Exp(-_design.LambdaXenon * dt));
            }
            else
            {
                // Below threshold: let XeEq decay at Xe-135 rate so it doesn't
                // stay frozen at full-power value after an extended shutdown.
                _state.XenonEquilibrium *= Math.Exp(-_design.LambdaXenon * dt);
            }

            // ---- Promethium-149 ----
            double dPmdt = _design.GammaPromethium * phi
                         - _design.LambdaPromethium * _state.Promethium149;
            _state.Promethium149 = Math.Max(0, _state.Promethium149 + dPmdt * dt);

            // ---- Samarium-149  (stable: destroyed only by neutron capture) ----
            double dSmdt = _design.LambdaPromethium * _state.Promethium149
                         - _design.SigmaSamarium * phi * _state.Samarium149;
            _state.Samarium149 = Math.Max(0, _state.Samarium149 + dSmdt * dt);

            if (phi > 1e5)
                _state.SamariumEquilibrium = _design.GammaPromethium * phi
                                            / (_design.SigmaSamarium * phi);
        }

        // =====================================================================
        //  DECAY HEAT  (ANS-5.1-2005 23-group model)
        // =====================================================================

        private void UpdateDecayHeat(double dt)
        {
            double fissionPower = _state.ThermalPowerFraction;
            double totalDecay = 0;

            for (int g = 0; g < Array23.Length; g++)
            {
                double alpha = _design.DecayAlpha[g];
                double lambda = _design.DecayLambda[g];
                double E = _state.DecayHeatGroupEnergy[g];

                // dE/dt = α·P(t) - λ·E
                double dEdt = alpha * fissionPower - lambda * E;
                E = Math.Max(0, E + dEdt * dt);
                _state.DecayHeatGroupEnergy[g] = E;

                totalDecay += lambda * E;
            }

            _state.DecayHeatFraction = totalDecay;
            _state.DecayHeatMW = totalDecay * _design.RatedThermalPowerMW;
        }

        // =====================================================================
        //  FUEL ROD THERMAL MODEL  (radial steady-state conduction each step)
        // =====================================================================

        private void UpdateFuelRodThermal(double dt)
        {
            double power = _state.ThermalPowerFraction + _state.DecayHeatFraction;
            if (power < 0) power = 0;

            // Zr-steam exothermic heat adds to peak-rod heat generation
            double exoFrac = _state.CoreExothermicHeatMW / (_design.RatedThermalPowerMW + 1e-6);
            power += exoFrac;

            // linear heat rate  q' (W/m) for peak rod
            double totalRods = _design.FuelAssemblies * _design.RodsPerAssembly;
            double avgLinearHeatRate = power * _design.RatedThermalPowerMW * 1e6
                                      / (totalRods * _design.ActiveFuelLengthM);
            _state.LinearHeatRate = avgLinearHeatRate * _state.PeakingFactor;
            double qPrime = _state.LinearHeatRate;

            double rFuel = _design.FuelPelletRadiusM;
            double rCladI = _design.CladInnerRadiusM;
            double rCladO = _design.CladOuterRadiusM;

            double kFuel = _design.FuelThermalConductivity;
            double hGap = _state.GapConductance;
            double kClad = _design.CladConductivity;
            double hCool = _design.CoolantHTCNominal * _state.PrimaryFlowFraction;
            hCool = Math.Max(hCool, 500); // natural convection lower bound

            // coolant bulk temperature (core average)
            _state.CoolantBulkTemp = _state.TavgC;

            // clad outer surface:  q' = h_cool · 2π·r_co · (T_co - T_bulk)
            double TCladOut = _state.CoolantBulkTemp
                            + qPrime / (2.0 * Math.PI * rCladO * hCool);

            // clad inner surface:  conduction through clad
            double TCladIn = TCladOut
                           + qPrime * Math.Log(rCladO / rCladI) / (2.0 * Math.PI * kClad);

            // fuel surface:  gap
            double TFuelSurf = TCladIn
                             + qPrime / (2.0 * Math.PI * rFuel * hGap);

            // fuel centreline:  volumetric generation in cylinder
            double TFuelCentre = TFuelSurf + qPrime / (4.0 * Math.PI * kFuel);

            double TFuelAvg = (TFuelCentre + TFuelSurf) / 2.0;

            // apply with thermal inertia (simple first-order lag)
            double tauFuel = 6.0; // s
            double tauClad = 1.0; // s
            double a_fuel = dt / (tauFuel + dt);
            double a_clad = dt / (tauClad + dt);

            _state.FuelCentrelineTemp += (TFuelCentre - _state.FuelCentrelineTemp) * a_fuel;
            _state.FuelAverageTemp += (TFuelAvg - _state.FuelAverageTemp) * a_fuel;
            _state.FuelSurfaceTemp += (TFuelSurf - _state.FuelSurfaceTemp) * a_fuel;
            _state.CladInnerTemp += (TCladIn - _state.CladInnerTemp) * a_clad;
            _state.CladOuterTemp += (TCladOut - _state.CladOuterTemp) * a_clad;

            // gap conductance degrades with burnup (empirical)
            _state.GapConductance = _design.GapConductanceNominal
                                  * (1.0 - 0.15 * Math.Min(_state.FuelBurnupMWdPerTonne / 60000.0, 1.0));
        }

        // =====================================================================
        //  PRIMARY COOLANT
        // =====================================================================

        private void UpdatePrimaryCoolant(double dt)
        {
            double power = (_state.ThermalPowerFraction + _state.DecayHeatFraction)
                           * _design.RatedThermalPowerMW;  // MW into coolant

            // additional heat sources: pressuriser heaters + RCP pump shaft heat + Zr-steam rxn
            power += _state.PressuriserHeaterPowerKW * 1e-3;          // kW → MW
            power += _state.CoreExothermicHeatMW;
            for (int i = 0; i < 4; i++)
                power += _state.RcpSpeed[i] * _design.RcpRatedPowerMW;

            double flow = _state.PrimaryFlowRateKgPerS;
            double cp = _design.CoolantSpecificHeatJPerKgC;

            // core ΔT = Q / (ṁ · cp)
            double deltaT = 0;
            if (flow > 1.0)
                deltaT = (power * 1e6) / (flow * cp);

            // cold-leg temp tracks SG primary outlet
            double Tcold = _state.SgPrimaryOutletTempC;
            if (Tcold < 30) Tcold = _state.CoreInletTempC; // fallback

            double Thot = Tcold + deltaT;

            // apply with transport lag
            double tauLoop = 12.0; // s, loop transit time
            double a = dt / (tauLoop + dt);
            _state.CoreInletTempC += (Tcold - _state.CoreInletTempC) * a;
            _state.CoreOutletTempC += (Thot - _state.CoreOutletTempC) * a;
            _state.CoreDeltaT = _state.CoreOutletTempC - _state.CoreInletTempC;
            _state.TavgC = (_state.CoreInletTempC + _state.CoreOutletTempC) / 2.0;

            // coolant density (simplified linear fit for subcooled water at ~15.5 MPa)
            // ρ ≈ 1013 - 0.95·T(°C)    valid ~ 250-340 °C
            _state.CoolantDensityKgPerM3 = 1013.0 - 0.95 * _state.TavgC;
            _state.CoolantDensityKgPerM3 = Math.Max(_state.CoolantDensityKgPerM3, 100);

            // void fraction (if Tout approaches Tsat)
            double Tsat = SaturationTemperature(_state.PrimaryPressureMPa);
            _state.SubcoolingMarginC = Tsat - _state.CoreOutletTempC;
            if (_state.SubcoolingMarginC < 0)
            {
                // boiling in core
                _state.CoolantVoidFraction = Math.Min(
                    (-_state.SubcoolingMarginC) / 50.0, 0.95);
            }
            else
            {
                _state.CoolantVoidFraction = Math.Max(0,
                    _state.CoolantVoidFraction - 0.05 * dt);
            }

            // primary pressure follows pressuriser (they're connected)
            _state.PrimaryPressureMPa = _state.PressuriserPressureMPa;
            _state.PrimaryPressurePsi = _state.PrimaryPressureMPa * 145.038;

            // flow fraction
            _state.PrimaryFlowFraction = _state.PrimaryFlowRateKgPerS
                                         / _design.DesignPrimaryFlowKgPerS;
        }

        // =====================================================================
        //  PRESSURISER
        // =====================================================================

        private void UpdatePressuriser(double dt)
        {
            double P = _state.PressuriserPressureMPa;

            // Pressuriser heaters and spray only operate when the RCS is
            // pressurised with at least one RCP running.
            bool pressuriserActive = _state.TotalRcpFlowFraction > 0.05;

            // Heater power and spray flow are set by the controller (or
            // directly by operator commands).  Physics just reads them.

            // ---- PORV ----
            if (P > _design.PorvSetpointMPa)
                _state.PorvOpen = 1;
            else if (P < _design.PorvResetMPa)
                _state.PorvOpen = 0;

            // ---- code safety valve ----
            _state.SafetyValveOpen = (P > _design.SafetyValveSetpointMPa) ? (byte)1 : (byte)0;

            // ---- pressure dynamics (simplified steam-table model) ----
            // heating increases P, spray & relief decrease it
            double dPdt = 0;

            // Steam-bubble factor: heater-to-pressure coupling only works when there is
            // a compressible steam space in the pressurizer.  Below ~0.5 MPa the pressurizer
            // is all liquid; heaters warm the water but do not directly raise steam pressure.
            // The factor rises smoothly from 0 at 0.5 MPa to 1.0 at 2.5 MPa.
            double steamBubbleFactor = Math.Clamp((P - 0.5) / 2.0, 0.0, 1.0);

            // In steam-bubble mode: efficient pressure rise (small steam space, sensitive to heat).
            // In all-liquid/cold mode: slow thermal expansion only.
            double heaterHot  = _state.PressuriserHeaterPowerKW * 1e-3 * 0.08  * steamBubbleFactor;
            double heaterCold = _state.PressuriserHeaterPowerKW * 1e-3 * 0.003 * (1.0 - steamBubbleFactor);
            dPdt += heaterHot + heaterCold;

            // spray quenching steam → pressure drop
            double sprayEffect = -_state.PressuriserSprayFlowKgPerS * 0.004; // MPa/s per kg/s rough
            dPdt += sprayEffect;

            // PORV relief
            if (_state.PorvOpen == 1)
                dPdt -= 0.5; // MPa/s  (rapid blowdown)

            // safety valve
            if (_state.SafetyValveOpen == 1)
                dPdt -= 2.0;

            // Surge-line coupling: primary thermal expansion/contraction pushes water into
            // or out of the pressurizer, changing pressure.  Scaled by steam-bubble factor
            // because an all-liquid (cold) pressurizer has no compressible steam space to
            // amplify the surge effect; the coupling is negligible until a bubble forms.
            double Tavg = _state.TavgC;
            if (pressuriserActive)
            {
                double surgeTerm = (Tavg - _design.DesignTavgC) * 0.002;
                dPdt += surgeTerm * 0.1 * steamBubbleFactor;
            }

            // ECCS injection can raise level / pressure
            if (_state.HpiFlowKgPerS > 0)
                dPdt += _state.HpiFlowKgPerS * 0.001;

            P += dPdt * dt;
            P = Math.Clamp(P, 0.10, 25.0);  // 0.10 MPa = atmospheric floor
            _state.PressuriserPressureMPa = P;

            // level  (simplified: level responds to average coolant temperature)
            double levelTarget = 0.60 + (Tavg - _design.DesignTavgC) * 0.005;
            levelTarget = Math.Clamp(levelTarget, 0.05, 0.95);
            double tauLevel = 30.0;
            _state.PressuriserLevelFraction += (levelTarget - _state.PressuriserLevelFraction)
                                               * dt / tauLevel;
            _state.PressuriserLevelFraction = Math.Clamp(_state.PressuriserLevelFraction, 0, 1);

            // pressuriser temperature tracks saturation
            // pressuriser temperature: when a steam bubble exists (P above ~1 MPa
            // and heaters are available), the steam space is at Tsat.  When the
            // system is depressurised / cold, the pressuriser is water at bulk
            // coolant temperature.
            if (P > 1.0)
                _state.PressuriserTempC = SaturationTemperature(P);
            else
                _state.PressuriserTempC = Tavg;
        }

        // =====================================================================
        //  STEAM GENERATOR
        // =====================================================================

        private void UpdateSteamGenerator(double dt)
        {
            // primary side
            _state.SgPrimaryInletTempC = _state.CoreOutletTempC;

            // heat transfer:  Q = UA · LMTD
            double UA = _design.SgDesignUAMwPerC * _state.SgTubeFoulingFactor;
            UA *= Math.Sqrt(Math.Max(_state.PrimaryFlowFraction, 0.01)); // flow dependence

            double Thi = _state.SgPrimaryInletTempC;
            double Tci = _state.SgFeedwaterTempC;
            double Tsat = SaturationTemperature(_state.SgSecondaryPressureMPa);

            // LMTD across SG (counter-flow approximation)
            double dT1 = Thi - Tsat;   // hot end
            // Cold-end effective temperature: blend from Tsat (no FW flow) to design FW
            // temperature (full FW flow).  At zero feedwater flow, the secondary cold end
            // is at saturation — not at the subcooled design feedwater temperature.
            double designFwFlow = _design.RatedThermalPowerMW * 1e6 / (1500e3);
            double fwFrac = Math.Clamp(_state.SgFeedwaterFlowKgPerS / designFwFlow, 0.0, 1.0);
            double TciEff = Tsat + (Tci - Tsat) * fwFrac;
            double dT2 = _state.CoreInletTempC - TciEff;  // cold end
            if (dT2 < 1) dT2 = 1;

            // No steam generation when primary hot leg is at or below secondary
            // saturation: heat would flow in reverse (condensation into primary).
            // Clamp Q to zero in this regime; the secondary pressure stops rising
            // and the primary temperature stabilises naturally at Tsat.
            double Qsg;
            if (dT1 <= 0)
            {
                Qsg = 0;
            }
            else
            {
                if (dT1 < 1) dT1 = 1;
                double lmtd;
                if (Math.Abs(dT1 - dT2) < 0.01)
                    lmtd = (dT1 + dT2) / 2.0;
                else
                    lmtd = (dT1 - dT2) / Math.Log(dT1 / dT2);
                Qsg = Math.Max(0, UA * lmtd);
            }

            // primary outlet
            double Tcold = Thi;
            if (_state.PrimaryFlowRateKgPerS > 1)
                Tcold = Thi - Qsg * 1e6 / (_state.PrimaryFlowRateKgPerS
                        * _design.CoolantSpecificHeatJPerKgC);
            _state.SgPrimaryOutletTempC = Tcold;

            // secondary side steam generation
            double hfg = 1500e3; // J/kg  enthalpy of vaporisation at ~7 MPa (approx)
            double steamGenRate = Qsg * 1e6 / hfg; // kg/s of steam

            _state.SgSteamFlowKgPerS = steamGenRate;

            // secondary pressure dynamics
            // pressure rises if steam production > turbine demand
            double steamDemand = 0;
            if (_state.TurbineOnline == 1)
                steamDemand = _state.TurbineThrottlePosition * _state.SgSteamFlowKgPerS;

            double dpSec = (steamGenRate - steamDemand) / (_design.SgWaterMassKg * 0.1);
            _state.SgSecondaryPressureMPa += dpSec * dt;
            _state.SgSecondaryPressureMPa = Math.Clamp(_state.SgSecondaryPressureMPa, 0.05, 10.0);

            _state.SgSteamTempC = SaturationTemperature(_state.SgSecondaryPressureMPa);
            _state.MainSteamPressureMPa = _state.SgSecondaryPressureMPa;
            _state.MainSteamTempC = _state.SgSteamTempC;

            // SG level  (mass balance: feed in - steam out)
            double levelRate = (_state.SgFeedwaterFlowKgPerS - steamGenRate) / _design.SgWaterMassKg;
            // swell / shrink: power increase causes apparent level rise (void swell)
            double swellEffect = ((_state.ThermalPowerFraction + _state.DecayHeatFraction) - _state.PreviousTotalPowerFraction) * 0.02;
            _state.SgLevelFraction += (levelRate + swellEffect) * dt;
            _state.SgLevelFraction = Math.Clamp(_state.SgLevelFraction, 0, 1);
            _state.SgLevelFractionWide = _state.SgLevelFraction;

            _state.SgHeatTransferCoefficient = UA;
        }

        // =====================================================================
        //  SECONDARY SIDE & TURBINE-GENERATOR
        // =====================================================================

        private void UpdateSecondaryAndTurbine(double dt)
        {
            // ----------------------------------------------------------------
            //  Condenser: temperature drifts toward design point when turbine
            //  is rejecting heat; approaches ambient when idle.
            // ----------------------------------------------------------------
            double tCondTarget = (_state.TurbineOnline == 1 && _state.TurbineTripFlag == 0)
                ? _design.CondenserDesignTempC : 25.0;
            _state.CondenserTempC += (tCondTarget - _state.CondenserTempC) * dt / 60.0;

            // Condenser saturation pressure from temperature (Antoine)
            double tK = _state.CondenserTempC + 273.15;
            double log10PCond = 8.14019 - 1810.94 / (tK - 28.665);
            _state.CondenserVacuumKPa = Math.Clamp(
                Math.Pow(10, log10PCond) / 7500.6 * 1000.0, 1.0, 30.0);

            // ----------------------------------------------------------------
            //  Turbine offline / coasting down
            // ----------------------------------------------------------------
            if (_state.TurbineOnline == 0 || _state.TurbineTripFlag == 1)
            {
                // Rotor coast-down: exponential decay (τ ≈ 30 s)
                _state.TurbineSpeedRPM *= Math.Exp(-dt / 30.0);
                _state.TurbineSpeedFraction = _state.TurbineSpeedRPM / _design.TurbineRatedSpeedRPM;
                _state.GridFrequencyHz = 0;
                _state.TurbineMechanicalPowerMW = 0;
                _state.TurbineLoadMW = 0;
                _state.GeneratorOutputMW = 0;
                _state.TurbineSteamConsumptionKgPerS = 0;
                _state.TurbineInletEnthalpyKJPerKg = 0;
                _state.TurbineExhaustEnthalpyKJPerKg = 0;
                _state.CondenserHeatRejectionMW = 0;
                _state.TurbineUnderspeedAlarm = 0;
                _state.TurbineOverspeedWarn = 0;
                _state.TurbineBreakerAlarm = 0;
                if (_state.TurbineTripFlag == 1)
                    _state.GeneratorBreakerClosed = 0;
                _state.MainFeedPumpFlowKgPerS = _state.SgFeedwaterFlowKgPerS;
                return;
            }

            // ----------------------------------------------------------------
            //  Steam thermodynamics (Rankine cycle)
            // ----------------------------------------------------------------
            double pSteamMPa = _state.MainSteamPressureMPa;
            double hIn       = SaturatedSteamEnthalpyKJ(pSteamMPa);
            double sIn       = SaturatedSteamEntropyKJ(pSteamMPa);
            double dh        = TurbineIsentropicDrop(hIn, sIn, _state.CondenserTempC);
            const double etaIs = 0.85; // turbine isentropic efficiency (stage losses, moisture)
            double dhActual  = dh * etaIs;
            double hExhaust  = hIn - dhActual;

            _state.TurbineInletEnthalpyKJPerKg   = hIn;
            _state.TurbineExhaustEnthalpyKJPerKg = hExhaust;

            // ----------------------------------------------------------------
            //  Steam mass flow and mechanical power
            // ----------------------------------------------------------------
            double steamAvail = _state.SgSteamFlowKgPerS;
            double steamUsed  = _state.TurbineThrottlePosition * steamAvail;
            _state.TurbineSteamConsumptionKgPerS = steamUsed;

            double mechPowerMW = steamUsed * dhActual / 1000.0; // kJ/kg × kg/s → MW
            _state.TurbineMechanicalPowerMW = mechPowerMW;
            _state.TurbineLoadMW = mechPowerMW;

            double tCondC = _state.CondenserTempC;
            double hfCond = 4.186 * tCondC;
            _state.CondenserHeatRejectionMW = Math.Max(0, steamUsed * (hExhaust - hfCond) / 1000.0);

            // ----------------------------------------------------------------
            //  Swing equation — per-unit using rated electrical MW as base
            //    2H · dN_pu/dt = P_mech_e_pu − P_elec_pu − D·(N_pu−1)
            //
            //  Three modes determined by GridTiedMode and GeneratorBreakerClosed:
            //    A. Breaker open (runup): only mechanical friction, speed free
            //    B. Breaker closed, grid-tied: infinite-bus; sync torque holds N≈1
            //    C. Breaker closed, islanded: machine IS the grid; freq tracks speed
            // ----------------------------------------------------------------
            double etaGen    = _design.GeneratorEfficiency;
            double P_base    = _design.RatedElectricalMW;            // electrical MW base
            double N_pu      = _state.TurbineSpeedFraction;
            double H         = _design.TurbineInertiaConstantH;      // 6 s

            // Mechanical power converted to electrical-equivalent per-unit
            // (accounts for generator losses so the pu system is consistent)
            double P_mech_e_pu = mechPowerMW * etaGen / P_base;

            // Mechanical friction only (windage, bearing losses) — always present,
            // always opposes rotation. Very small: ~0.3% at rated speed.
            // This term ONLY applies when no electrical load damping is present.
            const double D_mech = 0.003;

            double accel;
            if (_state.GeneratorBreakerClosed == 1)
            {
                double D_load = _design.TurbineDampingD; // frequency-sensitive load damping

                if (_state.GridTiedMode == 1)
                {
                    // --- Mode B: Grid-tied (infinite bus) ---
                    // The grid provides a strong synchronising torque that holds N≈1.
                    // Electrical output follows mechanical power (governor determines both).
                    _state.GeneratorOutputMW = mechPowerMW * etaGen;
                    double P_elec_pu = _state.GeneratorOutputMW / P_base;

                    // Grid frequency: 60 Hz (infinite bus). Show small droop deviation
                    // to give the operator visibility into the power imbalance.
                    double imbalance = P_mech_e_pu - P_elec_pu;
                    _state.GridFrequencyHz = Math.Clamp(
                        60.0 + imbalance * _design.TurbineGovernorDroop * 60.0, 58.0, 62.0);

                    // Swing equation: synchronising torque (K_sync) strongly pulls N→1
                    double K_sync = _design.TurbineSyncKpow;
                    accel = (P_mech_e_pu - P_elec_pu
                             - D_load * (N_pu - 1.0)
                             + K_sync  * (1.0 - N_pu)) / (2.0 * H);
                }
                else
                {
                    // --- Mode C: Islanded (isochronous) ---
                    // Machine IS the grid. Frequency = rotor speed × 60.
                    // Electrical load = TurbinePowerTarget (what connected loads demand).
                    // If P_mech < load → frequency drops. Governor responds isochronously.
                    double P_load_pu = _state.TurbinePowerTarget / P_base;

                    // Swing equation: no synchronising torque; load damping still applies
                    // (connected induction-motor loads speed up when frequency rises).
                    accel = (P_mech_e_pu - P_load_pu
                             - D_load * (N_pu - 1.0)) / (2.0 * H);

                    _state.GridFrequencyHz = N_pu * 60.0;

                    // Generator delivers minimum of capacity and load demand
                    _state.GeneratorOutputMW = Math.Clamp(
                        mechPowerMW * etaGen, 0, _state.TurbinePowerTarget);
                }
            }
            else
            {
                // --- Mode A: Breaker open — runup or post-trip cooling ---
                // No electrical load. Only mechanical friction damps the rotor.
                accel = (P_mech_e_pu - D_mech * N_pu) / (2.0 * H);
                _state.GridFrequencyHz = N_pu * 60.0; // local bus tracks rotor
                _state.GeneratorOutputMW = 0;
            }

            // Euler integration with hard limits (step ≤ 0.05 s → stable for H=6)
            N_pu = Math.Clamp(N_pu + accel * dt, 0.0, _design.TurbineOverspeedTripFraction + 0.05);
            _state.TurbineSpeedFraction = N_pu;
            _state.TurbineSpeedRPM      = N_pu * _design.TurbineRatedSpeedRPM;

            // ----------------------------------------------------------------
            //  Emergency overspeed trip (hardware mechanical OST, cannot be bypassed)
            // ----------------------------------------------------------------
            if (N_pu > _design.TurbineOverspeedTripFraction && _state.TurbineTripFlag == 0)
            {
                _state.TurbineTripFlag        = 1;
                _state.GeneratorBreakerClosed = 0;
                LogEvent("TURB", $"Turbine EMERGENCY OVERSPEED TRIP at {N_pu * 100:F1}% rated speed.");
            }

            // ----------------------------------------------------------------
            //  Annunciator alarms
            // ----------------------------------------------------------------
            bool running = _state.TurbineOnline == 1 && _state.TurbineTripFlag == 0;
            double totalPower = _state.ThermalPowerFraction + _state.DecayHeatFraction;

            byte prevUnderspeed = _state.TurbineUnderspeedAlarm;
            byte prevOverWarn   = _state.TurbineOverspeedWarn;
            byte prevBkrAlarm   = _state.TurbineBreakerAlarm;

            _state.TurbineUnderspeedAlarm = (byte)(
                running && N_pu < _design.TurbineUnderspeedAlarmFraction ? 1 : 0);
            _state.TurbineOverspeedWarn = (byte)(
                running && N_pu > _design.TurbineOverspeedWarnFraction ? 1 : 0);
            // Breaker alarm: breaker trips unexpectedly while generating significant power
            _state.TurbineBreakerAlarm = (byte)(
                running && _state.GeneratorBreakerClosed == 0
                && _state.TurbineMechanicalPowerMW > 0.05 * P_base ? 1 : 0);

            // Log alarm transitions (alarm → active edge)
            if (_state.TurbineUnderspeedAlarm == 1 && prevUnderspeed == 0)
                LogEvent("TURB", $"ALARM: Turbine UNDERSPEED — {N_pu * 100:F1}% rated speed.");
            if (_state.TurbineOverspeedWarn == 1 && prevOverWarn == 0)
                LogEvent("TURB", $"ALARM: Turbine OVERSPEED WARNING — {N_pu * 100:F1}% (trip at {_design.TurbineOverspeedTripFraction * 100:F0}%).");
            if (_state.TurbineBreakerAlarm == 1 && prevBkrAlarm == 0)
                LogEvent("TURB", $"ALARM: Generator BREAKER OPEN at load ({_state.TurbineMechanicalPowerMW:F0} MW mech).");

            // Feedwater
            _state.MainFeedPumpFlowKgPerS = _state.SgFeedwaterFlowKgPerS;
        }

        // =====================================================================
        //  REACTOR COOLANT PUMPS
        // =====================================================================

        private void UpdateRcpModel(double dt)
        {
            double totalFlow = 0;
            for (int i = 0; i < 4; i++)
            {
                PumpState ps = (PumpState)(int)_state.RcpState[i];

                switch (ps)
                {
                    case PumpState.Running:
                        _state.RcpSpeed[i] = 1.0;
                        _state.RcpFlowFraction[i] = 1.0;
                        _state.RcpCurrentAmps[i] = 350.0;
                        break;

                    case PumpState.CoastingDown:
                        // exponential coast-down  τ ≈ 15 s
                        _state.RcpSpeed[i] *= Math.Exp(-dt / 15.0);
                        _state.RcpFlowFraction[i] = _state.RcpSpeed[i]; // affinity laws
                        _state.RcpCurrentAmps[i] = 0;
                        if (_state.RcpSpeed[i] < 0.01)
                        {
                            _state.RcpSpeed[i] = 0;
                            _state.RcpFlowFraction[i] = 0;
                            _state.RcpState[i] = (double)PumpState.Off;
                        }
                        break;

                    case PumpState.Starting:
                        _state.RcpSpeed[i] += dt / 20.0; // 20 s ramp to full
                        _state.RcpCurrentAmps[i] = 600; // inrush
                        if (_state.RcpSpeed[i] >= 1.0)
                        {
                            _state.RcpSpeed[i] = 1.0;
                            _state.RcpState[i] = (double)PumpState.Running;
                        }
                        _state.RcpFlowFraction[i] = _state.RcpSpeed[i];
                        break;

                    case PumpState.LockedRotor:
                        _state.RcpSpeed[i] = 0;
                        _state.RcpFlowFraction[i] = 0;
                        _state.RcpCurrentAmps[i] = 800; // stall current
                        break;

                    default: // Off
                        _state.RcpSpeed[i] = 0;
                        _state.RcpFlowFraction[i] = 0;
                        _state.RcpCurrentAmps[i] = 0;
                        break;
                }

                totalFlow += _state.RcpFlowFraction[i];
            }

            _state.TotalRcpFlowFraction = totalFlow / 4.0;
            _state.PrimaryFlowRateKgPerS = _state.TotalRcpFlowFraction
                                            * _design.DesignPrimaryFlowKgPerS;
            _state.PrimaryFlowFraction = _state.TotalRcpFlowFraction;
        }

        // =====================================================================
        //  CONTROL RODS
        // =====================================================================

        private void UpdateControlRods(double dt)
        {
            double speed = _state.RodSpeedStepsPerMin / 60.0; // steps per second

            for (int b = 0; b < 4; b++)
            {
                double pos = _state.RodBankPosition[b];
                double target = _state.RodBankTargetPosition[b];
                double diff = target - pos;

                if (Math.Abs(diff) < 0.1)
                    continue;

                // during scram, all rods fall under gravity (~2.5 s full stroke)
                if (_state.Scram != ScramState.Normal)
                {
                    double scramSpeed = _design.MaxRodSteps / 2.5; // steps/s
                    pos -= scramSpeed * dt;
                    pos = Math.Max(0, pos);
                }
                else
                {
                    double moveSteps = speed * dt;
                    if (diff > 0)
                        pos = Math.Min(pos + moveSteps, target);
                    else
                        pos = Math.Max(pos - moveSteps, target);
                }

                // enforce withdrawal sequence: D first, then C, B, A
                _state.RodBankPosition[b] = Math.Clamp(pos, 0, _design.MaxRodSteps);
            }
        }

        // =====================================================================
        //  BORON CONCENTRATION  (CVCS)
        // =====================================================================

        private void UpdateBoronConcentration(double dt)
        {
            double current = _state.BoronConcentrationPPM;
            double target = _state.BoronConcentrationTarget;
            double diff = target - current;
            if (Math.Abs(diff) < 0.01) return;

            // max rate ~ 2 ppm/min dilution, 5 ppm/min boration
            double maxRate = diff > 0 ? 5.0 / 60.0 : -2.0 / 60.0;
            double rate = Math.Sign(diff) * Math.Min(Math.Abs(diff), Math.Abs(maxRate) * 10);
            rate = Math.Clamp(rate, -2.0 / 60.0, 5.0 / 60.0);

            _state.BoronConcentrationPPM += rate * dt;
            _state.BoronConcentrationPPM = Math.Clamp(_state.BoronConcentrationPPM, 0, 3000);
        }

        // =====================================================================
        //  ECCS  (Emergency Core Cooling System)
        // =====================================================================

        private void UpdateEccs(double dt)
        {
            _state.HpiFlowKgPerS = 0;
            _state.LpiFlowKgPerS = 0;

            // SI signal is computed by the controller and stored in state.
            bool siSignal = _state.EccsSiSignal == 1;

            if (!siSignal)
            {
                // also check if accumulators should passively dump (they don't
                // need an SI signal — they are purely pressure-driven — but
                // only when the primary was previously pressurised)
                if (_state.AccumulatorIsolated == 0
                    && _state.PrimaryPressureMPa < _state.AccumulatorPressureMPa
                    && _state.AccumulatorWaterVolumeM3 > 0
                    && _state.Mode != ReactorMode.Shutdown
                    && _state.Mode != ReactorMode.Refueling)
                {
                    _state.EccsStatus = EccsMode.Accumulator;
                    double accumFlow = 200.0;
                    double volumeUsed = (accumFlow / _design.CoolantDensityNominal) * dt;
                    _state.AccumulatorWaterVolumeM3 = Math.Max(0,
                        _state.AccumulatorWaterVolumeM3 - volumeUsed);
                    _state.HpiFlowKgPerS += accumFlow;
                }
                else if (_state.EccsStatus != EccsMode.Standby)
                {
                    _state.EccsStatus = EccsMode.Standby;
                }
                return;
            }

            // HPI: available when P < ~12 MPa
            if (_state.PrimaryPressureMPa < 12.0)
            {
                _state.EccsStatus = EccsMode.HighPressureInjection;
                _state.HpiFlowKgPerS = _design.HpiDesignFlowKgPerS
                                       * Math.Clamp((12.0 - _state.PrimaryPressureMPa) / 4.0, 0, 1);
            }

            // Accumulators: passive injection when P drops below accumulator P
            if (_state.AccumulatorIsolated == 0
                && _state.PrimaryPressureMPa < _state.AccumulatorPressureMPa
                && _state.AccumulatorWaterVolumeM3 > 0)
            {
                _state.EccsStatus = EccsMode.Accumulator;
                double accumFlow = 200.0;
                double volumeUsed = (accumFlow / _design.CoolantDensityNominal) * dt;
                _state.AccumulatorWaterVolumeM3 = Math.Max(0,
                    _state.AccumulatorWaterVolumeM3 - volumeUsed);
                _state.HpiFlowKgPerS += accumFlow;
            }

            // LPI: available when P < ~1.5 MPa
            if (_state.PrimaryPressureMPa < 1.5)
            {
                _state.EccsStatus = EccsMode.LowPressureInjection;
                _state.LpiFlowKgPerS = _design.LpiDesignFlowKgPerS;
            }
        }

        // =====================================================================
        //  CONTAINMENT  (simplified single-node)
        // =====================================================================

        private void UpdateContainment(double dt)
        {
            // Containment pressurises from steam releases directly into the building.
            // PORV discharge goes to the pressurizer relief tank (not containment) —
            // only the code safety valve (17.24 MPa overpressure accident) discharges
            // to containment.  LOCA break flow is handled by the ECCS model.
            double leakRate = 0; // kg/s equivalent steam
            if (_state.SafetyValveOpen == 1)
                leakRate += 200.0;

            // very simplified: containment P rises with leak
            double dPcont = leakRate * 0.0002 * dt;
            _state.ContainmentPressureKPa += dPcont;

            // heat loss to containment walls / fan coolers
            _state.ContainmentPressureKPa -= 0.001 * dt;
            _state.ContainmentPressureKPa = Math.Clamp(
                _state.ContainmentPressureKPa, 100.0, 600.0);

            _state.ContainmentTempC = 30.0 + (_state.ContainmentPressureKPa - 101.3) * 0.3;
        }

        // =====================================================================
        //  ELECTRICAL
        // =====================================================================

        private void UpdateElectrical(double dt)
        {
            // GeneratorOutputMW is already set by UpdateSecondaryAndTurbine (swing equation).
            _state.NetElectricalMW = _state.GeneratorOutputMW - _state.HousekeepingLoadMW;
        }

        // =====================================================================
        //  FUEL & CLADDING INTEGRITY
        // =====================================================================

        private void UpdateFuelIntegrity(double dt)
        {
            // Zircaloy-4 oxidation (Baker-Just correlation, simplified)
            // oxide growth rate ∝ exp(-Ea/RT) at cladding temperature
            double Tclad_K = _state.CladOuterTemp + 273.15;
            if (Tclad_K > 600) // only significant at high T
            {
                double Ea = 45500.0; // cal/mol
                double R = 1.987;     // cal/mol·K
                double rate = 33.3e6 * Math.Exp(-Ea / (R * Tclad_K)); // cm²/s (parabolic)
                double dOxide = rate * dt * 1e4; // µm
                _state.CladOxideThickness += dOxide;
            }

            // DNBR (simplified W-3 correlation placeholder)
            double actualHeatFlux = _state.LinearHeatRate / (2 * Math.PI * _design.CladOuterRadiusM);
            if (actualHeatFlux < 1.0)
            {
                // negligible heat flux → DNBR is not meaningful, report safe
                _state.DNBRatio = 99.0;
            }
            else if (_state.PrimaryFlowFraction < 0.05)
            {
                // significant heat flux with no flow → genuine DNB concern
                // report low but not zero (CHF correlations undefined at zero flow)
                _state.DNBRatio = 0.1;
            }
            else
            {
                double criticalHeatFlux = 3.0e6; // W/m² (typical at PWR conditions)
                criticalHeatFlux *= _state.PrimaryFlowFraction;
                criticalHeatFlux *= (1.0 + 0.01 * Math.Max(0, _state.SubcoolingMarginC));
                _state.DNBRatio = criticalHeatFlux / actualHeatFlux;
            }

            // fuel melt check (UO₂ melts at ~2840 °C)
            _state.FuelMeltFraction = Math.Clamp(
                (_state.FuelCentrelineTemp - 2840.0) / 200.0, 0, 1);

            // cladding hoop stress  (simplified thin-wall)
            double Pinternal = _state.PrimaryPressureMPa; // fission gas ≈ primary P for simplicity
            double Pexternal = _state.PrimaryPressureMPa;
            double dP = Math.Max(0, Pinternal - Pexternal + 2.0); // fission-gas over-pressure
            double rClad = (_design.CladInnerRadiusM + _design.CladOuterRadiusM) / 2.0;
            double tClad = _design.CladOuterRadiusM - _design.CladInnerRadiusM;
            _state.CladHoopStress = dP * 1e6 * rClad / tClad / 1e6; // MPa
        }

        // =====================================================================
        //  CORE DESTRUCTION  (Zr-steam reaction, H₂ generation, damage states)
        // =====================================================================

        private void UpdateCoreDestruction(double dt)
        {
            double Tclad = _state.CladOuterTemp;
            double Tfuel = _state.FuelCentrelineTemp;

            // === Zircaloy-steam exothermic reaction ===
            // Above 1200 °C the Baker-Just parabolic regime transitions to an
            // ablative/steam-limited regime.  Rate increases with temperature.
            // Zr + 2H₂O → ZrO₂ + 2H₂   ΔH = −6.45 MJ/kg Zr
            const double ZrMassKg     = 20_000.0;  // ~20 t cladding in 193-assembly core
            const double ZrHeatPerKg  = 6.45e6;    // J/kg Zr oxidised
            const double H2PerKgZr    = 0.044;     // kg H₂ per kg Zr (stoichiometric)

            double remaining = 1.0 - _state.ZirconiumOxidizedFraction;
            double zrRatePerS = 0.0;
            if (Tclad > 1200.0 && remaining > 1e-4)
            {
                // log-linear rate: 1e-7/s at 1200 °C, 1e-4/s at 1800 °C, capped at 1800 °C
                double Tcap = Math.Min(Tclad, 1800.0);
                double logRate = -7.0 + (Tcap - 1200.0) / 600.0 * 3.0;
                zrRatePerS = Math.Pow(10.0, logRate) * remaining;
            }
            double dZrFrac = Math.Min(zrRatePerS * dt, remaining);
            _state.ZirconiumOxidizedFraction = Math.Clamp(
                _state.ZirconiumOxidizedFraction + dZrFrac, 0.0, 1.0);

            double dZrKg = dZrFrac * ZrMassKg;
            _state.CoreExothermicHeatMW = dZrKg * ZrHeatPerKg / (dt + 1e-30) * 1e-6;
            _state.HydrogenGenerationKg += dZrKg * H2PerKgZr;

            // === Core damage fraction ===
            // Cladding failure: balloon/rupture above PCT limit; rate grows with excess T
            double damageRate = 0.0;
            if (Tclad > _state.CladTemperatureLimit)
            {
                double excess = Tclad - _state.CladTemperatureLimit;
                // at 1204 °C: 0/s; at 1800 °C: ~0.001/s; beyond: faster via fuel melt term
                damageRate = Math.Min(excess / 596.0, 1.0) * 1e-3;
            }
            // Fuel melt adds an additional rapid damage contribution
            if (_state.FuelMeltFraction > 0)
                damageRate = Math.Max(damageRate, _state.FuelMeltFraction * 0.01);

            _state.CoreDamageFraction = Math.Clamp(
                _state.CoreDamageFraction + damageRate * dt, 0.0, 1.0);

            // === Core damage state (ratchet — never decreases) ===
            CoreDamageState newState = _state.CoreDamageState;
            double dmg = _state.CoreDamageFraction;
            if (Tclad > _state.CladTemperatureLimit || dmg > 0.001)
                if (newState < CoreDamageState.CladFailure) newState = CoreDamageState.CladFailure;
            if (_state.FuelMeltFraction > 0 || dmg > 0.05)
                if (newState < CoreDamageState.FuelMelting) newState = CoreDamageState.FuelMelting;
            if (dmg > 0.15)
                if (newState < CoreDamageState.CoreDamage)  newState = CoreDamageState.CoreDamage;
            if (dmg > 0.50)
                if (newState < CoreDamageState.Meltdown)    newState = CoreDamageState.Meltdown;
            if (dmg > 0.90)
                if (newState < CoreDamageState.VesselFailure) newState = CoreDamageState.VesselFailure;

            if (newState != _state.CoreDamageState)
            {
                LogEvent("CORE", $"Core damage state → {newState}  (damage {_state.CoreDamageFraction:P1}, Zr {_state.ZirconiumOxidizedFraction:P1} oxidised)");
                _state.CoreDamageState = newState;
            }

            // === Fission-product release to primary ===
            // Volatile FPs (I-131, Cs-137) released as cladding fails; scales with damage
            _state.FissionProductReleaseFraction = Math.Max(
                _state.FissionProductReleaseFraction, _state.CoreDamageFraction);

            // === Hydrogen in containment ===
            // Containment free volume ≈ 50 000 m³; at STP 1 kg H₂ = 11.2 m³
            const double ContVolumeM3 = 50_000.0;
            _state.ContainmentH2FractionPct =
                (_state.HydrogenGenerationKg * 11.2) / ContVolumeM3 * 100.0;

            // H₂ detonation when concentration enters the detonable range (8–18 vol%)
            // A continuous ignition source (safety valve arcing, hot surfaces) is assumed
            // once core damage has begun.  We trigger at 10% and burn 60% of accumulated H₂.
            if (_state.ContainmentH2FractionPct > 10.0
                && _state.CoreDamageState >= CoreDamageState.CladFailure)
            {
                double h2Burned = _state.HydrogenGenerationKg * 0.6;
                _state.HydrogenGenerationKg -= h2Burned;
                // Combustion energy → containment pressure spike (~142 MJ/kg H₂)
                double dPkPa = h2Burned * 142e6 / ContVolumeM3 * 1e-3;
                _state.ContainmentPressureKPa += dPkPa;
                LogEvent("CORE", $"Hydrogen deflagration/detonation in containment — {h2Burned:F0} kg H₂ burned, +{dPkPa:F0} kPa");
            }

            // === Ex-vessel corium at vessel failure ===
            if (_state.CoreDamageState == CoreDamageState.VesselFailure)
            {
                // Corium contacts liner; molten-core–concrete interaction produces CO₂/H₂
                // Simplified: 0.05 kPa/s additional containment pressurisation
                _state.ContainmentPressureKPa += 0.05 * dt;
                // Additional hydrogen from MCCI (~0.005 kg/s simplified)
                _state.HydrogenGenerationKg += 0.005 * dt;
            }
        }

        // =====================================================================
        //  BURNUP
        // =====================================================================

        private void UpdateBurnup(double dt)
        {
            // MWd/tHM accumulation
            // assume 100 tonnes heavy metal in core
            double tHM = 100.0;
            double powerMWd = _state.ThermalPowerMW * dt / 86400.0;
            _state.FuelBurnupMWdPerTonne += powerMWd / tHM;
        }


        // =====================================================================
        //  SCRAM EXECUTION
        // =====================================================================

        public void ExecuteScram(ScramState type)
        {
            _state.Scram = type;
            _state.Mode = ReactorMode.Tripped;
            _state.TripTimestamp = _state.SimulationTimeSeconds;

            // command all rods to fully insert
            for (int b = 0; b < 4; b++)
                _state.RodBankTargetPosition[b] = 0;

            // trip turbine
            _state.TurbineTripFlag = 1;

            // trip main feed pump
            _state.MainFeedPumpTripped = 1;
        }

        // =====================================================================
        //  UTILITY: saturation temperature approximation
        // =====================================================================

        /// <summary>
        /// Approximate Tsat(P) for water.  P in MPa, returns °C.
        /// Antoine-type fit valid 0.1 – 22 MPa (±1 °C typical).
        /// </summary>
        private static double SaturationTemperature(double pressureMPa)
        {
            double P = Math.Clamp(pressureMPa, 0.01, 22.0);
            // Antoine equation for water (valid ~45–374 °C)
            // Tsat = B / (A − log10(P_mmHg)) − C
            double log10PmmHg = Math.Log10(P * 7500.6);
            return 1810.94 / (8.14019 - log10PmmHg) - 244.485;
        }

        /// <summary>
        /// Saturated steam enthalpy (kJ/kg) at given pressure (MPa).
        /// Linear fit to IAPWS-IF97 data: ~2765 kJ/kg at 7 MPa, ~2800 at 1 MPa.
        /// </summary>
        /// <summary>
        /// Saturated vapour enthalpy hg (kJ/kg) vs pressure (MPa).
        /// Fit using ln(P) basis against IAPWS-IF97; error &lt; 20 kJ/kg over 0.001–15 MPa.
        ///   P=0.005 MPa → 2561  (real 2561) ✓
        ///   P=1.0   MPa → 2778  (real 2778) ✓
        ///   P=7.0   MPa → 2773  (real 2773) ✓
        ///   P=15    MPa → 2596  (real 2611) ✓
        /// </summary>
        private static double SaturatedSteamEnthalpyKJ(double pMPa)
        {
            double x = Math.Log(Math.Clamp(pMPa, 0.001, 20.0));  // ln(P/MPa)
            return 2778.0 + 9.126 * x - 6.009 * x * x;
        }

        /// <summary>
        /// Saturated vapour entropy sg (kJ/kg·K) vs pressure (MPa).
        /// Fit using ln(P) basis against IAPWS-IF97; error &lt; 0.1 kJ/kg·K over 0.001–15 MPa.
        ///   P=0.005 MPa → 8.34  (real 8.61, Δ=0.27) acceptable at condenser conditions
        ///   P=1.0   MPa → 6.59  (real 6.59) ✓
        ///   P=7.0   MPa → 5.77  (real 5.81) ✓
        /// </summary>
        private static double SaturatedSteamEntropyKJ(double pMPa)
        {
            double x = Math.Log(Math.Clamp(pMPa, 0.001, 20.0));  // ln(P/MPa)
            return 6.585 - 0.394 * x - 0.0118 * x * x;
        }

        /// <summary>
        /// Actual enthalpy drop (kJ/kg) across the turbine, expanding from (h_in, s_in)
        /// to condenser at T_cond (°C).  Assumes 85% isentropic efficiency and wet-steam
        /// exhaust at condenser conditions.
        /// </summary>
        private static double TurbineIsentropicDrop(double hInKJ, double sInKJ, double tCondC)
        {
            double tK = tCondC + 273.15;
            // Condenser saturation pressure from temperature (Antoine equation)
            double pCond = Math.Pow(10, 8.14019 - 1810.94 / (tK - 28.665)) / 7500.6;
            pCond = Math.Clamp(pCond, 0.001, 0.1);

            // Condenser liquid properties
            double hf  = 4.186 * tCondC;                                // liquid enthalpy (kJ/kg)
            double sf  = 4.186 * Math.Log(tK / 273.15);                 // liquid entropy  (kJ/kg·K)
            double hfg = SaturatedSteamEnthalpyKJ(pCond) - hf;          // latent heat
            double sfg = SaturatedSteamEntropyKJ(pCond) - sf;           // entropy of vaporisation

            // Isentropic exhaust quality (wet steam)
            // Clamped 0.65–1.0: below 0.65 excessive moisture; above 1.0 is superheated
            double x = Math.Clamp((sInKJ - sf) / (sfg + 1e-6), 0.65, 1.0);
            double hExhaust = hf + x * hfg;
            return hInKJ - hExhaust;
        }

        // =====================================================================
        //  EVENT LOG
        // =====================================================================

        public void LogEvent(string category, string message)
        {
            _events.Add(new SimulationEvent(_state.SimulationTimeSeconds, category, message));
            if (_events.Count > 10000)
                _events.RemoveRange(0, 1000); // rolling buffer
        }

        public void ClearEventLog() => _events.Clear();

        // =====================================================================
        //  OPERATOR COMMAND API
        // =====================================================================

        // ---- Control Rods ---------------------------------------------------

        /// <summary>Set target position for a control rod bank (0=A,1=B,2=C,3=D).</summary>
        public void CommandRodBank(int bank, double targetSteps)
        {
            if (bank < 0 || bank > 3) throw new ArgumentOutOfRangeException(nameof(bank));
            if (_state.Scram != ScramState.Normal && _state.RodScramOverride == 0)
            { LogEvent("CMD", "Rod ops blocked during scram (enable Rod Scram Override to bypass)."); return; }
            targetSteps = Math.Clamp(targetSteps, 0, _design.MaxRodSteps);
            _state.RodBankTargetPosition[bank] = targetSteps;
            LogEvent("CMD", $"Rod bank {(char)('A'+bank)} target → {targetSteps:F0} steps.");
        }

        /// <summary>Set rod stepping speed (steps per minute).</summary>
        public void CommandRodSpeed(double stepsPerMin)
        {
            _state.RodSpeedStepsPerMin = Math.Clamp(stepsPerMin, 0, _state.MaxRodSpeedStepsPerMin);
            LogEvent("CMD", $"Rod speed → {_state.RodSpeedStepsPerMin:F0} steps/min.");
        }

        /// <summary>Withdraw bank D by N steps from current position.</summary>
        public void WithdrawRods(int bank, double steps)
        {
            CommandRodBank(bank, _state.RodBankPosition[bank] + steps);
        }

        /// <summary>Insert bank D by N steps from current position.</summary>
        public void InsertRods(int bank, double steps)
        {
            CommandRodBank(bank, _state.RodBankPosition[bank] - steps);
        }

        /// <summary>Toggle automatic rod control (Tavg program).</summary>
        public void SetAutoRodControl(bool enabled)
        {
            _state.RodControlAutoMode = (byte)(enabled ? 1 : 0);
            LogEvent("CMD", $"Auto rod control {(enabled ? "ENABLED" : "DISABLED")}.");
        }

        // ---- Reactor Trip ---------------------------------------------------

        /// <summary>Manual reactor trip (scram).</summary>
        public void ManualTrip()
        {
            _state.ActiveTrips |= TripSignal.ManualTrip;
            ExecuteScram(ScramState.ManualScram);
            LogEvent("CMD", "MANUAL REACTOR TRIP initiated.");
        }

        /// <summary>Reset trip after conditions cleared.  Operator must re-verify.</summary>
        public void ResetTrip()
        {
            if (_state.Scram == ScramState.Normal) return;
            _state.Scram = ScramState.Normal;
            _state.ActiveTrips = TripSignal.None;
            _state.Mode = ReactorMode.Shutdown;
            _state.TurbineTripFlag = 0;
            _state.MainFeedPumpTripped = 0;
            LogEvent("CMD", "Trip reset.  Reactor is now in SHUTDOWN mode.");
        }

        /// <summary>Enable or disable the Reactor Protection System  (for training only!).</summary>
        public void SetProtectionSystem(bool enabled)
        {
            _state.ProtectionSystemEnabled = (byte)(enabled ? 1 : 0);
            LogEvent("CMD", $"RPS {(enabled ? "ENABLED" : "BYPASSED (training mode)")}.");
        }

        /// <summary>Bypass or un-bypass a specific trip function.  The condition is still
        /// annunciated in ActiveTrips but will not trigger an automatic scram.</summary>
        public void SetTripBypassed(TripSignal trip, bool bypassed)
        {
            if (bypassed)
                _state.TripBypassFlags |= trip;
            else
                _state.TripBypassFlags &= ~trip;
            LogEvent("RPS", $"Trip bypass {(bypassed ? "SET" : "CLEARED")}: {trip}");
        }

        // ---- Boron / CVCS ---------------------------------------------------

        /// <summary>Set boron concentration target (ppm).  CVCS will dilute or borate.</summary>
        public void CommandBoronTarget(double ppm)
        {
            _state.BoronConcentrationTarget = Math.Clamp(ppm, 0, 3000);
            LogEvent("CMD", $"Boron target → {ppm:F0} ppm.");
        }

        /// <summary>Emergency boration (sets target to high value immediately).</summary>
        public void EmergencyBoration()
        {
            _state.BoronConcentrationTarget = 2400;
            LogEvent("CMD", "EMERGENCY BORATION initiated (target 2400 ppm).");
        }

        // ---- Reactor Coolant Pumps ------------------------------------------

        /// <summary>Start or stop a reactor coolant pump (0-3).</summary>
        public void CommandRcp(int pump, bool start)
        {
            if (pump < 0 || pump > 3) throw new ArgumentOutOfRangeException(nameof(pump));
            if (start)
            {
                _state.RcpState[pump] = (double)PumpState.Starting;
                LogEvent("CMD", $"RCP {pump+1} START commanded.");
            }
            else
            {
                _state.RcpState[pump] = (double)PumpState.CoastingDown;
                LogEvent("CMD", $"RCP {pump+1} STOP commanded (coast-down).");
            }
        }

        // ---- Pressuriser ----------------------------------------------------

        /// <summary>Set pressuriser heater power command (kW).</summary>
        public void CommandPressuriserHeaters(double kw)
        {
            _state.PressuriserHeaterCommandKW = Math.Clamp(kw, 0, _state.PressuriserHeaterMaxKW);
            _state.PressuriserHeaterPowerKW = _state.PressuriserHeaterCommandKW;
            LogEvent("CMD", $"Pressuriser heaters → {_state.PressuriserHeaterCommandKW:F0} kW.");
        }

        /// <summary>Override pressuriser spray.</summary>
        public void CommandPressuriserSpray(bool on)
        {
            _state.PressuriserSprayOn = (byte)(on ? 1 : 0);
        }

        // ---- Turbine / Generator --------------------------------------------

        /// <summary>Bring turbine on-line or take off-line.</summary>
        public void CommandTurbineOnline(bool online)
        {
            if (online)
            {
                _state.TurbineOnline = 1;
                _state.TurbineTripFlag = 0;
                _state.TurbineSpeedRPM = 100; // start rolling
                _state.GeneratorBreakerClosed = 0; // breaker must be closed separately at synchronous speed
                LogEvent("CMD", "Turbine LATCHED and rolling — sync then close breaker.");
            }
            else
            {
                _state.TurbineTripFlag = 1;
                _state.GeneratorBreakerClosed = 0;
                LogEvent("CMD", "Turbine TRIP commanded.");
            }
        }

        /// <summary>Close or open the generator output breaker (grid connection).</summary>
        public void CommandGeneratorBreaker(bool close)
        {
            if (close)
            {
                if (_state.TurbineOnline == 0 || _state.TurbineTripFlag == 1)
                {
                    LogEvent("CMD", "Breaker close rejected — turbine not running.");
                    return;
                }
                double speedPct = _state.TurbineSpeedFraction * 100;
                if (_state.TurbineSpeedFraction < 0.98 || _state.TurbineSpeedFraction > 1.02)
                {
                    LogEvent("CMD", $"Breaker close rejected — speed {speedPct:F1}% (sync window 98–102%).");
                    return;
                }
                _state.GeneratorBreakerClosed = 1;
                LogEvent("CMD", "Generator breaker CLOSED — synchronized to grid.");
            }
            else
            {
                _state.GeneratorBreakerClosed = 0;
                LogEvent("CMD", "Generator breaker OPENED.");
            }
        }

        /// <summary>Switch between grid-tied (infinite-bus) and islanded (isochronous) mode.</summary>
        public void CommandGridMode(bool gridTied)
        {
            _state.GridTiedMode = (byte)(gridTied ? 1 : 0);
            LogEvent("CMD", gridTied
                ? "Grid mode → GRID TIED (infinite bus, 60 Hz reference)."
                : "Grid mode → ISLANDED (isochronous, machine is the grid).");
        }

        /// <summary>Set turbine electrical load target (MWe).</summary>
        public void CommandTurbineLoad(double mwe)
        {
            _state.TurbinePowerTarget = Math.Clamp(mwe, 0, _design.RatedElectricalMW * 1.05);
            LogEvent("CMD", $"Turbine load target → {mwe:F0} MWe.");
        }

        // ---- Steam Generator ------------------------------------------------

        /// <summary>Set SG feedwater temperature (simulates FW heater changes).</summary>
        public void SetFeedwaterTemperature(double tempC)
        {
            _state.SgFeedwaterTempC = Math.Clamp(tempC, 20, 260);
            _state.FeedwaterTempC = _state.SgFeedwaterTempC;
        }

        /// <summary>Adjust SG tube fouling factor  (1.0 = clean, 0.5 = badly fouled).</summary>
        public void SetSgFouling(double factor)
        {
            _state.SgTubeFoulingFactor = Math.Clamp(factor, 0.1, 1.0);
        }

        // ---- ECCS / Safety --------------------------------------------------

        /// <summary>Manually initiate safety injection.</summary>
        public void InitiateSafetyInjection()
        {
            _state.ActiveTrips |= TripSignal.SafetyInjection;
            LogEvent("CMD", "MANUAL SAFETY INJECTION initiated.");
        }

        /// <summary>Isolate accumulators (for low-pressure operations).</summary>
        public void IsolateAccumulators(bool isolate)
        {
            _state.AccumulatorIsolated = (byte)(isolate ? 1 : 0);
            LogEvent("CMD", $"Accumulators {(isolate ? "ISOLATED" : "ALIGNED")}.");
        }

        // ---- Electrical -----------------------------------------------------

        /// <summary>Simulate loss of off-site power.</summary>
        public void SetOffSitePower(bool available)
        {
            _state.OffSitePowerAvailable = (byte)(available ? 1 : 0);
            LogEvent("CMD", $"Off-site power {(available ? "AVAILABLE" : "LOST")}.");
        }

        /// <summary>Start/stop diesel generator.</summary>
        public void CommandDieselGenerator(bool start)
        {
            _state.DieselGeneratorRunning = (byte)(start ? 1 : 0);
            LogEvent("CMD", $"Diesel generator {(start ? "STARTED" : "STOPPED")}.");
        }

        // ---- Operator bias (shim) -------------------------------------------

        /// <summary>Add a manual reactivity bias (Δk/k).  Use sparingly.</summary>
        public void SetManualReactivityBias(double dkk)
        {
            _state.RhoManualBias = dkk;
            LogEvent("CMD", $"Manual reactivity bias → {dkk * 1e5:F1} pcm.");
        }

        // ---- Mode selection -------------------------------------------------

        /// <summary>Set reactor operational mode (advisory flag).</summary>
        public void SetMode(ReactorMode mode)
        {
            _state.Mode = mode;
            LogEvent("CMD", $"Mode → {mode}.");
        }

        /// <summary>Set peaking factor  (for scenario setup).</summary>
        public void SetPeakingFactor(double fq)
        {
            _state.PeakingFactor = Math.Clamp(fq, 1.0, 4.0);
        }

        // ---- auto-control parameter setters ----

        public void SetRodScramOverride(bool enabled)
            => _state.RodScramOverride = enabled ? (byte)1 : (byte)0;

        public void SetRodAutoGain(double stepsPerMinPerC)
            => _state.RodAutoGainStepsPerMinPerC = Math.Max(0, stepsPerMinPerC);

        public void SetRodAutoDeadband(double deadbandC)
            => _state.RodAutoDeadbandC = Math.Max(0, deadbandC);

        public void SetPrzHeaterAuto(bool enabled)
            => _state.PrzHeaterAutoEnabled = enabled ? (byte)1 : (byte)0;

        public void SetPrzSprayAuto(bool enabled)
            => _state.PrzSprayAutoEnabled = enabled ? (byte)1 : (byte)0;

        public void SetPrzPressureSetpoint(double mpa)
            => _state.PrzPressureSetpointMPa = Math.Clamp(mpa, 5.0, 20.0);

        public void SetPrzAutoGain(double gain)
            => _state.PrzAutoGain = Math.Max(0, gain);

        public void SetPrzAutoDeadband(double mpa)
            => _state.PrzAutoDeadbandMPa = Math.Max(0, mpa);

        public void SetTurbineGovernorAuto(bool enabled)
            => _state.TurbineGovernorAutoEnabled = enabled ? (byte)1 : (byte)0;

        public void SetFeedwaterAuto(bool enabled)
            => _state.FeedwaterAutoEnabled = enabled ? (byte)1 : (byte)0;

        public void SetFeedwaterTripOverride(bool enabled)
            => _state.FeedwaterTripOverride = enabled ? (byte)1 : (byte)0;

        public void SetFwLevelSetpoint(double fraction)
            => _state.FwLevelSetpoint = Math.Clamp(fraction, 0.1, 0.9);

        public void SetFwLevelGain(double gain)
            => _state.FwLevelGain = Math.Max(0, gain);

        public void SetFwFlowGain(double gain)
            => _state.FwFlowGain = Math.Max(0, gain);

        // =====================================================================
        //  QUERY / DIAGNOSTIC API
        // =====================================================================

        /// <summary>Quick summary struct for UI display.</summary>
        public readonly struct DiagnosticSummary
        {
            public readonly double PowerPercent;
            public readonly double TavgC;
            public readonly double PressureMPa;
            public readonly double BoronPPM;
            public readonly double BankDSteps;
            public readonly double XenonFraction;
            public readonly double StartupRate;
            public readonly double DNBRatio;
            public readonly double GeneratorMW;
            public readonly ReactorMode Mode;
            public readonly TripSignal Trips;

            public DiagnosticSummary(in ReactorState s)
            {
                PowerPercent = (s.ThermalPowerFraction + s.DecayHeatFraction) * 100.0;
                TavgC = s.TavgC;
                PressureMPa = s.PrimaryPressureMPa;
                BoronPPM = s.BoronConcentrationPPM;
                BankDSteps = s.RodBankPosition.V3;
                XenonFraction = (s.XenonEquilibrium > 0)
                    ? s.Xenon135 / s.XenonEquilibrium : 0;
                StartupRate = s.StartupRateDPM;
                DNBRatio = s.DNBRatio;
                GeneratorMW = s.GeneratorOutputMW;
                Mode = s.Mode;
                Trips = s.ActiveTrips;
            }
        }

        /// <summary>Get a quick diagnostic snapshot for a HUD / dashboard.</summary>
        public DiagnosticSummary GetDiagnostics() => new DiagnosticSummary(in _state);

        /// <summary>Get total reactivity in pcm (parts per 100,000).</summary>
        public double GetTotalReactivityPCM() => _state.RhoTotal * 1e5;

        /// <summary>Get the six-group precursor concentrations as an array.</summary>
        public double[] GetPrecursors()
        {
            double[] c = new double[6];
            for (int i = 0; i < 6; i++) c[i] = _state.PrecursorConcentrations[i];
            return c;
        }

        /// <summary>Get human-readable reactivity breakdown.</summary>
        public Dictionary<string, double> GetReactivityBreakdownPCM()
        {
            return new Dictionary<string, double>
            {
                ["ControlRods"] = _state.RhoControlRods * 1e5,
                ["Doppler"]     = _state.RhoDoppler * 1e5,
                ["Moderator"]   = _state.RhoModerator * 1e5,
                ["Boron"]       = _state.RhoBoron * 1e5,
                ["Xenon"]       = _state.RhoXenon * 1e5,
                ["Samarium"]    = _state.RhoSamarium * 1e5,
                ["Void"]        = _state.RhoVoid * 1e5,
                ["ManualBias"]  = _state.RhoManualBias * 1e5,
                ["Total"]       = _state.RhoTotal * 1e5,
            };
        }
    }
}
