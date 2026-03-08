// =============================================================================
// Program.cs — PWR Reactor Simulation Web Server
// =============================================================================
// ASP.NET Core minimal-API web server with an embedded retro-CRT control UI.
//
// Build & run:
//   cd PwrSimWeb
//   dotnet run
//
// Then open http://localhost:5000
// =============================================================================

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;
using PwrSimulator;

namespace PwrSimWeb
{
    // =====================================================================
    //  Request / response DTOs
    // =====================================================================
    public record TickRequest(double Dt = 1.0);
    public record AutoTickRequest(bool Enabled = false, double Speed = 1.0);
    public record RodBankCmd(int Bank = 3, double Target = 0);
    public record RodSpeedCmd(double Speed = 48);
    public record RodMoveCmd(int Bank = 3, double Steps = 1);
    public record ToggleCmd(bool Enabled = true);
    public record BoronCmd(double Ppm = 800);
    public record RcpCmd(int Pump = 0, bool Start = true);
    public record HeaterCmd(double Kw = 0);
    public record TurbineLoadCmd(double Mwe = 1150);
    public record FeedTempCmd(double TempC = 227);
    public record FoulingCmd(double Factor = 1.0);
    public record BiasCmd(double Dkk = 0);
    public record PeakingCmd(double Fq = 2.5);
    public record ValueCmd(double Value = 0);
    public record ModeCmd(string Mode = "SteadyState");
    public record BypassCmd(string Signal = "None", bool Bypassed = false);

    // =====================================================================
    //  Program
    // =====================================================================
    public class Program
    {
        private static ReactorEngine _engine = new ReactorEngine();
        private static ReactorController _controller = new ReactorController(_engine);
        private static readonly object _lock = new object();
        private static bool _autoTick = false;
        private static double _simSpeed = 1.0;

        private static readonly ConcurrentDictionary<string, WebSocket> _wsClients = new();
        private static int _wsNextId = 0;

        private static readonly JsonSerializerOptions JsonOpts = new JsonSerializerOptions
        {
            PropertyNamingPolicy = JsonNamingPolicy.CamelCase,
            Converters = { new JsonStringEnumConverter() },
            WriteIndented = false,
            NumberHandling = System.Text.Json.Serialization.JsonNumberHandling.AllowNamedFloatingPointLiterals
        };

        public static void Main(string[] args)
        {
            _engine.InitialiseFullPower();

            var builder = WebApplication.CreateBuilder(args);
            builder.Services.AddCors();
            var app = builder.Build();
            app.UseCors(p => p.AllowAnyOrigin().AllowAnyMethod().AllowAnyHeader());
            app.UseWebSockets();
            app.UseDefaultFiles();
            app.UseStaticFiles();

            var cts = new CancellationTokenSource();
            _ = Task.Run(async () => await BackgroundTicker(cts.Token));

            MapRoutes(app);

            app.Urls.Add("http://0.0.0.0:5000");
            Console.WriteLine("========================================");
            Console.WriteLine("  PWR Reactor Control Terminal");
            Console.WriteLine("  http://localhost:5000");
            Console.WriteLine("========================================");
            app.Run();

            cts.Cancel();
        }

        private static async Task BackgroundTicker(CancellationToken ct)
        {
            var sw = Stopwatch.StartNew();
            double lastMs = sw.Elapsed.TotalMilliseconds;
            int pushTick = 0;

            while (!ct.IsCancellationRequested)
            {
                try { await Task.Delay(50, ct); }
                catch (TaskCanceledException) { break; }

                if (!_autoTick) { lastMs = sw.Elapsed.TotalMilliseconds; }
                else
                {
                    double now = sw.Elapsed.TotalMilliseconds;
                    double dt = (now - lastMs) / 1000.0 * _simSpeed;
                    lastMs = now;
                    if (dt > 0 && dt < 10)
                        lock (_lock) { _controller.Update(dt); }
                }

                if (++pushTick >= 5)
                {
                    pushTick = 0;
                    await BroadcastState();
                }
            }
        }

        private static async Task BroadcastState()
        {
            if (_wsClients.IsEmpty) return;

            string json;
            lock (_lock)
            {
                var eventsArr = _engine.EventLog.TakeLast(200)
                    .Select(e => new { t = e.Timestamp, cat = e.Category, msg = e.Message });
                var payload = new { type = "state", autoTick = _autoTick, simSpeed = _simSpeed, data = BuildFullState(_engine), events = eventsArr };
                json = JsonSerializer.Serialize(payload, JsonOpts);
            }

            var bytes = Encoding.UTF8.GetBytes(json);
            var segment = new ArraySegment<byte>(bytes);
            var dead = new List<string>();

            foreach (var (id, ws) in _wsClients)
            {
                try
                {
                    if (ws.State == WebSocketState.Open)
                        await ws.SendAsync(segment, WebSocketMessageType.Text, true, CancellationToken.None);
                    else
                        dead.Add(id);
                }
                catch { dead.Add(id); }
            }

            foreach (var id in dead)
                _wsClients.TryRemove(id, out _);
        }

        private static void MapRoutes(WebApplication app)
        {
            app.Map("/ws", async context =>
            {
                if (!context.WebSockets.IsWebSocketRequest)
                {
                    context.Response.StatusCode = 400;
                    return;
                }
                var ws = await context.WebSockets.AcceptWebSocketAsync();
                var id = Interlocked.Increment(ref _wsNextId).ToString();
                _wsClients[id] = ws;
                Console.WriteLine($"[WS] client {id} connected  (total: {_wsClients.Count})");

                var buf = new byte[256];
                while (ws.State == WebSocketState.Open)
                {
                    try
                    {
                        var result = await ws.ReceiveAsync(new ArraySegment<byte>(buf), CancellationToken.None);
                        if (result.MessageType == WebSocketMessageType.Close) break;
                    }
                    catch { break; }
                }

                _wsClients.TryRemove(id, out _);
                Console.WriteLine($"[WS] client {id} disconnected (total: {_wsClients.Count})");
                try { await ws.CloseAsync(WebSocketCloseStatus.NormalClosure, "bye", CancellationToken.None); } catch { }
            });

            app.MapGet("/api/state", () =>
            {
                lock (_lock) { return Results.Json(BuildFullState(_engine), JsonOpts); }
            });
            app.MapGet("/api/diagnostics", () =>
            {
                lock (_lock) { return Results.Json(BuildDiagnostics(_engine), JsonOpts); }
            });
            app.MapGet("/api/reactivity", () =>
            {
                lock (_lock) { return Results.Json(_engine.GetReactivityBreakdownPCM(), JsonOpts); }
            });
            app.MapGet("/api/events", () =>
            {
                lock (_lock)
                {
                    var evts = _engine.EventLog.TakeLast(200).Select(e => new
                    {
                        t = e.Timestamp, cat = e.Category, msg = e.Message
                    });
                    return Results.Json(evts, JsonOpts);
                }
            });

            app.MapPost("/api/tick", (TickRequest req) =>
            {
                lock (_lock) { _controller.Update(req.Dt > 0 ? req.Dt : 1.0); }
                return Results.Ok();
            });
            app.MapPost("/api/autotick", (AutoTickRequest req) =>
            {
                _autoTick = req.Enabled;
                if (req.Speed > 0) _simSpeed = Math.Clamp(req.Speed, 0.1, 100.0);
                return Results.Json(new { autoTick = _autoTick, simSpeed = _simSpeed });
            });

            app.MapPost("/api/init/cold",  () => { lock (_lock) _engine.InitialiseColdShutdown(); _autoTick = false; return Results.Ok(); });
            app.MapPost("/api/init/hzp",   () => { lock (_lock) _engine.InitialiseHotZeroPower();  _autoTick = false; return Results.Ok(); });
            app.MapPost("/api/init/full",  () => { lock (_lock) _engine.InitialiseFullPower();     _autoTick = false; return Results.Ok(); });

            app.MapPost("/api/rod/bank", (RodBankCmd cmd) =>
            { lock (_lock) _engine.CommandRodBank(cmd.Bank, cmd.Target); return Results.Ok(); });
            app.MapPost("/api/rod/speed", (RodSpeedCmd cmd) =>
            { lock (_lock) _engine.CommandRodSpeed(cmd.Speed); return Results.Ok(); });
            app.MapPost("/api/rod/withdraw", (RodMoveCmd cmd) =>
            { lock (_lock) _engine.WithdrawRods(cmd.Bank, cmd.Steps); return Results.Ok(); });
            app.MapPost("/api/rod/insert", (RodMoveCmd cmd) =>
            { lock (_lock) _engine.InsertRods(cmd.Bank, cmd.Steps); return Results.Ok(); });
            app.MapPost("/api/rod/auto", (ToggleCmd cmd) =>
            { lock (_lock) _engine.SetAutoRodControl(cmd.Enabled); return Results.Ok(); });

            app.MapPost("/api/boron", (BoronCmd cmd) =>
            { lock (_lock) _engine.CommandBoronTarget(cmd.Ppm); return Results.Ok(); });
            app.MapPost("/api/boron/emergency", () =>
            { lock (_lock) _engine.EmergencyBoration(); return Results.Ok(); });

            app.MapPost("/api/trip",       () => { lock (_lock) _engine.ManualTrip();  return Results.Ok(); });
            app.MapPost("/api/trip/reset", () => { lock (_lock) _engine.ResetTrip();   return Results.Ok(); });
            app.MapPost("/api/rps", (ToggleCmd cmd) =>
            { lock (_lock) _engine.SetProtectionSystem(cmd.Enabled); return Results.Ok(); });
            app.MapPost("/api/rps/bypass", (BypassCmd cmd) =>
            {
                if (Enum.TryParse<TripSignal>(cmd.Signal, true, out var sig))
                    lock (_lock) _engine.SetTripBypassed(sig, cmd.Bypassed);
                return Results.Ok();
            });

            app.MapPost("/api/rcp", (RcpCmd cmd) =>
            { lock (_lock) _engine.CommandRcp(cmd.Pump, cmd.Start); return Results.Ok(); });

            app.MapPost("/api/pressurizer/heaters", (HeaterCmd cmd) =>
            { lock (_lock) _engine.CommandPressuriserHeaters(cmd.Kw); return Results.Ok(); });
            app.MapPost("/api/pressurizer/spray", (ToggleCmd cmd) =>
            { lock (_lock) _engine.CommandPressuriserSpray(cmd.Enabled); return Results.Ok(); });

            app.MapPost("/api/turbine/online", (ToggleCmd cmd) =>
            { lock (_lock) _engine.CommandTurbineOnline(cmd.Enabled); return Results.Ok(); });
            app.MapPost("/api/turbine/load", (TurbineLoadCmd cmd) =>
            { lock (_lock) _engine.CommandTurbineLoad(cmd.Mwe); return Results.Ok(); });

            app.MapPost("/api/sg/feedtemp", (FeedTempCmd cmd) =>
            { lock (_lock) _engine.SetFeedwaterTemperature(cmd.TempC); return Results.Ok(); });
            app.MapPost("/api/sg/fouling", (FoulingCmd cmd) =>
            { lock (_lock) _engine.SetSgFouling(cmd.Factor); return Results.Ok(); });

            app.MapPost("/api/eccs/si", () =>
            { lock (_lock) _engine.InitiateSafetyInjection(); return Results.Ok(); });
            app.MapPost("/api/eccs/accumulators", (ToggleCmd cmd) =>
            { lock (_lock) _engine.IsolateAccumulators(!cmd.Enabled); return Results.Ok(); });

            app.MapPost("/api/power/offsite", (ToggleCmd cmd) =>
            { lock (_lock) _engine.SetOffSitePower(cmd.Enabled); return Results.Ok(); });
            app.MapPost("/api/power/diesel", (ToggleCmd cmd) =>
            { lock (_lock) _engine.CommandDieselGenerator(cmd.Enabled); return Results.Ok(); });

            app.MapPost("/api/bias", (BiasCmd cmd) =>
            { lock (_lock) _engine.SetManualReactivityBias(cmd.Dkk); return Results.Ok(); });
            app.MapPost("/api/peaking", (PeakingCmd cmd) =>
            { lock (_lock) _engine.SetPeakingFactor(cmd.Fq); return Results.Ok(); });
            app.MapPost("/api/mode", (ModeCmd cmd) =>
            {
                if (Enum.TryParse<ReactorMode>(cmd.Mode, true, out var m))
                    lock (_lock) _engine.SetMode(m);
                return Results.Ok();
            });

            // ---- auto-control parameters ----
            app.MapPost("/api/rod/scramoverride",    (ToggleCmd c) => { lock (_lock) _engine.SetRodScramOverride(c.Enabled);    return Results.Ok(); });
            app.MapPost("/api/rod/autogain",         (ValueCmd  c) => { lock (_lock) _engine.SetRodAutoGain(c.Value);           return Results.Ok(); });
            app.MapPost("/api/rod/autodeadband",     (ValueCmd  c) => { lock (_lock) _engine.SetRodAutoDeadband(c.Value);       return Results.Ok(); });
            app.MapPost("/api/pressurizer/heaterauto",  (ToggleCmd c) => { lock (_lock) _engine.SetPrzHeaterAuto(c.Enabled);   return Results.Ok(); });
            app.MapPost("/api/pressurizer/sprayauto",   (ToggleCmd c) => { lock (_lock) _engine.SetPrzSprayAuto(c.Enabled);    return Results.Ok(); });
            app.MapPost("/api/pressurizer/setpoint",    (ValueCmd  c) => { lock (_lock) _engine.SetPrzPressureSetpoint(c.Value); return Results.Ok(); });
            app.MapPost("/api/pressurizer/autogain",    (ValueCmd  c) => { lock (_lock) _engine.SetPrzAutoGain(c.Value);       return Results.Ok(); });
            app.MapPost("/api/pressurizer/autodeadband",(ValueCmd  c) => { lock (_lock) _engine.SetPrzAutoDeadband(c.Value);   return Results.Ok(); });
            app.MapPost("/api/turbine/governorauto",    (ToggleCmd c) => { lock (_lock) _engine.SetTurbineGovernorAuto(c.Enabled); return Results.Ok(); });
            app.MapPost("/api/sg/fwauto",               (ToggleCmd c) => { lock (_lock) _engine.SetFeedwaterAuto(c.Enabled);   return Results.Ok(); });
            app.MapPost("/api/sg/fwtripoverride",       (ToggleCmd c) => { lock (_lock) _engine.SetFeedwaterTripOverride(c.Enabled); return Results.Ok(); });
            app.MapPost("/api/sg/fwlevelsetpoint",      (ValueCmd  c) => { lock (_lock) _engine.SetFwLevelSetpoint(c.Value);   return Results.Ok(); });
            app.MapPost("/api/sg/fwlevelgain",          (ValueCmd  c) => { lock (_lock) _engine.SetFwLevelGain(c.Value);       return Results.Ok(); });
            app.MapPost("/api/sg/fwflowgain",           (ValueCmd  c) => { lock (_lock) _engine.SetFwFlowGain(c.Value);        return Results.Ok(); });

            app.MapGet("/api/snapshot", () =>
            {
                byte[] data;
                lock (_lock) data = _engine.SerialiseState();
                return Results.Bytes(data, "application/octet-stream", "reactor_state.bin");
            });
            app.MapPost("/api/restore", async (HttpRequest req) =>
            {
                using var ms = new MemoryStream();
                await req.Body.CopyToAsync(ms);
                lock (_lock) _engine.DeserialiseState(ms.ToArray());
                return Results.Ok();
            });
        }

        private static object BuildFullState(ReactorEngine engine)
        {
            ref readonly var s = ref engine.State;
            return new
            {
                sim = new { time = s.SimulationTimeSeconds, tick = s.TickCount, dt = s.TimeStepSeconds },
                mode = s.Mode.ToString(),
                scram = s.Scram.ToString(),
                trips = s.ActiveTrips.ToString(),
                rpsEnabled = s.ProtectionSystemEnabled == 1,
                tripBypassFlags = (uint)s.TripBypassFlags,
                neutronics = new {
                    neutronPop = s.NeutronPopulation,
                    thermalPowerFrac = s.ThermalPowerFraction + s.DecayHeatFraction,
                    thermalPowerMW = s.ThermalPowerMW, decayHeatFrac = s.DecayHeatFraction,
                    decayHeatMW = s.DecayHeatMW, totalPowerMW = s.TotalPowerMW,
                    startupRateDPM = s.StartupRateDPM },
                reactivity = new {
                    total = s.RhoTotal * 1e5, controlRods = s.RhoControlRods * 1e5,
                    doppler = s.RhoDoppler * 1e5, moderator = s.RhoModerator * 1e5,
                    boron = s.RhoBoron * 1e5, xenon = s.RhoXenon * 1e5,
                    samarium = s.RhoSamarium * 1e5, voidEff = s.RhoVoid * 1e5,
                    bias = s.RhoManualBias * 1e5 },
                poisons = new {
                    iodine135 = s.Iodine135, xenon135 = s.Xenon135, xenonEq = s.XenonEquilibrium,
                    promethium149 = s.Promethium149, samarium149 = s.Samarium149,
                    samariumEq = s.SamariumEquilibrium },
                fuel = new {
                    centrelineTemp = s.FuelCentrelineTemp, averageTemp = s.FuelAverageTemp,
                    surfaceTemp = s.FuelSurfaceTemp, cladInnerTemp = s.CladInnerTemp,
                    cladOuterTemp = s.CladOuterTemp, coolantBulkTemp = s.CoolantBulkTemp,
                    linearHeatRate = s.LinearHeatRate, peakingFactor = s.PeakingFactor,
                    cladOxide = s.CladOxideThickness, cladHoopStress = s.CladHoopStress,
                    fuelMeltFrac = s.FuelMeltFraction, dnbr = s.DNBRatio,
                    burnup = s.FuelBurnupMWdPerTonne },
                primary = new {
                    pressureMPa = s.PrimaryPressureMPa, pressurePsi = s.PrimaryPressurePsi,
                    flowKgPerS = s.PrimaryFlowRateKgPerS, flowFraction = s.PrimaryFlowFraction,
                    inletTemp = s.CoreInletTempC, outletTemp = s.CoreOutletTempC,
                    deltaT = s.CoreDeltaT, tavg = s.TavgC, density = s.CoolantDensityKgPerM3,
                    voidFraction = s.CoolantVoidFraction, subcoolingMargin = s.SubcoolingMarginC },
                pressurizer = new {
                    pressureMPa = s.PressuriserPressureMPa, tempC = s.PressuriserTempC,
                    level = s.PressuriserLevelFraction, heaterKW = s.PressuriserHeaterPowerKW,
                    sprayOn = s.PressuriserSprayOn == 1, sprayFlow = s.PressuriserSprayFlowKgPerS,
                    porvOpen = s.PorvOpen == 1, safetyValveOpen = s.SafetyValveOpen == 1,
                    heaterAutoEnabled = s.PrzHeaterAutoEnabled == 1,
                    sprayAutoEnabled  = s.PrzSprayAutoEnabled == 1,
                    setpointMPa = s.PrzPressureSetpointMPa,
                    autoGain = s.PrzAutoGain, autoDeadbandMPa = s.PrzAutoDeadbandMPa },
                rcp = new {
                    speeds = new[] { s.RcpSpeed.V0, s.RcpSpeed.V1, s.RcpSpeed.V2, s.RcpSpeed.V3 },
                    states = new[] {
                        ((PumpState)(int)s.RcpState.V0).ToString(),
                        ((PumpState)(int)s.RcpState.V1).ToString(),
                        ((PumpState)(int)s.RcpState.V2).ToString(),
                        ((PumpState)(int)s.RcpState.V3).ToString() },
                    flows = new[] { s.RcpFlowFraction.V0, s.RcpFlowFraction.V1,
                                    s.RcpFlowFraction.V2, s.RcpFlowFraction.V3 },
                    totalFlow = s.TotalRcpFlowFraction },
                rods = new {
                    positions = new[] { s.RodBankPosition.V0, s.RodBankPosition.V1,
                                        s.RodBankPosition.V2, s.RodBankPosition.V3 },
                    targets = new[] { s.RodBankTargetPosition.V0, s.RodBankTargetPosition.V1,
                                      s.RodBankTargetPosition.V2, s.RodBankTargetPosition.V3 },
                    speed = s.RodSpeedStepsPerMin, autoMode = s.RodControlAutoMode == 1,
                    tavgSetpoint = s.TavgProgramSetpointC,
                    autoGain = s.RodAutoGainStepsPerMinPerC, autoDeadband = s.RodAutoDeadbandC,
                    scramOverride = s.RodScramOverride == 1 },
                boron = new { ppm = s.BoronConcentrationPPM, target = s.BoronConcentrationTarget },
                sg = new {
                    primaryInletTemp = s.SgPrimaryInletTempC, primaryOutletTemp = s.SgPrimaryOutletTempC,
                    secondaryPressure = s.SgSecondaryPressureMPa, steamTemp = s.SgSteamTempC,
                    feedwaterTemp = s.SgFeedwaterTempC, level = s.SgLevelFraction,
                    steamFlow = s.SgSteamFlowKgPerS, feedFlow = s.SgFeedwaterFlowKgPerS,
                    fouling = s.SgTubeFoulingFactor,
                    fwAutoEnabled = s.FeedwaterAutoEnabled == 1,
                    fwTripOverride = s.FeedwaterTripOverride == 1,
                    fwLevelSetpoint = s.FwLevelSetpoint,
                    fwLevelGain = s.FwLevelGain, fwFlowGain = s.FwFlowGain,
                    fwTimeConstantS = s.FwTimeConstantS },
                turbine = new {
                    online = s.TurbineOnline == 1, tripped = s.TurbineTripFlag == 1,
                    loadMW = s.TurbineLoadMW, speedRPM = s.TurbineSpeedRPM,
                    speedFrac = s.TurbineSpeedFraction, throttle = s.TurbineThrottlePosition,
                    powerTarget = s.TurbinePowerTarget, condenserTemp = s.CondenserTempC,
                    condenserVacuum = s.CondenserVacuumKPa,
                    governorAutoEnabled = s.TurbineGovernorAutoEnabled == 1 },
                electrical = new {
                    generatorMW = s.GeneratorOutputMW, housekeepingMW = s.HousekeepingLoadMW,
                    netMW = s.NetElectricalMW, offSitePower = s.OffSitePowerAvailable == 1,
                    dieselRunning = s.DieselGeneratorRunning == 1 },
                eccs = new {
                    status = s.EccsStatus.ToString(), accumulatorPressure = s.AccumulatorPressureMPa,
                    accumulatorVolume = s.AccumulatorWaterVolumeM3,
                    accumulatorIsolated = s.AccumulatorIsolated == 1,
                    hpiFlow = s.HpiFlowKgPerS, lpiFlow = s.LpiFlowKgPerS },
                containment = new { pressureKPa = s.ContainmentPressureKPa, tempC = s.ContainmentTempC,
                    h2FractionPct = s.ContainmentH2FractionPct },
                coreDamage = new {
                    state = s.CoreDamageState.ToString(),
                    damageFraction = s.CoreDamageFraction,
                    zrOxidizedFraction = s.ZirconiumOxidizedFraction,
                    exothermicHeatMW = s.CoreExothermicHeatMW,
                    hydrogenKg = s.HydrogenGenerationKg,
                    fpReleaseFraction = s.FissionProductReleaseFraction,
                    meltdown = s.CoreDamageState >= CoreDamageState.Meltdown }
            };
        }

        private static object BuildDiagnostics(ReactorEngine engine)
        {
            var d = engine.GetDiagnostics();
            return new {
                powerPercent = d.PowerPercent, tavg = d.TavgC, pressure = d.PressureMPa,
                boron = d.BoronPPM, bankD = d.BankDSteps, xenonFrac = d.XenonFraction,
                startupRate = d.StartupRate, dnbr = d.DNBRatio, generatorMW = d.GeneratorMW,
                mode = d.Mode.ToString(), trips = d.Trips.ToString()
            };
        }
    }
}
