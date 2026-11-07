using System.Diagnostics;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// Keeps KSA's auto-burn aimed for the n-body trajectory. KSA fixes a burn's
// direction once, when it loads the burn, but the n-body path moves on (a
// fraction of a degree misses the Moon by thousands of km). Until the flight
// computer starts aligning, KSA's own target is recomputed every quarter
// second from the burn's patch: Principia's state at the burn time
// (BurnAnchor), else KSA's conic.
internal static class BurnTargetRefresher
{
    private static readonly long Interval = Stopwatch.Frequency / 4;
    private static long _next;

    // Re-aims are logged from this size, at most every LogInterval.
    private const double LoggedDegrees = 0.1;
    private static readonly long LogInterval = 10 * Stopwatch.Frequency;
    private static long _nextLog;

    public static long Refreshes;
    public static double LastChangeDegrees;

    private static BurnTarget? _flying, _finished;
    private static double _flyingTargetDeltaV;

    // Logs each burn KSA flies: its ignition, and the Δv and orbit at the end.
    private static void Monitor(Vehicle vehicle)
    {
        BurnTarget? target = vehicle.FlightComputer.Burn;
        if (_flying == null && target is { StartedAt: { } started } && target != _finished)
        {
            _flying = target;
            _flyingTargetDeltaV = target.DeltaVTargetCci.Length();
            Log.Info($"[burns] {vehicle.Id}: ignition {GameTime.Seconds(target.ImpulsiveInstant, started):F1} s from " +
                     $"the burn time (planned {GameTime.Seconds(target.ImpulsiveInstant, target.IgnitionTime):F1} s), " +
                     $"Δv target {_flyingTargetDeltaV:F2} m/s, mode {vehicle.FlightComputer.BurnMode}");
        }
        else if (_flying != null &&
                 (target != _flying || !_flying.StartedAt.HasValue || vehicle.FlightComputer.AutoBurnCompleted))
        {
            _finished = _flying;
            Orbit o = vehicle.Orbit;
            Log.Info($"[burns] {vehicle.Id}: burn ended, Δv flown {_flying.DeltaVAccumCci.Length():F2} of " +
                     $"{_flyingTargetDeltaV:F2} m/s ({_flying.DeltaVToGoCci.Length():F2} m/s left), " +
                     $"now around {(o.Parent as IObjectId)?.Id}: pe {o.PeriapsisAltitude / 1000:F1} km, " +
                     $"ap {o.ApoapsisAltitude / 1000:F1} km");
            _flying = null;
        }
    }

    // Main thread, before the frame's jobs, which share the target.
    public static void Tick()
    {
        try
        {
            if (Program.ControlledVehicle is { } controlled)
            {
                Monitor(controlled);
            }
            Refresh();
        }
        catch (Exception e)
        {
            Log.Warn($"[burns] {e.Message}");
        }
    }

    private static void Refresh()
    {
        long now = Stopwatch.GetTimestamp();
        if (now < _next || PrincipiaSystem.Current is not { Driving: true } ||
            Program.ControlledVehicle is not { } vehicle)
        {
            return;
        }
        _next = now + Interval;
        FlightComputer fc = vehicle.FlightComputer;
        BurnTarget? target = fc.Burn;
        if (target == null || target.StartedAt.HasValue || target.FixedDurationSeconds.HasValue ||
            Universe.GetElapsedTime() >= target.AlignmentStartTime)
        {
            return;
        }
        Burn? burn = vehicle.BurnPlan.FindFirstExecutableBurn();
        if (burn == null || !burn.HasDeltaV || burn.ImpulseTime != target.ImpulsiveInstant || burn.IsInterstellar)
        {
            return;  // A fresh burn has no Δv until it is dragged.
        }
        float3 before = target.DeltaVTargetCci;
        UniverseTime? plannedIgnition = target.PlannedIgnition;
        float? throttle = target.Throttle;
        target.UpdateFromBurn(burn);
        target.PlannedIgnition = plannedIgnition;
        target.Throttle = throttle;
        Refreshes++;

        float3 after = target.DeltaVTargetCci;
        double norms = (double)before.Length() * after.Length();
        double change = norms > 0
            ? Math.Acos(Math.Clamp(float3.Dot(before, after) / norms, -1, 1)) * 180 / Math.PI
            : 0;
        LastChangeDegrees = change;
        if (change >= LoggedDegrees && now >= _nextLog)
        {
            _nextLog = now + LogInterval;
            Log.Info($"[burns] {vehicle.Id}: burn target re-aimed by {change:F3}° for the n-body trajectory " +
                     $"(burn in {GameTime.Seconds(Universe.GetElapsedTime(), burn.ImpulseTime):F0} s)");
        }
    }
}
