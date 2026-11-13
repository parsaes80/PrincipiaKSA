using System.Diagnostics;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// KSA's planned burns on Principia's path: each burn's patch and body from
// the n-body state (BurnAnchor), burns KSA's conics don't reach kept
// (BurnPatchRescue), and the flight computer's target re-aimed
// (BurnTargetRefresher).

// KSA's burn plans, as the mod reads them.
internal static class BurnPlans
{
    // Every burn of the plan, in its order.
    public static IEnumerable<Burn> All(this BurnPlan plan)
    {
        for (int i = 0; i < plan.BurnCount; i++)
        {
            if (plan.TryGetBurn(i, out Burn? burn) && burn != null)
            {
                yield return burn;
            }
        }
    }

    // When the vessel makes a burn. A departure planned on the parent's orbit
    // (the transfer planner's, Burn.ParentDepartureBurn) is a vessel burn at
    // ParentDepartureTime, whose Δv KSA computes (CalculateTransferBurn); its
    // Time and Patch are the parent's.
    public static UniverseTime VesselTime(this Burn burn) =>
        burn.ParentDepartureBurn ? burn.ParentDepartureTime : burn.Time;

    // Whether a burn with Δv other than `except` comes before `time` (or at
    // it too, `inclusive`): where the flight plan, not the prediction, is
    // the vessel's path.
    public static bool AnyWithDeltaVBefore(this BurnPlan plan, UniverseTime time, bool inclusive,
                                           Burn? except = null) =>
        plan.All().Any(b => b != except && b.HasDeltaV && !b.IsInterstellar &&
                            (inclusive ? b.VesselTime() <= time : b.VesselTime() < time));
}

// Where a maneuver node's burn is on Principia's path, and the body it is
// about. KSA plans a burn's Δv in prograde/normal/radial about the parent of
// its patch at the burn time, a patch of its conics, which after an encounter
// (a close one above all) no longer follow the n-body path: a node there was
// about whatever body the conics had then. KSA flies the burn from the
// vessel's real orbit, about the body whose sphere of influence the vessel is
// really in. So the anchor applies KSA's own sphere-of-influence rules
// (Spheres) to the n-body position at the burn time, and KSA gets a patch
// with the n-body state there (BurnPatchAnchor): the node's arrows, the
// flight plan, the drag step, KSA's burn editor and the flight computer's
// target all have the body and state the burn will be flown from.

internal static class BurnAnchor
{
    // The n-body state at the burn time relative to Parent (Cce axes), from
    // the path whose samples are Source, and the patch handed to KSA.
    public sealed record Anchor(double Time, QP[] Source, Astronomical Parent, int ParentIndex,
                                double3 PositionCce, double3 VelocityCce, PatchedConic Patch);

    // Kept while the burn time and the path are unchanged.
    private static readonly ConditionalWeakTable<Burn, Anchor> Anchors = new();

    // The anchor last made for the burn, without making one.
    public static Anchor? Cached(Burn burn) => Anchors.TryGetValue(burn, out Anchor? anchor) ? anchor : null;

    // A burn still ahead on Principia's path; null where there is none yet
    // (KSA's patch stands in) or for the transfer planner's departures.
    public static Anchor? Get(Burn burn)
    {
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        double now = GameTime.Now;
        // Being flown, or past: KSA's, from the vessel's orbit.
        if (!(t > now) || burn.ParentDepartureBurn || burn.IsInterstellar || burn.Vehicle is not { } vehicle ||
            vehicle.Orbit.Parent is not Astronomical vesselParent || PrincipiaSystem.Current is not { } system)
        {
            return null;
        }
        bool hasCached = Anchors.TryGetValue(burn, out Anchor? cached) && cached!.Time == t;
        if (Source(burn, vehicle, t) is not { StateTimes.Length: > 0 } path)
        {
            // None just now (a prediction being redone, or failed): the last
            // anchor, while the burn hasn't moved.
            return hasCached ? cached : null;
        }
        if (hasCached && ReferenceEquals(cached!.Source, path.States))
        {
            return cached;
        }
        int k = Array.BinarySearch(path.StateTimes, t);
        k = k >= 0 ? k : ~k - 1;
        if (k < 0 || !system.Enter())
        {
            return null;
        }
        try
        {
            Ephemeris ephemeris = system.Ephemeris;
            QP state = path.StateTimes[k] < t
                ? ephemeris.FlowMassless(path.States[k], path.StateTimes[k], t)
                : path.States[k];
            if (system.IndexOf(vesselParent) is not int from)
            {
                return null;
            }
            int index = system.Spheres.Of(ephemeris, from, state.Q.ToDouble3(), t);
            Astronomical parent = system.Bodies[index];
            if (parent is not IParentBody parentBody)
            {
                return null;
            }
            QP centre = ephemeris.State(index, t);
            double3 q = state.Q.ToDouble3() - centre.Q.ToDouble3();
            double3 v = state.P.ToDouble3() - centre.P.ToDouble3();
            doubleQuat cce2Cci = parentBody.GetCci2Cce().Inverse();
            Orbit orbit = Orbit.CreateFromStateCci(parentBody, burn.Time, q.Transform(cce2Cci), v.Transform(cce2Cci),
                                                   vehicle.OrbitColor);
            // Its stretch of the path, in the body's sphere of influence, as
            // far as the path goes: where KSA's ±orbit buttons may move it.
            Stretch stretch = path.Line.Stretches[path.Line.StretchAt(t)];
            double from0 = double.IsFinite(stretch.From) ? stretch.From : path.Start;
            UniverseTime start = GameTime.FromSecondsSinceJ2000(Math.Min(t, Math.Max(Math.Max(now, path.Start), from0)));
            UniverseTime end = stretch.Closed && double.IsFinite(stretch.To)
                ? GameTime.FromSecondsSinceJ2000(Math.Max(t, stretch.To))
                : UniverseTime.EndOfTime;
            var patch = new PatchedConic(start, end, PatchTransition.Initial, PatchTransition.Final, orbit,
                                         vehicle.Hash);
            var anchor = new Anchor(t, path.States, parent, index, q, v, patch);
            Anchors.AddOrUpdate(burn, anchor);
            return anchor;
        }
        finally
        {
            system.Exit();
        }
    }

    // The n-body path before the burn: its node track (or the prediction, for
    // the first burn), else, until a node just created or moved gets its
    // track, the flight plan where it isn't burning.
    private static BurnNode.NodePath? Source(Burn burn, Vehicle vehicle, double t)
    {
        if (BurnNode.Path(burn, window: false) is { } path)
        {
            return path;
        }
        if (Predictor.PlanFor(vehicle) is { StateTimes.Length: > 1 } plan &&
            plan.StateTimes[0] <= t && t <= plan.StateTimes[^1] && !Burning(plan, t))
        {
            return new BurnNode.NodePath(plan.Line, plan.Times[0], double.NaN, plan.StateTimes, plan.States);
        }
        return null;
    }

    private static bool Burning(Prediction plan, double t) =>
        plan.BurnArcs.Any(a => plan.Times[a.Start] - 1 <= t && t <= plan.Times[a.End] + 1);

    // Why a burn has no anchor, for the log.
    public static string Explain(Burn burn)
    {
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        double now = GameTime.Now;
        string H(double x) => double.IsFinite(x) ? $"{(x - now) / 3600:F1}" : "∞";
        string Span(double[]? times) => times is { Length: > 0 } ? $"{H(times[0])}–{H(times[^1])} h" : "none";
        if (!(t > now) || burn.ParentDepartureBurn || burn.IsInterstellar)
        {
            return "past, a departure or interstellar";
        }
        Vehicle vehicle = burn.Vehicle;
        NodeTrack? track = Predictor.NodesFor(vehicle) is { } nodes &&
                           nodes.ByBurn.TryGetValue(burn, out NodeTrack? found) ? found : null;
        Prediction? plan = Predictor.PlanFor(vehicle);
        Prediction? coast = Predictor.PredictionFor(vehicle);
        string anchor = Anchors.TryGetValue(burn, out Anchor? last)
            ? $"last anchor at {H(last.Time)} h about {last.Parent.Id}"
            : "never anchored";
        return $"conics hidden {KsaConics.Hidden(vehicle)}, its track {Span(track?.StateTimes)} " +
               $"(tracks for {Predictor.LatestNodes?.ByBurn.Count ?? 0} nodes), " +
               $"flight plan {Span(plan?.StateTimes)}{(plan != null && Burning(plan, t) ? " (burning then)" : "")}, " +
               $"prediction {Span(coast?.StateTimes)}; {anchor}; showing prediction {Settings.ShowPrediction} " +
               $"({Settings.PredictionSteps:N0} steps), flight plan {Settings.ShowFlightPlan} " +
               $"({Settings.PlanSteps:N0} steps per coast); last prediction error: {Predictor.LastError ?? "none"}";
    }

    // Logged when KSA's conics and Principia's path put a burn about different
    // bodies, once per change.
    private static readonly ConditionalWeakTable<Burn, StrongBox<(IParentBody?, Astronomical)>> Logged = new();

    public static void Compare(Burn burn, PatchedConic? ksa, Anchor anchor)
    {
        IParentBody? theirs = ksa?.Orbit.Parent;
        if (theirs == anchor.Parent)
        {
            return;
        }
        if (Logged.TryGetValue(burn, out var last) && last.Value.Item1 == theirs && last.Value.Item2 == anchor.Parent)
        {
            return;
        }
        Logged.AddOrUpdate(burn, new StrongBox<(IParentBody?, Astronomical)>((theirs, anchor.Parent)));
        double hours = (anchor.Time - GameTime.Now) / 3600;
        Log.Info($"[node] {burn.Vehicle.Id}: the burn in {hours:F1} h is about {anchor.Parent.Id}, where Principia's " +
                 $"path is then; KSA's conics have the vessel about {(theirs as IObjectId)?.Id ?? "nothing"}");
    }
}

// A burn's patch (Burn.Patch, its frame and state at the burn time) from
// Principia's path. Not the getter, small enough to be inlined.
[HarmonyPatch(typeof(BurnPlan), nameof(BurnPlan.TryGetBurnPatch))]
internal static class BurnPatchAnchor
{
    [HarmonyPostfix]
    private static void Postfix(Burn burn, ref PatchedConic? __result)
    {
        using var profiled = FrameProfiler.BurnAnchor.Measure();
        try
        {
            if (BurnAnchor.Get(burn) is { } anchor)
            {
                BurnAnchor.Compare(burn, __result, anchor);
                __result = anchor.Patch;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[node] burn patch from Principia's path", e);
        }
    }
}

// KSA deletes a burn ("No patch found for burn") when none of its conic plans
// reaches the burn's time: the previous burns' plans, newest first, then the
// vessel's. After an encounter its conics are not the n-body path, and a
// later node can lie past where they end (a plan stops at the next burn, and
// a burn without Δv has none). The vessel's plan is KSA's last resort: where
// it fails too, KSA gets the burn's patch on Principia's path, so a burn is
// deleted only where Principia's path doesn't reach it either.
[HarmonyPatch(typeof(BurnPlan), nameof(BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly))]
internal static class BurnPatchRescue
{
    // The plans being computed on this thread: the controlled vessel's own.
    [ThreadStatic] private static BurnPlan? _burns;
    [ThreadStatic] private static FlightPlan? _vesselPlan;
    [ThreadStatic] private static List<FlightPlan>? _newPlans;

    private static long _nextDump;

    private static readonly ConditionalWeakTable<Burn, object> Logged = new();

    [HarmonyPrefix]
    private static void Prefix(BurnPlan __instance, FlightPlan vehicleFlightPlan, List<FlightPlan> newFlightPlans)
    {
        bool own = Program.ControlledVehicle is { } vehicle && vehicle.BurnPlan == __instance &&
                   KsaConics.Hidden(vehicle);
        _burns = own ? __instance : null;
        _vesselPlan = own ? vehicleFlightPlan : null;
        _newPlans = own ? newFlightPlans : null;
    }

    [HarmonyFinalizer]
    private static void Finalizer() => (_burns, _vesselPlan, _newPlans) = (null, null, null);

    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        return Patching.ReplaceCalls(instructions, "[node] BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly " +
                                     "no longer looks up patches; KSA may delete burns its conics don't reach",
            (AccessTools.Method(typeof(FlightPlan), nameof(FlightPlan.TryFindPatch)),
             AccessTools.Method(typeof(BurnPatchRescue), nameof(TryFindPatch))));
    }

    // FlightPlan.TryFindPatch, with the burn's patch on Principia's path where
    // KSA's last resort finds none.
    public static PatchedConic? TryFindPatch(FlightPlan plan, UniverseTime time)
    {
        PatchedConic? found = plan.TryFindPatch(time);
        if (found != null || _burns is not { } burns || !ReferenceEquals(plan, _vesselPlan))
        {
            return found;
        }
        try
        {
            Burn? unanchored = null;
            foreach (Burn burn in burns.All())
            {
                if (burn.ImpulseTime != time)
                {
                    continue;
                }
                unanchored = burn;
                if (BurnAnchor.Get(burn) is { } anchor)
                {
                    if (!Logged.TryGetValue(burn, out _))
                    {
                        Logged.AddOrUpdate(burn, new object());
                        double hours = (anchor.Time - GameTime.Now) / 3600;
                        Log.Info($"[node] {burn.Vehicle.Id}: KSA's conics don't reach the burn in {hours:F1} h; " +
                                 "it is held on Principia's path");
                    }
                    // A copy: KSA ends the patch it is given at the burn.
                    return new PatchedConic(anchor.Patch);
                }
            }
            Dump(burns, time, unanchored);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[node] holding a burn KSA's conics don't reach", e);
        }
        return null;
    }

    // KSA is about to delete a burn: what its plans and Principia's lines
    // cover, at most every 5 s.
    private static void Dump(BurnPlan burns, UniverseTime time, Burn? burn)
    {
        long tick = System.Diagnostics.Stopwatch.GetTimestamp();
        if (tick < Interlocked.Read(ref _nextDump))
        {
            return;
        }
        Interlocked.Exchange(ref _nextDump, tick + 5 * System.Diagnostics.Stopwatch.Frequency);
        double now = GameTime.Now;
        string H(UniverseTime x) =>
            GameTime.ToSecondsSinceJ2000(x) is var s && double.IsFinite(s) ? $"{(s - now) / 3600:F1}" : "∞";
        string Patches(FlightPlan? plan) => plan == null || plan.Patches.Count == 0
            ? "no patches"
            : string.Join(", ", plan.Patches.Select(p =>
                $"{(p.PrimaryBody as IObjectId)?.Id} {H(p.StartTime)}–{H(p.EndTime)} h {p.EndTransition}"));
        var lines = new List<string>
        {
            $"[node] KSA has no patch for the burn in {H(time)} h and deletes it; Principia's path: " +
            (burn != null ? BurnAnchor.Explain(burn) : "no burn at that time"),
            $"  vessel plan: {Patches(_vesselPlan)}",
        };
        List<FlightPlan>? computed = _newPlans;
        for (int i = 0; i < burns.BurnCount; i++)
        {
            if (burns.TryGetBurn(i, out Burn? b) && b != null)
            {
                FlightPlan? plan = computed != null && i < computed.Count ? computed[i] : b.FlightPlan;
                lines.Add($"  burn {i + 1} in {H(b.Time)} h, Δv {b.DeltaVVlf.Length():F1} m/s: {Patches(plan)}");
            }
        }
        Log.Info(string.Join(Environment.NewLine, lines));
    }
}

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
