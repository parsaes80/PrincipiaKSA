using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

// Maneuver nodes on Principia's lines. KSA draws a node at its patch's conic
// at the burn time and drags it along that conic, which far ahead can be
// thousands of km off the n-body path; with KSA's conics hidden, both follow
// Principia's path instead. The burn's time and Δv stay KSA's.
internal static class BurnNode
{
    // A stretch of n-body path as drawn; Window is how far either side of
    // the node a drag step may move it.
    public readonly record struct NodePath(Line Line, double Start, double Window, double[] StateTimes, QP[] States);

    // The path a node lies on: its approach track (the path before its burn),
    // or, until that arrives, the prediction for a node before the first burn.
    public static NodePath? Path(Burn burn) => Path(burn, window: true);

    // Without `window`, Window is NaN where it would take the burn's patch,
    // which BurnAnchor makes from this path.
    public static NodePath? Path(Burn burn, bool window)
    {
        if (burn.ParentDepartureBurn || burn.Vehicle is not { } vehicle || !KsaConics.Hidden(vehicle))
        {
            return null;
        }
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (Predictor.LatestNodes is { } nodes && nodes.Vehicle == vehicle &&
            nodes.ByBurn.TryGetValue(burn, out NodeTrack? track) && t >= track.Times[0] && t <= track.End)
        {
            return new NodePath(track.Line, track.Start, nodes.Windows.TryGetValue(burn, out double w) ? w : 0,
                                track.StateTimes, track.States);
        }
        if (IsFirst(burn) && Settings.ShowPrediction && Predictor.Latest is { } coast && coast.Vehicle == vehicle &&
            Covers(coast, t))
        {
            return new NodePath(coast.Line, coast.Times[0],
                                window ? Predictor.NodeWindow(burn.Patch.Orbit, t - coast.Times[0]) : double.NaN,
                                coast.StateTimes, coast.States);
        }
        return null;
    }

    // The drawn line a node without a path sits on.
    private static Prediction? DrawnLine(Burn burn)
    {
        Vehicle vehicle = burn.Vehicle;
        Prediction? coast = Settings.ShowPrediction && Predictor.Latest is { } c && c.Vehicle == vehicle ? c : null;
        Prediction? plan = Settings.ShowFlightPlan && Predictor.LatestPlan is { } p && p.Vehicle == vehicle ? p : null;
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (IsFirst(burn) && Covers(coast, t))
        {
            return coast;
        }
        return Covers(plan, t) ? plan : Covers(coast, t) ? coast : null;
    }

    // No burn with Δv comes before it.
    private static bool IsFirst(Burn burn)
    {
        BurnPlan burns = burn.Vehicle.BurnPlan;
        for (int i = 0; i < burns.BurnCount; i++)
        {
            if (burns.TryGetBurn(i, out Burn? other) && other != null && other != burn && other.HasDeltaV &&
                !other.IsInterstellar && Predictor.VesselTime(other) < burn.Time)
            {
                return false;
            }
        }
        return true;
    }

    private static bool Covers(Prediction? line, double t) =>
        line is { Times.Length: > 1 } && t >= line.Times[0] && t <= line.Times[^1];

    // Where a node is drawn from, for the log (NodeDiagnostics).
    public enum Source { Track, Prediction, FlightPlan, LastDrawn, KsaConic }

    private static (double3 Ecl, Source Source)? Ecl(Burn burn)
    {
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (Path(burn) is { } path)
        {
            bool track = Predictor.LatestNodes is { } nodes && nodes.ByBurn.ContainsKey(burn);
            return (path.Line.EclAt(t), track ? Source.Track : Source.Prediction);
        }
        return DrawnLine(burn) is { } line
            ? (line.Line.EclAt(t), line.IsFlightPlan ? Source.FlightPlan : Source.Prediction)
            : null;
    }

    // Where each node was placed last, and from what.
    public static readonly ConditionalWeakTable<Burn, StrongBox<Source>> LastSource = new();

    private static void Placed(Burn burn, Source source) =>
        LastSource.AddOrUpdate(burn, new StrongBox<Source>(source));

    // The times at which KSA can hold this burn: it must lie on a patch of the
    // vessel's plan (first burn) or of the previous burn's still ahead, or KSA
    // deletes it. Only an impact ends that plan for good: KSA cuts a previous
    // burn's plan at this burn's ignition but rebuilds it to wherever this
    // burn moves.
    public static (double Start, double End) KsaCoverage(Burn burn)
    {
        Vehicle vehicle = burn.Vehicle;
        BurnPlan burns = vehicle.BurnPlan;
        UniverseTime now = Universe.GetElapsedTime();
        FlightPlan? source = null;
        for (int j = burns.TryGetBurnIndex(burn) - 1; j >= 0 && source == null; j--)
        {
            if (burns.TryGetBurn(j, out Burn? previous) && previous != null && Predictor.VesselTime(previous) >= now)
            {
                source = previous.FlightPlan;
            }
        }
        source ??= vehicle.FlightPlan;
        List<PatchedConic> patches = source.Patches;
        if (patches.Count == 0)
        {
            return (double.NegativeInfinity, double.PositiveInfinity);
        }
        PatchedConic last = patches[^1];
        double end = last.EndTransition == PatchTransition.Impact
            ? GameTime.ToSecondsSinceJ2000(last.EndTime)
            : double.PositiveInfinity;
        return (GameTime.ToSecondsSinceJ2000(patches[0].StartTime), end);
    }

    // Where each node was last drawn, kept while its time is unchanged, so a
    // node never falls back to KSA's conic between two predictions.
    private static readonly ConditionalWeakTable<Burn, StrongBox<(UniverseTime Time, double3 Cce)>> Drawn = new();

    // Replaces Burn.PositionCce where KSA places the node: relative to the
    // parent's position now.
    public static double3 PositionCce(Burn burn)
    {
        try
        {
            // A departure's node is on its parent's orbit, where KSA puts it.
            if (!burn.ParentDepartureBurn && burn.Vehicle is { } vehicle && KsaConics.Hidden(vehicle))
            {
                if (Ecl(burn) is var (ecl, source))
                {
                    double3 cce = ecl - burn.Patch.Orbit.Parent.GetPositionEclFromCce(double3.Zero);
                    Drawn.AddOrUpdate(burn, new StrongBox<(UniverseTime, double3)>((burn.Time, cce)));
                    Placed(burn, source);
                    return cce;
                }
                if (Drawn.TryGetValue(burn, out var last) && last.Value.Time == burn.Time)
                {
                    Placed(burn, Source.LastDrawn);
                    return last.Value.Cce;
                }
                Placed(burn, Source.KsaConic);
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[node] maneuver node position", e);
        }
        return burn.PositionCce;
    }
}

// The node's sphere and handles are placed in UpdateGizmos from
// Burn.PositionCce, a getter small enough to be inlined, so the call is
// replaced in UpdateGizmos itself. (They are turned by the frame of the
// burn's patch at the burn time: Principia's state there, BurnAnchor.)
[HarmonyPatch(typeof(Burn), nameof(Burn.UpdateGizmos))]
internal static class BurnGizmoPositionPatch
{
    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var position = AccessTools.PropertyGetter(typeof(Burn), nameof(Burn.PositionCce));
        int positions = 0;
        foreach (CodeInstruction instruction in instructions)
        {
            if (instruction.Calls(position))
            {
                instruction.opcode = OpCodes.Call;
                instruction.operand = AccessTools.Method(typeof(BurnNode), nameof(BurnNode.PositionCce));
                positions++;
            }
            yield return instruction;
        }
        if (positions == 0)
        {
            Log.Warn("[node] Burn.UpdateGizmos no longer reads Burn.PositionCce; nodes stay on KSA's conics");
        }
    }
}

// Dragging a node along its orbit. KSA shifts the burn by the time since
// periapsis of the conic point nearest the cursor, never before the start of
// the burn's patch, so a node dragged past a sphere-of-influence boundary
// could not come back. Here the burn's time is set to the time of the
// nearest point of its path, within its window and its sphere of influence
// (both ways), and KSA finds the patch for it. Kept from KSA: never in the
// past or before the previous burn, never past the end of KSA's plan;
// parent-departure burns stay KSA's.
[HarmonyPatch(typeof(Burn), nameof(Burn.GizmoPositionLerp))]
internal static class BurnGizmoDragPatch
{
    // How far short of a sphere crossing a node stops (s): well beyond the
    // crossing's precision (1 ms).
    private const double CrossingMargin = 0.01;

    // The tightest of `bounds` by `pick` (Math.Max for a lower bound), and its name.
    private static (double Value, string Why) Bound(Func<double, double, double> pick,
                                                   params (double Value, string Why)[] bounds)
    {
        (double value, string why) = bounds[0];
        foreach (var (v, w) in bounds.Skip(1))
        {
            if (pick(v, value) != value && !double.IsNaN(v))
            {
                (value, why) = (v, w);
            }
        }
        return (value, why);
    }

    [HarmonyPrefix]
    private static bool Prefix(Burn __instance, FlightComputer flightComputer, IViewport inViewPort,
                               ref bool __result)
    {
        using var profiled = FrameProfiler.NodeDrag.Measure();
        try
        {
            Burn burn = __instance;
            if (burn.ParentDepartureBurn || burn.Vehicle is not { } vehicle || !KsaConics.Hidden(vehicle))
            {
                return true;
            }
            __result = false;
            if (BurnNode.Path(burn) is not { } path)
            {
                NodeDiagnostics.Drag(burn, "held: no path for it yet (its track is being computed)");
                return false;  // Its track is on its way: hold still.
            }
            Camera camera = inViewPort.GetCamera();
            float2 mouse = Cursor.GetPosition(inViewPort);
            double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
            double t = GameTime.ToSecondsSinceJ2000(burn.Time);
            double reach = path.Window > 0 ? 0.9 * path.Window : double.PositiveInfinity;
            (double coverStart, double coverEnd) = BurnNode.KsaCoverage(burn);
            Line line = path.Line;
            // Within its sphere of influence, as KSA keeps a node on its
            // patch: the burn is about that body, so beyond, its Δv would
            // mean another burn. It stops just short of an encounter or an
            // escape: at the crossing itself it would be in the next sphere,
            // on the next stretch.
            int s = line.StretchAt(t);
            Stretch stretch = line.Stretches[s];
            double first = s > 0 ? stretch.From + CrossingMargin : stretch.From;
            double last = stretch.Closed ? stretch.To - CrossingMargin : stretch.To;
            // The bounds, each named for the log.
            (double lo, string loWhy) = Bound(Math.Max, (now, "now"), (path.Start, "the start of its path"),
                                              (t - reach, "its drag window"), (coverStart, "the start of KSA's plan"),
                                              (first, "its sphere of influence"));
            (double hi, string hiWhy) = Bound(Math.Min, (line.Times[^1], "the end of its track"),
                                              (t + reach, "its drag window"),
                                              (coverEnd - 1e-3, "an impact in KSA's plan"),
                                              (last, "its sphere of influence"));
            if (!(hi > lo))
            {
                NodeDiagnostics.Drag(burn, $"held: no room between {loWhy} and {hiWhy}", path);
                return false;
            }
            FrameMapper[] mappers = line.Mappers();
            double grab = 1.0;  // The whole screen, as KSA's drag.
            (double Time, double3 Ecl, PatchedConic Patch)? hit = null;
            OrbitHover.Nearest(line, 0, mappers, lo, hi, inViewPort, camera, mouse, inViewPort.Size, burn.Patch,
                               ref grab, ref hit);
            if (hit is not { } h)
            {
                NodeDiagnostics.Drag(burn, "held: no point of its path found near the cursor", path);
                return false;
            }
            // Only toward the cursor, as KSA.
            float2 node = camera.EclToScreen(line.EclAt(mappers, t));
            const double atBound = 1;  // s
            if (!(float2.Distance(camera.EclToScreen(h.Ecl), mouse) < float2.Distance(node, mouse)))
            {
                NodeDiagnostics.Drag(burn, t - lo < atBound ? $"held at {loWhy}"
                                         : hi - t < atBound ? $"held at {hiWhy}"
                                         : "held: already the nearest point to the cursor", path);
                return false;
            }
            NodeDiagnostics.Drag(burn, h.Time - lo < atBound ? $"followed, stopped at {loWhy}"
                                     : hi - h.Time < atBound ? $"followed, stopped at {hiWhy}"
                                     : "followed", path);
            burn.Time = GameTime.FromSecondsSinceJ2000(h.Time);
            burn.Update(flightComputer);
            __result = true;
            return false;
        }
        catch (Exception e)
        {
            Log.WarnOnce("[node] maneuver node drag", e);
            __result = false;
            return false;  // Hold still rather than follow KSA's conic.
        }
    }
}
