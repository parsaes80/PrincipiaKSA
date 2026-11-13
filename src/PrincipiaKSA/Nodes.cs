using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// Maneuver nodes on Principia's lines: where a node is drawn and how it is
// dragged (BurnNode), and why, in the log (NodeDiagnostics).

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
        if (Predictor.NodesFor(vehicle) is { } nodes &&
            nodes.ByBurn.TryGetValue(burn, out NodeTrack? track) && t >= track.Times[0] && t <= track.End)
        {
            return new NodePath(track.Line, track.Start, nodes.Windows.TryGetValue(burn, out double w) ? w : 0,
                                track.StateTimes, track.States);
        }
        if (IsFirst(burn) && Predictor.PredictionFor(vehicle) is { } coast && Covers(coast, t))
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
        Prediction? coast = Predictor.PredictionFor(vehicle);
        Prediction? plan = Predictor.PlanFor(vehicle);
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (IsFirst(burn) && Covers(coast, t))
        {
            return coast;
        }
        return Covers(plan, t) ? plan : Covers(coast, t) ? coast : null;
    }

    // No burn with Δv comes before it: its path is the prediction.
    private static bool IsFirst(Burn burn) =>
        !burn.Vehicle.BurnPlan.AnyWithDeltaVBefore(burn.Time, inclusive: false, except: burn);

    private static bool Covers(Prediction? line, double t) =>
        line is { Times.Length: > 1 } && t >= line.Times[0] && t <= line.Times[^1];

    // Where a node is drawn from, for the log (NodeDiagnostics).
    public enum Source { Track, Prediction, FlightPlan, LastDrawn, KsaConic }

    private static (double3 Ecl, Source Source)? Ecl(Burn burn)
    {
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (Path(burn) is { } path)
        {
            bool track = Predictor.NodesFor(burn.Vehicle) is { } nodes && nodes.ByBurn.ContainsKey(burn);
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
            if (burns.TryGetBurn(j, out Burn? previous) && previous != null && previous.VesselTime() >= now)
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
        return Patching.ReplaceCalls(instructions, "[node] Burn.UpdateGizmos no longer reads Burn.PositionCce; " +
                                     "nodes stay on KSA's conics",
            (AccessTools.PropertyGetter(typeof(Burn), nameof(Burn.PositionCce)),
             AccessTools.Method(typeof(BurnNode), nameof(BurnNode.PositionCce))));
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
            double now = GameTime.Now;
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

// Why a maneuver node doesn't follow the cursor, or isn't where it should be
// ("[node]" in the log). Main thread.
internal static class NodeDiagnostics
{
    private static readonly long Interval = Stopwatch.Frequency / 2;

    // A drag: each frame's outcome, summed and logged every Interval.
    private static Burn? _dragged;
    private static long _since, _lastFrame;
    private static readonly Dictionary<string, int> Outcomes = new();
    private static string _context = "";

    private static double Now => GameTime.Now;
    private static string Hours(double t) => $"{(t - Now) / 3600:F2} h";

    // One frame of dragging `burn` along its path: what the drag step did.
    public static void Drag(Burn burn, string outcome, BurnNode.NodePath? path = null)
    {
        long now = Stopwatch.GetTimestamp();
        if (_dragged != burn || now - _lastFrame > Interval)
        {
            Flush();
            _dragged = burn;
            _since = now;
        }
        _lastFrame = now;
        Outcomes[outcome] = Outcomes.GetValueOrDefault(outcome) + 1;
        _context = Context(burn, path);
        if (now - _since >= Interval)
        {
            Flush();
            _since = now;
        }
    }

    // Logs the drag so far if any frame didn't simply follow the cursor.
    private static void Flush()
    {
        if (_dragged == null || Outcomes.Count == 0)
        {
            return;
        }
        int frames = Outcomes.Values.Sum();
        if (Outcomes.Keys.Any(k => k != "followed"))
        {
            string outcomes = string.Join(", ", Outcomes.OrderByDescending(o => o.Value)
                                                        .Select(o => $"{o.Key} ×{o.Value}"));
            Log.Info($"[node] dragging the node in {Hours(GameTime.ToSecondsSinceJ2000(_dragged.Time))}, " +
                     $"{frames} frames: {outcomes}; {_context}");
        }
        Outcomes.Clear();
    }

    // What the drag works from: the node's track and how old it is.
    private static string Context(Burn burn, BurnNode.NodePath? path)
    {
        var text = new StringBuilder();
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (Predictor.NodesFor(burn.Vehicle) is { } nodes && nodes.ByBurn.TryGetValue(burn, out NodeTrack? track))
        {
            double window = nodes.Windows.GetValueOrDefault(burn);
            double centre = track.End - window;
            text.Append($"its track was published {Clock.Ms(Stopwatch.GetTimestamp() - nodes.Computed):F0} ms ago, " +
                        $"for the node at {Hours(centre)} ±{window / 3600:F2} h (now at {Hours(t)}, " +
                        $"{Math.Abs(t - centre) / 3600:F2} h from that)");
        }
        else
        {
            text.Append(path != null
                ? "no track of its own yet: on the prediction's line"
                : $"no track and no line: {BurnAnchor.Explain(burn)}");
        }
        if (Predictor.PlanFor(burn.Vehicle) is { } plan)
        {
            text.Append($"; the flight plan with the tracks took {plan.ComputeMs:F0} ms");
        }
        return text.ToString();
    }

    // Each node's placement as last logged, and when.
    private sealed record Placement(string Signature, long Logged);
    private static readonly ConditionalWeakTable<Burn, Placement> Placements = new();
    private static readonly ConditionalWeakTable<Burn, object> Seen = new();

    // After KSA places a node's sphere and arrows: logs where they went
    // whenever that changes in a way that matters (another source of position,
    // another body, off screen, not a number).
    public static void Placed(Burn burn, IViewport viewport)
    {
        if (!viewport.IsMain() || burn.ParentDepartureBurn || burn.Vehicle is not { } vehicle ||
            vehicle != Program.ControlledVehicle || !KsaConics.Hidden(vehicle))
        {
            return;
        }
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (!Seen.TryGetValue(burn, out _))
        {
            Seen.Add(burn, new object());
            Log.Info($"[node] KSA made a node object for the burn in {Hours(t)} " +
                     $"(Δv {burn.DeltaVVlf.Length():F1} m/s); the mod's tracks and anchors are per object");
        }
        PatchedConic patch = burn.Patch;
        IParentBody parent = patch.Orbit.Parent;
        string parentId = (parent as IObjectId)?.Id ?? "?";
        BurnAnchor.Anchor? anchor = BurnAnchor.Cached(burn);
        bool anchored = anchor != null && ReferenceEquals(anchor.Patch, patch);
        double3 cce = BurnNode.PositionCce(burn);
        BurnNode.Source source = BurnNode.LastSource.TryGetValue(burn, out var s) ? s.Value : BurnNode.Source.KsaConic;
        double3 ecl = parent.GetPositionEclFromCce(cce);
        StateVectors sv = patch.Orbit.GetStateVectorsAt(burn.Time);
        bool finite = IsFinite(ecl) && IsFinite(sv.PositionCci) && IsFinite(sv.VelocityCci);
        Camera camera = viewport.GetCamera();
        float2 screen = finite ? camera.EclToScreen(ecl) : default;
        int2 size = viewport.Size;
        bool onScreen = finite && float.IsFinite(screen.X) && float.IsFinite(screen.Y) &&
                        screen.X >= 0 && screen.Y >= 0 && screen.X <= size.X && screen.Y <= size.Y;
        string signature = $"{source}|{parentId}|{anchored}|{finite}|{onScreen}";
        long now = Stopwatch.GetTimestamp();
        if (Placements.TryGetValue(burn, out Placement? last) &&
            (last.Signature == signature || now - last.Logged < Interval / 2))
        {
            return;
        }
        Placements.AddOrUpdate(burn, new Placement(signature, now));
        var text = new StringBuilder();
        text.Append($"[node] the node in {Hours(t)} is drawn from {Describe(source)}, about {parentId} " +
                    $"({(anchored ? "anchored on Principia's path" : "on KSA's own patch")}), ");
        text.Append(!finite ? "NOT A NUMBER (position or arrows)" :
                    onScreen ? $"on screen at ({screen.X:F0}, {screen.Y:F0})" :
                    $"OFF SCREEN at ({screen.X:F0}, {screen.Y:F0}) of {size.X}×{size.Y}");
        text.Append($"; its patch {Hours(GameTime.ToSecondsSinceJ2000(patch.StartTime))} to " +
                    $"{Hours(GameTime.ToSecondsSinceJ2000(patch.EndTime))}, e {patch.Orbit.Eccentricity:F4}");
        if (DrawnPosition(burn, t) is { } line && finite)
        {
            text.Append($"; {(ecl - line).Length() / 1000:N0} km from Principia's line there");
        }
        if (!anchored || !finite || !onScreen || source is BurnNode.Source.KsaConic or BurnNode.Source.LastDrawn)
        {
            text.Append($"; {Candidates(burn, t, ecl, parent)}; {BurnAnchor.Explain(burn)}");
        }
        Log.Info(text.ToString());
    }

    private static string Describe(BurnNode.Source source) => source switch
    {
        BurnNode.Source.Track => "its own track",
        BurnNode.Source.Prediction => "the prediction's line",
        BurnNode.Source.FlightPlan => "the flight plan's line",
        BurnNode.Source.LastDrawn => "where it was last drawn (no line covers it now)",
        _ => "KSA's conic (no line covers it)",
    };

    // Where each line, and KSA's own conic, would put the node, as distances
    // from where it was drawn, with the stretch of each line it is on.
    private static string Candidates(Burn burn, double t, double3 drawn, IParentBody parent)
    {
        var parts = new List<string>();
        string Away(double3 p) => IsFinite(p) ? $"{(p - drawn).Length() / 1000:N0} km away" : "not a number";
        void Add(string name, Line? line)
        {
            if (line is not { } l || l.Times.Length < 2 || !(l.Times[0] <= t && t <= l.Times[^1]))
            {
                parts.Add($"{name}: doesn't cover it");
                return;
            }
            Stretch s = l.Stretches[l.StretchAt(t)];
            parts.Add($"{name}: {Away(l.EclAt(t))} (stretch about {s.Body.Id}, drawn {s.Frame.Describe()})");
        }
        Add("its track", BurnNode.Path(burn)?.Line);
        Add("flight plan", Predictor.PlanFor(burn.Vehicle)?.Line);
        Add("prediction", Predictor.PredictionFor(burn.Vehicle)?.Line);
        parts.Add($"KSA's conic: {Away(parent.GetPositionEclFromCce(burn.PositionCce))}");
        parts.Add($"{(parent as IObjectId)?.Id} now: {Away(parent.GetPositionEclFromCce(double3.Zero))}");
        return "candidates: " + string.Join(", ", parts);
    }

    // Where the drawn lines put the vessel at t, if one covers it.
    private static double3? DrawnPosition(Burn burn, double t)
    {
        foreach (Prediction? p in new[] { Predictor.PlanFor(burn.Vehicle), Predictor.PredictionFor(burn.Vehicle) })
        {
            if (p is { Times.Length: > 1 } && p.Times[0] <= t && t <= p.Times[^1])
            {
                return p.Line.EclAt(t);
            }
        }
        return null;
    }

    private static bool IsFinite(double3 v) => double.IsFinite(v.X) && double.IsFinite(v.Y) && double.IsFinite(v.Z);
}

// After KSA places a node's sphere and arrows (Burn.UpdateGizmos).
[HarmonyPatch(typeof(Burn), nameof(Burn.UpdateGizmos))]
internal static class BurnGizmoDiagnosticsPatch
{
    [HarmonyPostfix]
    private static void Postfix(Burn __instance, IViewport inViewport)
    {
        try
        {
            NodeDiagnostics.Placed(__instance, inViewport);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[node] checking a node's placement", e);
        }
    }
}
