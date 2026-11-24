using System.Diagnostics;
using System.Reflection;
using System.Reflection.Emit;
using System.Runtime.CompilerServices;
using System.Text;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// KSA draws and drags a node on its patch's conic, thousands of km off the n-body path far ahead; with KSA's conics
// hidden, nodes follow Principia's path instead. The burn's time and Δv stay KSA's.
internal static class BurnNode
{
    // Window: how far either side of the node a drag step may move it.
    public readonly record struct NodePath(Line Line, double Start, double Window, double[] StateTimes, QP[] States);

    // Its track, or until that arrives, the prediction for a node before the first burn. Without `window`, Window is
    // NaN where it would take the burn's patch, which BurnAnchor makes from this path.
    public static NodePath? Path(Burn burn, bool window = true)
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

    // Its path is the prediction.
    private static bool IsFirst(Burn burn) =>
        !burn.Vehicle.BurnPlan.AnyWithDeltaVBefore(burn.Time, inclusive: false, except: burn);

    private static bool Covers(Prediction? line, double t) =>
        line is { Times.Length: > 1 } && t >= line.Times[0] && t <= line.Times[^1];

    // For the log.
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

    public static readonly ConditionalWeakTable<Burn, StrongBox<Source>> LastSource = new();

    private static void Placed(Burn burn, Source source) =>
        LastSource.AddOrUpdate(burn, new StrongBox<Source>(source));

    // KSA deletes a burn off the patches of the vessel's or the previous burn's plan; only an impact ends that plan for
    // good, as KSA rebuilds a previous burn's plan to wherever this burn moves.
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

    // Kept while the time is unchanged, so a node never falls back to KSA's conic between two predictions.
    private static readonly ConditionalWeakTable<Burn, StrongBox<(UniverseTime Time, double3 Cce)>> Drawn = new();

    // Relative to the parent's position now.
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

// Burn.PositionCce is small enough to be inlined, so it is replaced inside UpdateGizmos.
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

// KSA shifts a dragged burn by conic time since periapsis, never before its patch's start, so a node dragged past a
// sphere boundary couldn't come back. Here it gets the time of its path's point nearest the cursor, within its window
// and its sphere, never in the past or past the end of KSA's plan.
[HarmonyPatch(typeof(Burn), nameof(Burn.GizmoPositionLerp))]
internal static class BurnGizmoDragPatch
{
    // Short of a sphere crossing (s), well beyond its precision (1 ms).
    private const double CrossingMargin = 0.01;

    // At now a node would be past at once, with no track to follow (s).
    private const double MinimumLead = 60;

    // The tightest of `bounds` by `pick` (Math.Max for a lower bound), with its name.
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
            // Within its sphere, as KSA keeps a node on its patch: beyond, its Δv would mean another burn.
            int s = line.StretchAt(t);
            Stretch stretch = line.Stretches[s];
            double first = s > 0 ? stretch.From + CrossingMargin : stretch.From;
            double last = stretch.Closed ? stretch.To - CrossingMargin : stretch.To;
            (double lo, string loWhy) = Bound(Math.Max, (now + MinimumLead, "a minute from now"), (path.Start, "the start of its path"),
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
            OrbitHover.Nearest(line, 0, mappers, lo, hi, camera, mouse, inViewPort.Size, burn.Patch,
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

// Why a node doesn't follow the cursor, or isn't where it should be ("[node]"). Main thread.
internal static class NodeDiagnostics
{
    private static readonly long Interval = Stopwatch.Frequency / 2;

    // Each drag frame's outcome, summed and logged every Interval.
    private static Burn? _dragged;
    private static long _since, _lastFrame;
    private static readonly Dictionary<string, int> Outcomes = new();
    private static string _context = "";

    private static double Now => GameTime.Now;
    private static string Hours(double t) => $"{(t - Now) / 3600:F2} h";

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

    // Only if a frame didn't simply follow the cursor.
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

    private sealed record Placement(string Signature, long Logged);
    private static readonly ConditionalWeakTable<Burn, Placement> Placements = new();
    private static readonly ConditionalWeakTable<Burn, object> Seen = new();

    // Logs where KSA placed a node whenever that changes in a way that matters (source, body, off screen, NaN).
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

    // Where each line and KSA's conic would put the node, from where it was drawn.
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

// A node's prograde/normal/outward in the plotting frame (arrows, drags, KSA's burn fields and buttons); Δv stays KSA's.
internal static class BurnAxes
{
    // A burn's axes (x prograde, y normal, z outward) to Cce: KSA's and the frame's at the burn, and the frame's as drawn now.
    internal readonly record struct Axes(doubleQuat Ksa, doubleQuat Frame, doubleQuat FrameShown);

    public static Axes? Of(Burn burn)
    {
        if (burn.ParentDepartureBurn || burn.Vehicle is not { } vehicle ||
            !KsaConics.Hidden(vehicle) || PrincipiaSystem.Current is not { Failed: false } system ||
            (Predictor.PlanFor(vehicle) ?? Predictor.PredictionFor(vehicle))?.Line is not { Stretches.Length: > 0 } line)
        {
            return null;
        }
        Orbit orbit = burn.Patch.Orbit;
        Ephemeris ephemeris = system.Ephemeris;
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (orbit.Parent is not Astronomical parent || system.IndexOf(parent) is not int parentIndex ||
            !(t <= ephemeris.TimeRange.TMax))
        {
            return null;
        }
        // The frame the line is drawn in where the burn is: an encounter's stretch is drawn about its body.
        int stretch = line.StretchAt(t);
        FrameSpec frame = line.Stretches[stretch].Frame;
        FrameMapper now = line.Mappers()[stretch];
        StateVectors sv = orbit.GetStateVectorsAt(burn.Time);
        if (sv.GetVlf2ParentCce(in orbit) is not { } ksa)
        {
            return null;
        }
        // As KSA's axes, with the velocity seen in the frame in place of the one about the parent.
        doubleQuat cci2Cce = orbit.Parent.GetCci2Cce();
        double3 r = sv.PositionCci.Transform(cci2Cce), v = sv.VelocityCci.Transform(cci2Cce);
        QP p = ephemeris.State(parentIndex, t);
        double3 seen = frame.FrameVelocity(ephemeris, t, new QP(p.Q.ToDouble3() + r, p.P.ToDouble3() + v));
        if (StateVectors.GetVlf2GivenFrame(r.Normalized(), seen.Normalized()) is not { } axes)
        {
            return null;
        }
        double3 x = frame.DirectionNow(ephemeris, t, double3.UnitX.Transform(axes), now);
        double3 y = frame.DirectionNow(ephemeris, t, double3.UnitY.Transform(axes), now);
        double3 z = frame.DirectionNow(ephemeris, t, double3.UnitZ.Transform(axes), now);
        doubleQuat shown = doubleQuat.CreateFromRotationMatrix(new double4x4(x.X, x.Y, x.Z, 0, y.X, y.Y, y.Z, 0,
                                                                              z.X, z.Y, z.Z, 0, 0, 0, 0, 1));
        return new Axes(ksa, axes, shown);
    }

    // A Δv along the frame's axes, in KSA's.
    public static double3 ToKsa(in Axes axes, double3 inFrame) => Along(inFrame.Transform(axes.Frame), axes.Ksa);

    // A Δv along KSA's axes, in the frame's.
    public static double3 FromKsa(in Axes axes, double3 inKsa) => Along(inKsa.Transform(axes.Ksa), axes.Frame);

    private static double3 Along(double3 cce, doubleQuat axes) =>
        new(double3.Dot(cce, double3.UnitX.Transform(axes)), double3.Dot(cce, double3.UnitY.Transform(axes)),
            double3.Dot(cce, double3.UnitZ.Transform(axes)));
}

// KSA turns a node's sphere and arrows by its axes at the burn (GetVlf2ParentCce): the frame's instead, as drawn now.
[HarmonyPatch(typeof(Burn), nameof(Burn.UpdateGizmos))]
internal static class BurnArrowsPatch
{
    [ThreadStatic] private static Burn? _burn;

    [HarmonyPrefix]
    private static void Prefix(Burn __instance) => _burn = __instance;

    [HarmonyFinalizer]
    private static void Finalizer() => _burn = null;

    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions) =>
        Patching.ReplaceCalls(instructions, "[burn arrows] Burn.UpdateGizmos no longer reads GetVlf2ParentCce; " +
                              "node arrows stay in KSA's axes", 1,
            (AccessTools.Method(typeof(StateVectors), nameof(StateVectors.GetVlf2ParentCce)),
             AccessTools.Method(typeof(BurnArrowsPatch), nameof(Vlf2ParentCce))));

    private static doubleQuat? Vlf2ParentCce(in StateVectors sv, in Orbit orbit)
    {
        try
        {
            if (_burn is { } burn && BurnAxes.Of(burn) is { } axes)
            {
                return axes.FrameShown;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] turning a node's arrows", e);
        }
        return sv.GetVlf2ParentCce(in orbit);
    }
}

// A drag on arrow i adds along the frame's axis i, converted to KSA's axes (Burn.Update calls ApplyGizmoDeltaV).
[HarmonyPatch(typeof(Burn), nameof(Burn.Update), typeof(FlightComputer), typeof(IViewport), typeof(double))]
internal static class BurnArrowDragPatch
{
    private static readonly Action<Burn, double3> Apply =
        AccessTools.MethodDelegate<Action<Burn, double3>>(AccessTools.Method(typeof(Burn), "ApplyGizmoDeltaV"));

    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions) =>
        Patching.ReplaceCalls(instructions, "[burn arrows] Burn.Update no longer calls ApplyGizmoDeltaV; " +
                              "arrow drags stay in KSA's axes", 3,
            (AccessTools.Method(typeof(Burn), "ApplyGizmoDeltaV"),
             AccessTools.Method(typeof(BurnArrowDragPatch), nameof(ApplyGizmoDeltaV))));

    private static void ApplyGizmoDeltaV(Burn burn, double3 offsetVlf)
    {
        try
        {
            if (BurnAxes.Of(burn) is { } axes)
            {
                offsetVlf = BurnAxes.ToKsa(axes, offsetVlf);
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] converting a drag", e);
        }
        Apply(burn, offsetVlf);
    }
}

// KSA's burn editor and HUD burn panel fields read and write Burn.DeltaVVlf: they show and take the frame's components.
[HarmonyPatch]
internal static class BurnFieldsPatch
{
    private static readonly FieldInfo DeltaV = AccessTools.Field(typeof(Burn), nameof(Burn.DeltaVVlf));

    // The frame's components, for the field reads (one burn at a time, on the UI thread).
    private static double3 _shown;

    [HarmonyTargetMethods]
    private static IEnumerable<MethodBase> TargetMethods() =>
    [
        AccessTools.Method(typeof(Burn), nameof(Burn.DrawBurnEditorWindowContent)),
        AccessTools.Method(typeof(BurnCanvasHost), nameof(BurnCanvasHost.Draw)),
    ];

    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions, MethodBase original)
    {
        int replaced = 0;
        foreach (CodeInstruction instruction in instructions)
        {
            MethodInfo? to = !Equals(instruction.operand, DeltaV) ? null
                : instruction.opcode == OpCodes.Ldflda ? AccessTools.Method(typeof(BurnFieldsPatch), nameof(Shown))
                : instruction.opcode == OpCodes.Ldfld ? AccessTools.Method(typeof(BurnFieldsPatch), nameof(ShownValue))
                : instruction.opcode == OpCodes.Stfld ? AccessTools.Method(typeof(BurnFieldsPatch), nameof(Enter))
                : null;
            if (to != null)
            {
                instruction.opcode = OpCodes.Call;
                instruction.operand = to;
                replaced++;
            }
            yield return instruction;
        }
        if (replaced == 0)
        {
            Log.Warn($"[burn arrows] {original.DeclaringType?.Name}.{original.Name} no longer uses Burn.DeltaVVlf; " +
                     "its fields stay in KSA's axes");
        }
    }

    private static ref double3 Shown(Burn burn)
    {
        try
        {
            if (BurnAxes.Of(burn) is { } axes)
            {
                double3 v = BurnAxes.FromKsa(axes, burn.DeltaVVlf);
                _shown = new(Displayed(v.X), Displayed(v.Y), Displayed(v.Z));
                return ref _shown;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] showing a burn in the frame", e);
        }
        return ref burn.DeltaVVlf;
    }

    private static double3 ShownValue(Burn burn) => Shown(burn);

    // To the six decimals KSA's fields show, without -0, so the path's recomputation noise below them doesn't flicker.
    private static double Displayed(double x) => Math.Round(x, 6) + 0.0;

    private static void Enter(Burn burn, double3 inFrame)
    {
        try
        {
            if (BurnAxes.Of(burn) is { } axes)
            {
                burn.DeltaVVlf = BurnAxes.ToKsa(axes, inFrame);
                return;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] entering a burn in the frame", e);
        }
        burn.DeltaVVlf = inFrame;
    }
}

// KSA's HUD burn buttons add a step along prograde/normal/outward (GaugeButtonBurn.ApplyDeltaV): the frame's.
[HarmonyPatch(typeof(GaugeButtonBurn), nameof(GaugeButtonBurn.OnActivated))]
internal static class BurnButtonsPatch
{
    private static readonly Action<Burn, FlightComputer, double, double, double> Apply =
        AccessTools.MethodDelegate<Action<Burn, FlightComputer, double, double, double>>(
            AccessTools.Method(typeof(GaugeButtonBurn), "ApplyDeltaV"));

    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions) =>
        Patching.ReplaceCalls(instructions, "[burn arrows] GaugeButtonBurn.OnActivated no longer calls ApplyDeltaV; " +
                              "the HUD burn buttons stay in KSA's axes", 6,
            (AccessTools.Method(typeof(GaugeButtonBurn), "ApplyDeltaV"),
             AccessTools.Method(typeof(BurnButtonsPatch), nameof(ApplyDeltaV))));

    private static void ApplyDeltaV(Burn burn, FlightComputer fc, double x, double y, double z)
    {
        try
        {
            if (BurnAxes.Of(burn) is { } axes)
            {
                double3 ksa = BurnAxes.ToKsa(axes, new double3(x, y, z));
                (x, y, z) = (ksa.X, ksa.Y, ksa.Z);
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] converting a burn button", e);
        }
        Apply(burn, fc, x, y, z);
    }
}

// A node moved in time (drag, ±period, Time fields or buttons) keeps its Δv along the frame's axes, not KSA's.
[HarmonyPatch(typeof(InputEvents), nameof(InputEvents.ApplyInputEvents))]
internal static class BurnMovePatch
{
    // The controlled vessel's burns as KSA last applied them: time and Δv. UI thread.
    private static readonly ConditionalWeakTable<Burn, StrongBox<(UniverseTime Time, double3 DeltaV)>> Applied = new();

    [HarmonyPrefix]
    private static void Prefix()
    {
        try
        {
            // Every move queues a burn update: one whose time changed with its Δv untouched was moved.
            foreach (InputEvents.BurnUpdateData update in InputEvents.BurnUpdateBuffer.Get())
            {
                if (update.Burn is { } burn && !update.AddBurn && !update.DeleteBurn &&
                    Applied.TryGetValue(burn, out var last) && last.Value.Time != burn.Time &&
                    last.Value.DeltaV.Equals(burn.DeltaVVlf))
                {
                    burn.DeltaVVlf = Moved(burn, last.Value.Time);
                    last.Value = (burn.Time, burn.DeltaVVlf);
                }
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] keeping a moved burn in the frame", e);
        }
    }

    [HarmonyPostfix]
    private static void Postfix()
    {
        try
        {
            if (Program.ControlledVehicle is not { } vehicle)
            {
                return;
            }
            foreach (Burn burn in vehicle.BurnPlan.All())
            {
                Applied.AddOrUpdate(burn, new((burn.Time, burn.DeltaVVlf)));
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] noting the burns", e);
        }
    }

    // The burn's Δv in the frame's axes at `from`, in KSA's at its time now.
    private static double3 Moved(Burn burn, UniverseTime from)
    {
        UniverseTime to = burn.Time;
        BurnAxes.Axes? before;
        burn.Time = from;
        try
        {
            before = BurnAxes.Of(burn);
        }
        finally
        {
            burn.Time = to;
        }
        return before is { } a && BurnAxes.Of(burn) is { } b
            ? BurnAxes.ToKsa(b, BurnAxes.FromKsa(a, burn.DeltaVVlf))
            : burn.DeltaVVlf;
    }
}
