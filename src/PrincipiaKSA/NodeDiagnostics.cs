using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Text;
using Brutal.Numerics;
using HarmonyLib;
using KSA;
using PrincipiaKSA.Patches;

namespace PrincipiaKSA;

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

    private static double Ms(long ticks) => ticks * 1000.0 / Stopwatch.Frequency;
    private static double Now => GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
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
        if (Predictor.LatestNodes is { } nodes && nodes.Vehicle == burn.Vehicle &&
            nodes.ByBurn.TryGetValue(burn, out NodeTrack? track))
        {
            double window = nodes.Windows.GetValueOrDefault(burn);
            double centre = track.End - window;
            text.Append($"its track was published {Ms(Stopwatch.GetTimestamp() - nodes.Computed):F0} ms ago, " +
                        $"for the node at {Hours(centre)} ±{window / 3600:F2} h (now at {Hours(t)}, " +
                        $"{Math.Abs(t - centre) / 3600:F2} h from that)");
        }
        else
        {
            text.Append(path != null
                ? "no track of its own yet: on the prediction's line"
                : $"no track and no line: {BurnAnchor.Explain(burn)}");
        }
        if (Predictor.LatestPlan is { } plan && plan.Vehicle == burn.Vehicle)
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
        Add("flight plan", Predictor.LatestPlan is { } plan && plan.Vehicle == burn.Vehicle ? plan.Line : null);
        Add("prediction", Predictor.Latest is { } coast && coast.Vehicle == burn.Vehicle ? coast.Line : null);
        parts.Add($"KSA's conic: {Away(parent.GetPositionEclFromCce(burn.PositionCce))}");
        parts.Add($"{(parent as IObjectId)?.Id} now: {Away(parent.GetPositionEclFromCce(double3.Zero))}");
        return "candidates: " + string.Join(", ", parts);
    }

    // Where the drawn lines put the vessel at t, if one covers it.
    private static double3? DrawnPosition(Burn burn, double t)
    {
        foreach (Prediction? p in new[] { Predictor.LatestPlan, Predictor.Latest })
        {
            if (p is { Times.Length: > 1 } && p.Vehicle == burn.Vehicle && p.Times[0] <= t && t <= p.Times[^1])
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
