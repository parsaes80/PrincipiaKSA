using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// The map view: Principia's lines (OrbitLines) and their markers (MapOverlay)
// in place of KSA's patched conics for the controlled vessel (KsaConics).

// The map's colours: the lines', and the markers'.
internal static class MapColors
{
    public static readonly byte4 Burn = new(255, 230, 80, 255);
    public static readonly byte4 Apsis = new(90, 220, 255, 255);
    public static readonly byte4 Approach = new(255, 90, 220, 255);
    public static readonly byte4 Impact = new(255, 60, 60, 255);

    // The prediction orange, then one colour per burn, cycling after ten: hues
    // at least 80° apart from one segment to the next, as saturated as sRGB
    // allows at OKLCH lightness 0.66–0.80, no yellows (the burns').
    public static readonly byte4[] Segments =
    {
        new(255, 150, 40, 235),   // Orange: the prediction.
        new(220, 110, 255, 235),  // Purple: after the 1st burn.
        new(16, 184, 253, 235),   // Sky blue.
        new(20, 227, 95, 235),    // Green.
        new(249, 40, 168, 235),   // Pink.
        new(83, 141, 253, 235),   // Blue.
        new(21, 219, 190, 235),   // Teal.
        new(254, 68, 58, 235),    // Red.
        new(148, 119, 253, 235),  // Violet.
        new(20, 214, 231, 235),   // Cyan: after the 9th; the 10th is orange again.
    };

    public static byte4 Coast => Segments[0];

    // Segment s's colour (0: before the first burn).
    public static byte4 Segment(int s) => Segments[s % Segments.Length];

    // Alphas of what is drawn dimmer: a stretch drawn about another body, as
    // it goes in the plotting frame (Stretch.Ghost); a node's track.
    public const byte Ghost = 110;
    public const byte Track = 120;

    public static byte4 Dimmed(byte4 c, byte alpha) => new(c.X, c.Y, c.Z, alpha);

    // The flight plan's colour at point i: yellow under thrust, else its
    // segment's.
    public static byte4 Plan(Prediction plan, int i)
    {
        int segment = 0;
        foreach (var (start, end) in plan.BurnArcs)
        {
            if (i > start && i <= end)
            {
                return Burn;
            }
            if (end < i)
            {
                segment++;
            }
        }
        return Segment(segment);
    }

    // The same at time t, for markers.
    public static byte4 PlanAt(Prediction plan, double t)
    {
        int i = Array.BinarySearch(plan.Times, t);
        return Plan(plan, i >= 0 ? i : Math.Min(~i, plan.Times.Length - 1));
    }
}

// Draws Principia's lines with KSA's own 3D orbit-line pass, so planets hide
// the parts behind them, queued where KSA queues its own (CelestialSystem
// rather than Universe.OnPreRender, which may be inlined).
[HarmonyPatch(typeof(CelestialSystem), nameof(CelestialSystem.OnPreRender))]
internal static class OrbitLines
{
    // Main thread only.
    private static double3[] _positions = new double3[1024];
    private static byte4[] _colors = new byte4[1024];
    private static float3[] _packed = new float3[1024 + 512];
    private static byte4[] _packedColors = new byte4[1024 + 512];

    [HarmonyPostfix]
    private static void Postfix(IViewport inViewport)
    {
        using var profiled = FrameProfiler.OrbitLines.Measure();
        try
        {
            if (inViewport.Mode != CameraMode.Map || Program.ControlledVehicle is not { } vehicle)
            {
                return;
            }
            if (Predictor.PredictionFor(vehicle) is { } prediction)
            {
                Submit(inViewport, prediction);
                SubmitDanger(inViewport, prediction);
            }
            if (Predictor.PlanFor(vehicle) is { } plan)
            {
                Submit(inViewport, plan);
                SubmitDanger(inViewport, plan);
            }
            if (Predictor.NodesFor(vehicle) is { } nodes && KsaConics.Hidden(vehicle))
            {
                SubmitNodeTracks(inViewport, nodes);
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[map] drawing the lines", e);
        }
    }

    // A prediction from the vessel on, or a flight plan from its first burn:
    // a line per stretch, each in its frame.
    private static void Submit(IViewport viewport, Prediction prediction)
    {
        Camera camera = viewport.GetCamera();
        double now = GameTime.Now;
        Line line = prediction.Line;
        FrameMapper[] mappers = line.Mappers();
        int n = prediction.Times.Length;
        Ensure(n + 1);
        // From the vessel itself, so the line never lags it between two
        // predictions; the plan only once its first burn has started.
        bool fromVessel = !prediction.IsFlightPlan ||
                          (prediction.FirstIgnition < n && prediction.Times[prediction.FirstIgnition] <= now);
        bool first = true;
        for (int s = 0; s < line.Stretches.Length; s++)
        {
            Stretch stretch = line.Stretches[s];
            int count = 0;
            for (int i = Math.Max(stretch.Start, prediction.FirstIgnition); i <= stretch.End; i++)
            {
                if (prediction.Times[i] <= now)
                {
                    continue;  // Already in the past.
                }
                if (count == 0 && first && fromVessel)
                {
                    _positions[count] = camera.GetPositionEgo(prediction.Vehicle);
                    _colors[count++] = !prediction.IsFlightPlan ? MapColors.Coast
                                                                : MapColors.Plan(prediction, prediction.FirstIgnition + 1);
                }
                _positions[count] = camera.EclToEgo(mappers[s].ToEcl(prediction.FrameCoordinates[i]));
                _colors[count++] = !prediction.IsFlightPlan ? MapColors.Coast : MapColors.Plan(prediction, i);
            }
            first &= count == 0;
            AddLine(viewport, count);
            if (stretch.Ghost != null)
            {
                SubmitGhost(viewport, prediction, stretch, now);
            }
        }
    }

    // A stretch drawn about another body, as it goes in the plotting frame:
    // its colour, dimmer (MapColors.Ghost), as KSA shows the path on through
    // an encounter.
    private static void SubmitGhost(IViewport viewport, Prediction prediction, Stretch stretch, double now)
    {
        Camera camera = viewport.GetCamera();
        var mapper = new FrameMapper(prediction.Frame);
        double[] times = stretch.GhostTimes!;
        double3[] points = stretch.Ghost!;
        Ensure(times.Length);
        int count = 0;
        for (int i = 0; i < times.Length; i++)
        {
            if (times[i] <= now)
            {
                continue;
            }
            byte4 c = prediction.IsFlightPlan ? MapColors.PlanAt(prediction, times[i]) : MapColors.Coast;
            _positions[count] = camera.EclToEgo(mapper.ToEcl(points[i]));
            _colors[count++] = MapColors.Dimmed(c, MapColors.Ghost);
        }
        AddLine(viewport, count);
    }

    // _positions and _colors up to count as one line, if it is one.
    private static void AddLine(IViewport viewport, int count)
    {
        if (count < 2)
        {
            return;
        }
        // Split near the camera, so float precision doesn't bend them.
        int length = Orbit.PackLineNearCamera(_positions.AsSpan(0, count), _colors.AsSpan(0, count),
                                              _packed, _packedColors);
        OrbitLinePass.AddLineVertices(viewport, _packed.AsSpan(0, length), _packedColors.AsSpan(0, length));
        OrbitLinePass.AddLineEnd(viewport);
    }

    // KSA's danger track (Orbit.DrawLines, DangerDisplay.Full): the path's
    // runs inside the centre's near-surface radius, turned with the body to
    // the ground they will pass over, in KSA's danger colour, a little nearer
    // the camera than the path.
    private static void SubmitDanger(IViewport viewport, Prediction prediction)
    {
        if (prediction.Danger.Length == 0 || prediction.Frame.Centre is not IParentBody body)
        {
            return;
        }
        Camera camera = viewport.GetCamera();
        double nowSeconds = GameTime.Now;
        double3 centreEgo = camera.GetPositionEgo(prediction.Frame.Centre);
        byte4 color = (byte4)FlightPlan.DangerColor;
        foreach ((double Time, double3 Relative)[] run in prediction.Danger)
        {
            Ensure(run.Length);
            int count = 0;
            foreach ((double t, double3 relative) in run)
            {
                if (t < nowSeconds)
                {
                    continue;
                }
                _positions[count] = (centreEgo + GroundNowCce(body, t, relative)) * 0.998;
                _colors[count++] = color;
            }
            if (count < 2)
            {
                continue;
            }
            AddLine(viewport, count);
        }
    }

    // A point relative to `body` (Cce axes) at time t, turned with the body's
    // rotation to where the ground under it is now (Orbit.
    // ProjectToCurrentRotationCce): KSA's danger ground track.
    public static double3 GroundNowCce(IParentBody body, double t, double3 relative)
    {
        doubleQuat cci2Cce = body.GetCci2Cce();
        double3 ccf = relative.Transform(cci2Cce.Inverse())
            .Transform(body.GetCcf2Cci(GameTime.FromSecondsSinceJ2000(t)).Inverse());
        return ccf.Transform(body.GetCcf2Cci(Universe.GetElapsedTime())).Transform(cci2Cce);
    }

    // The parts of the nodes' approach tracks no bright line shows, dimmer
    // (MapColors.Track), so a node is always on a line, as KSA draws the whole
    // orbit it sits on.
    private static void SubmitNodeTracks(IViewport viewport, NodeTracks nodes)
    {
        Camera camera = viewport.GetCamera();
        double now = GameTime.Now;
        foreach (NodeTrack track in nodes.Distinct)
        {
            if (track.Times.Length < 2)
            {
                continue;
            }
            // Where the drawn lines stop following this track.
            double covered = double.NegativeInfinity;
            if (track.Segment == 0)
            {
                if (Predictor.PredictionFor(nodes.Vehicle) is { Times.Length: > 1 } coast)
                {
                    covered = coast.Times[^1];
                }
            }
            else if (Predictor.PlanFor(nodes.Vehicle) != null)
            {
                covered = track.Shared;
            }
            double from = Math.Max(Math.Max(now, track.Times[0]), covered);
            if (!(from < track.End))
            {
                continue;
            }
            byte4 color = MapColors.Dimmed(MapColors.Segment(track.Segment), MapColors.Track);
            Line line = track.Line;
            FrameMapper[] mappers = line.Mappers();
            Ensure(track.Times.Length + 1);
            for (int s = line.StretchAt(from); s < line.Stretches.Length; s++)
            {
                Stretch stretch = line.Stretches[s];
                int count = 0;
                if (stretch.From <= from)
                {
                    _positions[count] = camera.EclToEgo(line.EclAt(mappers, from));
                    _colors[count++] = color;
                }
                for (int i = stretch.Start; i <= stretch.End; i++)
                {
                    if (track.Times[i] <= from)
                    {
                        continue;
                    }
                    _positions[count] = camera.EclToEgo(mappers[s].ToEcl(track.FrameCoordinates[i]));
                    _colors[count++] = color;
                }
                AddLine(viewport, count);
            }
        }
    }

    private static void Ensure(int n)
    {
        if (_positions.Length < n)
        {
            int size = Math.Max(n, _positions.Length * 2);
            _positions = new double3[size];
            _colors = new byte4[size];
            _packed = new float3[size + 512];
            _packedColors = new byte4[size + 512];
        }
    }
}

// The markers of the prediction and the flight plan on the map, hidden behind
// bodies as the lines are.
[HarmonyPatch(typeof(Universe), nameof(Universe.OnDrawUi))]
internal static class MapOverlay
{
    [HarmonyPostfix]
    private static void Postfix(IGameViewport inViewport)
    {
        using var profiled = FrameProfiler.MapOverlay.Measure();
        try
        {
            Draw(inViewport);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[map] drawing the markers", e);
        }
    }

    private static void Draw(IGameViewport viewport)
    {
        if (viewport.Mode != CameraMode.Map)
        {
            return;
        }
        Vehicle? vehicle = Program.ControlledVehicle;
        if (Predictor.PredictionFor(vehicle) is { } prediction)
        {
            DrawMarkers(viewport, prediction);
        }
        if (Predictor.PlanFor(vehicle) is { } plan)
        {
            DrawMarkers(viewport, plan);
        }
    }

    // As KSA's markers: a dot and a name; hovered (KSA's 50 px box for Ap and
    // Pe, 12 px otherwise), the altitude or detail and a countdown.
    private static void DrawMarkers(IGameViewport viewport, Prediction prediction)
    {
        Camera camera = viewport.GetCamera();
        ImDrawListPtr drawList = ImGuiHelper.GetOverlayDrawList(viewport);
        float2 origin = viewport.Position;
        float2 mouse = Cursor.GetPosition(viewport);
        double now = GameTime.Now;
        Line line = prediction.Line;
        FrameMapper[] mappers = line.Mappers();

        float2? Screen(double3 ecl)
        {
            if (Occluded(camera, ecl))
            {
                return null;
            }
            float2 p = camera.EclToScreen(ecl);
            return float.IsNaN(p.X) || float.IsNaN(p.Y) ? null : p;
        }

        void Draw(float2 at, byte4 c, string name, double time, double? altitude, string? detail, bool apsis)
        {
            float2 p = origin + at;
            ImGuiHelper.DrawMarkerDot(drawList, p, c);
            ImGuiHelper.DrawTextOnScreen(drawList, p, name, c);
            bool hovered = apsis ? Math.Abs(at.X - mouse.X) < 50 && Math.Abs(at.Y - mouse.Y) < 50
                                 : float2.Distance(at, mouse) < 12;
            if (!hovered)
            {
                return;
            }
            Span<char> buffer = stackalloc char[128];
            float line = ImGui.GetTextLineHeight();
            float2 row = p + new float2(0, line);
            if (altitude is double a)
            {
                ImGuiHelper.DrawTextOnScreen(drawList, row, new string(DistanceReference.ToNearest(a, buffer)), c);
                row.Y += line;
            }
            if (detail != null)
            {
                ImGuiHelper.DrawTextOnScreen(drawList, row, detail, c);
                row.Y += line;
            }
            ImGuiHelper.DrawTextOnScreen(drawList, row,
                                         Format.Countdown(time - now), c);
        }

        // Only the next Pe and Ap, and every burn and closest approach.
        bool nextPe = false, nextAp = false;
        foreach (var marker in prediction.Markers)
        {
            if (marker.Time < now)
            {
                continue;
            }
            if (marker.Kind == Prediction.MarkerKind.Periapsis)
            {
                if (nextPe) continue;
                nextPe = true;
            }
            else if (marker.Kind == Prediction.MarkerKind.Apoapsis)
            {
                if (nextAp) continue;
                nextAp = true;
            }
            if (Screen(mappers[marker.Stretch].ToEcl(marker.Position)) is not { } at)
            {
                continue;
            }
            // On the flight plan, its segment's colour; encounters and
            // escapes, the line's on the prediction too.
            byte4 color = marker.Kind switch
            {
                Prediction.MarkerKind.Burn => MapColors.Burn,
                _ when prediction.IsFlightPlan => MapColors.PlanAt(prediction, marker.Time),
                Prediction.MarkerKind.Sphere or Prediction.MarkerKind.Body => MapColors.Coast,
                Prediction.MarkerKind.ClosestApproach => MapColors.Approach,
                _ => MapColors.Apsis,
            };
            Draw(at, color, marker.Name, marker.Time, marker.Altitude, marker.Detail,
                 apsis: marker.Kind is Prediction.MarkerKind.Periapsis or Prediction.MarkerKind.Apoapsis);
        }
        int last = prediction.Times.Length - 1;
        if (prediction.Impact && last >= 0 && Screen(line.ToEcl(mappers, last)) is { } impact)
        {
            Draw(impact, MapColors.Impact, "Impact", prediction.Times[last], null, null, apsis: false);
        }
        // Where the danger track ends on an impact: the ground it crashes
        // into, where that ground is now, with an X, in the track's colour.
        if (prediction.Impact && prediction.Danger.Length > 0 && prediction.Danger[^1].Length > 0 &&
            prediction.Frame.Centre is IParentBody body)
        {
            (double t, double3 relative) = prediction.Danger[^1][^1];
            double3 site = prediction.Frame.Centre.GetPositionEcl() + OrbitLines.GroundNowCce(body, t, relative);
            if (t >= now && Screen(site) is { } at)
            {
                byte4 c = (byte4)FlightPlan.DangerColor;
                float2 p = origin + at;
                const float arm = 3.5f;
                drawList.AddLine(p - new float2(arm, arm), p + new float2(arm, arm), c);
                drawList.AddLine(p - new float2(arm, -arm), p + new float2(arm, -arm), c);
                if (float2.Distance(at, mouse) < 12)
                {
                    float row = ImGui.GetTextLineHeight();
                    ImGuiHelper.DrawTextOnScreen(drawList, p + new float2(arm + 4, -row / 2), "Impact site", c);
                    ImGuiHelper.DrawTextOnScreen(drawList, p + new float2(arm + 4, row / 2),
                                                 Format.Countdown(t - now), c);
                }
            }
        }
    }

    // Whether a body lies between the camera and `ecl`.
    internal static bool Occluded(Camera camera, double3 ecl)
    {
        if (PrincipiaSystem.Current is not { } system)
        {
            return false;
        }
        double3 e = camera.EclToEgo(ecl);
        double e2 = double3.Dot(e, e);
        if (!(e2 > 0))
        {
            return false;
        }
        foreach (Astronomical body in system.Bodies)
        {
            double radius = body.MeanRadius;
            double3 b = camera.GetPositionEgo(body);
            double along = double3.Dot(b, e) / e2;
            if (along <= 0 || along >= 1)
            {
                continue;  // Behind the camera, or beyond the point.
            }
            if ((b - along * e).Length() < radius * 0.999)
            {
                return true;
            }
        }
        return false;
    }
}

// KSA's patched-conic lines and markers for the controlled vessel, hidden by
// default while Principia's lines replace them.
internal static class KsaConics
{
    // Whether Principia's lines replace KSA's conics for this vessel: drawing,
    // hover and node drag then never use the conics. Not while Principia isn't
    // driving (multiplayer), when the conics are the vessel's path.
    public static bool Hidden(Vehicle? vehicle) =>
        !Settings.ShowKsaConics && vehicle != null && vehicle == Program.ControlledVehicle &&
        (Settings.ShowPrediction || Settings.ShowFlightPlan) && PrincipiaSystem.Current is { Driving: true };

    // The vessel's flight plan, or one of its burns' post-burn flight plans.
    public static bool Hidden(FlightPlan plan)
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        if (!Hidden(vehicle))
        {
            return false;
        }
        if (plan == vehicle!.FlightPlan)
        {
            return true;
        }
        return vehicle.BurnPlan.All().Any(burn => burn.FlightPlan == plan);
    }
}

// The orbit lines of the vessel's and its burns' flight plans. (Not
// Vehicle.AddLineInstances, which gets inlined.)
[HarmonyPatch(typeof(FlightPlan), nameof(FlightPlan.AddLineInstances))]
internal static class FlightPlanLinesPatch
{
    [HarmonyPrefix]
    private static bool Prefix(IOrbiter orbiter)
    {
        try
        {
            return !KsaConics.Hidden(orbiter as Vehicle);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[conics] hiding KSA's orbit lines", e);
            return true;
        }
    }
}

// The markers on those lines.
[HarmonyPatch(typeof(FlightPlan), nameof(FlightPlan.DrawUi))]
internal static class FlightPlanMarkersPatch
{
    [HarmonyPrefix]
    private static bool Prefix(FlightPlan __instance, ref bool __result)
    {
        try
        {
            if (KsaConics.Hidden(__instance))
            {
                __result = false;  // Nothing hovered.
                return false;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[conics] hiding KSA's orbit markers", e);
        }
        return true;
    }
}
