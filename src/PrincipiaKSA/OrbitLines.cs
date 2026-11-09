using Brutal.Numerics;
using HarmonyLib;
using KSA;
using PrincipiaKSA.Patches;

namespace PrincipiaKSA;

// Maps a plotting frame's coordinates to Ecl positions now: about each body
// where it is now, as KSA draws its own orbit lines.
internal readonly struct FrameMapper
{
    private readonly double3 _origin, _x, _y, _z;
    private readonly doubleQuat _rotation;
    private readonly PlotFrame _kind;

    public FrameMapper(FrameSpec frame)
    {
        _kind = frame.Kind;
        _x = _y = _z = double3.Zero;
        _rotation = doubleQuat.Identity;
        switch (frame.Kind)
        {
            case PlotFrame.BodySurface:
                _origin = frame.Centre.GetPositionEcl();
                _rotation = ((IParentBody)frame.Centre).GetCcf2Cce(Universe.GetElapsedTime());
                break;
            case PlotFrame.BarycentricRotating:
                (_origin, _x, _y, _z) = Predictor.RotatingAxes(
                    ((IParentBody)frame.Centre).Mu, frame.Centre.GetPositionEcl(), frame.Centre.GetVelocityEcl(),
                    ((IParentBody)frame.Secondary!).Mu, frame.Secondary!.GetPositionEcl(),
                    frame.Secondary!.GetVelocityEcl());
                break;
            default:
                _origin = frame.Centre.GetPositionEcl();
                break;
        }
    }

    public double3 ToEcl(double3 c) => _kind switch
    {
        PlotFrame.BodySurface => _origin + c.Transform(_rotation),
        PlotFrame.BarycentricRotating => _origin + c.X * _x + c.Y * _y + c.Z * _z,
        _ => _origin + c,
    };
}

// Draws Principia's lines with KSA's own 3D orbit-line pass, so planets hide
// the parts behind them, queued where KSA queues its own (CelestialSystem
// rather than Universe.OnPreRender, which may be inlined).
[HarmonyPatch(typeof(CelestialSystem), nameof(CelestialSystem.OnPreRender))]
internal static class OrbitLines
{
    private static readonly byte4 BurnColor = new(255, 230, 80, 255);

    // The prediction orange, then one colour per burn, cycling after ten: hues
    // at least 80° apart from one segment to the next, as saturated as sRGB
    // allows at OKLCH lightness 0.66–0.80, no yellows (the burns').
    private static readonly byte4[] SegmentColors =
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

    public static byte4 CoastColor => SegmentColors[0];

    // Main thread only.
    private static double3[] _positions = new double3[1024];
    private static byte4[] _colors = new byte4[1024];
    private static float3[] _packed = new float3[1024 + 512];
    private static byte4[] _packedColors = new byte4[1024 + 512];

    [HarmonyPostfix]
    private static void Postfix(IViewport inViewport)
    {
        try
        {
            if (inViewport.Mode != CameraMode.Map || Program.ControlledVehicle is not { } vehicle)
            {
                return;
            }
            if (Settings.ShowPrediction && Predictor.Latest is { } prediction && prediction.Vehicle == vehicle)
            {
                Submit(inViewport, prediction);
                SubmitDanger(inViewport, prediction);
            }
            if (Settings.ShowFlightPlan && Predictor.LatestPlan is { } plan && plan.Vehicle == vehicle)
            {
                Submit(inViewport, plan);
                SubmitDanger(inViewport, plan);
            }
            if (Predictor.LatestNodes is { } nodes && nodes.Vehicle == vehicle && KsaConics.Hidden(vehicle))
            {
                SubmitNodeTracks(inViewport, nodes);
            }
        }
        catch (Exception e)
        {
            Log.Warn($"[map] orbit line failed, prediction hidden: {e.Message}");
            Settings.ShowPrediction = Settings.ShowFlightPlan = false;
        }
    }

    // A prediction from the vessel on, or a flight plan from its first burn:
    // a line per stretch, each in its frame.
    private static void Submit(IViewport viewport, Prediction prediction)
    {
        Camera camera = viewport.GetCamera();
        double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
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
                    _colors[count++] = !prediction.IsFlightPlan ? CoastColor
                                                                : PlanColor(prediction, prediction.FirstIgnition + 1);
                }
                _positions[count] = camera.EclToEgo(mappers[s].ToEcl(prediction.FrameCoordinates[i]));
                _colors[count++] = !prediction.IsFlightPlan ? CoastColor : PlanColor(prediction, i);
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
    // its colour, dimmer, as KSA shows the path on through an encounter.
    private const byte GhostAlpha = 110;

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
            byte4 c = prediction.IsFlightPlan ? PlanColorAt(prediction, times[i]) : CoastColor;
            _positions[count] = camera.EclToEgo(mapper.ToEcl(points[i]));
            _colors[count++] = new byte4(c.X, c.Y, c.Z, GhostAlpha);
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
        double nowSeconds = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
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

    // The parts of the nodes' approach tracks no bright line shows, dimmer, so
    // a node is always on a line, as KSA draws the whole orbit it sits on.
    private const byte TrackAlpha = 120;

    private static void SubmitNodeTracks(IViewport viewport, NodeTracks nodes)
    {
        Camera camera = viewport.GetCamera();
        double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
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
                if (Settings.ShowPrediction && Predictor.Latest is { Times.Length: > 1 } coast &&
                    coast.Vehicle == nodes.Vehicle)
                {
                    covered = coast.Times[^1];
                }
            }
            else if (Settings.ShowFlightPlan && Predictor.LatestPlan is { } plan && plan.Vehicle == nodes.Vehicle)
            {
                covered = track.Shared;
            }
            double from = Math.Max(Math.Max(now, track.Times[0]), covered);
            if (!(from < track.End))
            {
                continue;
            }
            byte4 c = SegmentColors[track.Segment % SegmentColors.Length];
            byte4 color = new(c.X, c.Y, c.Z, TrackAlpha);
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

    // The flight plan's colour at point i: yellow under thrust, else its
    // segment's.
    public static byte4 PlanColor(Prediction plan, int i)
    {
        int segment = 0;
        foreach (var (start, end) in plan.BurnArcs)
        {
            if (i > start && i <= end)
            {
                return BurnColor;
            }
            if (end < i)
            {
                segment++;
            }
        }
        return SegmentColors[segment % SegmentColors.Length];
    }

    // The same at time t, for markers.
    public static byte4 PlanColorAt(Prediction plan, double t)
    {
        int i = Array.BinarySearch(plan.Times, t);
        return PlanColor(plan, i >= 0 ? i : Math.Min(~i, plan.Times.Length - 1));
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
