using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// The markers of the prediction and the flight plan on the map, hidden behind
// bodies as the lines are; and the PrincipiaKSA window.
[HarmonyPatch(typeof(Universe), nameof(Universe.OnDrawUi))]
internal static class MapOverlay
{
    private static readonly byte4 ApsisColor = new(90, 220, 255, 255);
    private static readonly byte4 ApproachColor = new(255, 90, 220, 255);
    private static readonly byte4 BurnColor = new(255, 230, 80, 255);
    private static readonly byte4 ImpactColor = new(255, 60, 60, 255);

    [HarmonyPostfix]
    private static void Postfix(IGameViewport inViewport)
    {
        using var profiled = FrameProfiler.MapOverlay.Measure();
        // Once per frame (StarMap's [StarMapAfterGui] isn't called).
        if (inViewport == Program.MainViewport)
        {
            try
            {
                using var window = FrameProfiler.Window.Measure();
                SettingsWindow.Draw();
            }
            catch (Exception e)
            {
                Log.Warn($"PrincipiaKSA window failed: {e.Message}");
            }
        }
        try
        {
            Draw(inViewport);
        }
        catch (Exception e)
        {
            Log.Warn($"[map] drawing failed, prediction hidden: {e.Message}");
            Settings.ShowPrediction = Settings.ShowFlightPlan = false;
        }
    }

    private static void Draw(IGameViewport viewport)
    {
        if (viewport.Mode != CameraMode.Map)
        {
            return;
        }
        Vehicle? vehicle = Program.ControlledVehicle;
        if (Settings.ShowPrediction && Predictor.Latest is { } prediction && prediction.Vehicle == vehicle)
        {
            DrawMarkers(viewport, prediction);
        }
        if (Settings.ShowFlightPlan && Predictor.LatestPlan is { } plan && plan.Vehicle == vehicle)
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
        double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
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
                                         "T-" + new string(TimeSpanReference.ToBreakdown(time - now, buffer)), c);
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
                Prediction.MarkerKind.Burn => BurnColor,
                _ when prediction.IsFlightPlan => OrbitLines.PlanColorAt(prediction, marker.Time),
                Prediction.MarkerKind.Sphere or Prediction.MarkerKind.Body => OrbitLines.CoastColor,
                Prediction.MarkerKind.ClosestApproach => ApproachColor,
                _ => ApsisColor,
            };
            Draw(at, color, marker.Name, marker.Time, marker.Altitude, marker.Detail,
                 apsis: marker.Kind is Prediction.MarkerKind.Periapsis or Prediction.MarkerKind.Apoapsis);
        }
        int last = prediction.Times.Length - 1;
        if (prediction.Impact && last >= 0 && Screen(line.ToEcl(mappers, last)) is { } impact)
        {
            Draw(impact, ImpactColor, "Impact", prediction.Times[last], null, null, apsis: false);
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
                    Span<char> buffer = stackalloc char[128];
                    float row = ImGui.GetTextLineHeight();
                    ImGuiHelper.DrawTextOnScreen(drawList, p + new float2(arm + 4, -row / 2), "Impact site", c);
                    ImGuiHelper.DrawTextOnScreen(drawList, p + new float2(arm + 4, row / 2),
                                                 "T-" + new string(TimeSpanReference.ToBreakdown(t - now, buffer)), c);
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

    public static string Duration(double seconds)
    {
        var t = TimeSpan.FromSeconds(Math.Max(0, seconds));
        return t.TotalDays >= 1 ? $"{(int)t.TotalDays}d {t.Hours:00}:{t.Minutes:00}:{t.Seconds:00}"
                                : $"{t.Hours:00}:{t.Minutes:00}:{t.Seconds:00}";
    }
}
