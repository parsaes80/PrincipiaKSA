using System.Runtime.CompilerServices;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

// KSA creates maneuver nodes where the mouse hovers its patched-conic lines.
// With those hidden, the hover follows Principia's lines instead, handing KSA
// a point of its own patch at the hovered time, placed on our line. KSA turns
// a point into a burn time by the next pass through it, which only covers one
// revolution, so each point handed over carries its exact time (Remember),
// returned by CrossingTimePatch.
internal static class OrbitHover
{
    // Recent points handed to KSA, by patch and position (which KSA passes on
    // unchanged), with their exact times.
    private static readonly Dictionary<(PatchedConic, double3), UniverseTime> Exact = new(new KeyComparer());
    private static readonly Queue<(PatchedConic, double3)> ExactOrder = new();

    public static void Remember(PatchedConic patch, double3 positionCce, UniverseTime time)
    {
        var key = (patch, positionCce);
        if (!Exact.ContainsKey(key))
        {
            ExactOrder.Enqueue(key);
            while (ExactOrder.Count > 512)
            {
                Exact.Remove(ExactOrder.Dequeue());
            }
        }
        Exact[key] = time;
    }

    // By patch identity: KSA's PatchedConic equality has no matching hash.
    private sealed class KeyComparer : IEqualityComparer<(PatchedConic, double3)>
    {
        public bool Equals((PatchedConic, double3) a, (PatchedConic, double3) b) =>
            ReferenceEquals(a.Item1, b.Item1) && a.Item2.Equals(b.Item2);

        public int GetHashCode((PatchedConic, double3) key) =>
            HashCode.Combine(RuntimeHelpers.GetHashCode(key.Item1), key.Item2);
    }

    public static bool TryExact(in CelestialPosition position, out UniverseTime time)
    {
        time = default;
        return position.Patch is { } patch && Exact.TryGetValue((patch, position.Point.PositionCce), out time);
    }

    // Whether the cursor is on the vessel as KSA tests it (Astronomical.
    // UpdateMouseHover): its marker's box, or its disc when larger. KSA lets
    // a point of a vessel's own line under the cursor win over the vessel;
    // ours starts at the vessel, so there the vessel is left to be clicked.
    public static bool OnVessel(Vehicle vehicle, IViewport viewport, Camera camera, float2 mouse)
    {
        if (!vehicle.CursorHoverEligible)
        {
            return false;
        }
        double3 ego = camera.GetPositionEgo(vehicle);
        float2 screen = camera.EgoToScreen(ego);
        if (float.IsNaN(screen.X) || float.IsNaN(screen.Y))
        {
            return false;
        }
        float radius = (float)(camera.GetObjectDiameterPixels(2 * vehicle.MeanRadius, ego.Length()) * 0.5);
        float pad = MathF.Max(viewport.Size.Y * 0.005f, 8f);
        float half = radius + pad;
        if (MathF.Sqrt(2) * half < 4 * pad)
        {
            return MathF.Abs(mouse.X - screen.X) <= half && MathF.Abs(mouse.Y - screen.Y) <= half;
        }
        return float2.Distance(mouse, screen) <= radius;
    }

    // The point of `patches` under the mouse on `line`, from `from` on.
    public static CelestialPosition? Find(Prediction line, IReadOnlyList<PatchedConic> patches, UniverseTime from,
                                          IViewport viewport, Camera camera, float2 mouse, float grabRadius)
    {
        // Only where the line is drawn: OrbitLines draws in map view only.
        if (viewport.Mode != CameraMode.Map)
        {
            return null;
        }
        double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
        double start = Math.Max(now, GameTime.ToSecondsSinceJ2000(from));
        FrameMapper[] mappers = line.Line.Mappers();
        int2 size = viewport.Size;
        if (size.X <= 0 || size.Y <= 0)
        {
            return null;
        }

        double bestDistance = grabRadius;
        (double Time, double3 Ecl, PatchedConic Patch)? best = null;
        for (int p = 0; p < patches.Count; p++)
        {
            PatchedConic patch = patches[p];
            if (patch.IsAllBurn)
            {
                continue;
            }
            double patchStart = GameTime.ToSecondsSinceJ2000(patch.StartTime);
            double patchEnd = GameTime.ToSecondsSinceJ2000(patch.EndTime);
            // Within the patch, or KSA deletes the burn.
            double lo = Math.Max(start, patchStart);
            double hi = patchEnd - 1e-3;
            if (!(hi > lo))
            {
                continue;
            }
            Nearest(line.Line, line.FirstIgnition, mappers, lo, hi, viewport, camera, mouse, size, patch,
                    ref bestDistance, ref best);
        }
        if (best is not { } hit)
        {
            return null;
        }

        // KSA's point at that time on its patch, moved onto our line.
        PatchedConic chosen = hit.Patch;
        Orbit orbit = chosen.Orbit;
        OrbitPointCce k = orbit.GetPointAt(GameTime.FromSecondsSinceJ2000(hit.Time));
        double3 cce = hit.Ecl - orbit.Parent.GetPositionEclFromCce(double3.Zero);
        var point = new OrbitPointCce(cce, k.TimeSincePe, k.RemainingTimeTo, k.TrueAnomaly, k.InDangerZone);
        Remember(chosen, cce, GameTime.FromSecondsSinceJ2000(hit.Time));
        return new CelestialPosition(orbit, orbit.Parent, point, chosen);
    }

    // The point of the line between times lo and hi nearest the mouse on
    // screen, within the grab radius as KSA measures it (Orbit.GetNearestPoint).
    // Never across a stretch's end: the line isn't joined there.
    public static void Nearest(Line line, int first, FrameMapper[] mappers, double lo, double hi,
                               IViewport viewport, Camera camera, float2 mouse, int2 size, PatchedConic patch,
                               ref double bestDistance, ref (double, double3, PatchedConic)? best)
    {
        for (int s = 0; s < line.Stretches.Length; s++)
        {
            Stretch stretch = line.Stretches[s];
            if (stretch.To > lo && stretch.From < hi)
            {
                Nearest(line.Times, line.Coordinates, Math.Max(first, stretch.Start + 1), stretch.End, mappers[s],
                        lo, hi, camera, mouse, size, patch, ref bestDistance, ref best);
            }
        }
    }

    private static void Nearest(double[] times, double3[] coordinates, int first, int last, FrameMapper mapper,
                                double lo, double hi, Camera camera, float2 mouse, int2 size, PatchedConic patch,
                                ref double bestDistance, ref (double, double3, PatchedConic)? best)
    {
        double aspect = size.X / size.Y;
        for (int i = Math.Max(first, 1); i <= last; i++)
        {
            double t0 = times[i - 1], t1 = times[i];
            if (t1 <= lo || t0 >= hi || !(t1 > t0))
            {
                continue;
            }
            // The segment clipped to [lo, hi].
            double s0 = Math.Max(0, (lo - t0) / (t1 - t0)), s1 = Math.Min(1, (hi - t0) / (t1 - t0));
            double3 a = mapper.ToEcl(double3.Lerp(coordinates[i - 1], coordinates[i], s0));
            double3 b = mapper.ToEcl(double3.Lerp(coordinates[i - 1], coordinates[i], s1));
            float2 sa = camera.EclToScreen(a), sb = camera.EclToScreen(b);
            if (float.IsNaN(sa.X) || float.IsNaN(sa.Y) || float.IsNaN(sb.X) || float.IsNaN(sb.Y))
            {
                continue;  // Behind the camera.
            }
            double2 d = new(sb.X - sa.X, sb.Y - sa.Y);
            double2 m = new(mouse.X - sa.X, mouse.Y - sa.Y);
            double dd = d.X * d.X + d.Y * d.Y;
            double u = dd > 0 ? Math.Clamp((m.X * d.X + m.Y * d.Y) / dd, 0, 1) : 0;
            double dx = (sa.X + u * d.X - mouse.X) * 2 / size.X * aspect;
            double dy = (sa.Y + u * d.Y - mouse.Y) * 2 / size.Y;
            double distance = Math.Sqrt(dx * dx + dy * dy);
            if (distance > bestDistance)
            {
                continue;
            }
            double3 ecl = double3.Lerp(a, b, u);
            if (MapOverlay.Occluded(camera, ecl))
            {
                continue;
            }
            bestDistance = distance;
            best = (t0 + (t1 - t0) * (s0 + (s1 - s0) * u), ecl, patch);
        }
    }
}

// The vessel's own patches: the prediction.
[HarmonyPatch(typeof(CelestialSystem), "FindNearestFlightPlanPoint")]
internal static class FlightPlanHoverPatch
{
    [HarmonyPrefix]
    private static bool Prefix(Vehicle vehicle, IViewport inViewPort, Camera camera, float2 mousePosScreen,
                               float grabRadiusPercent, ref OrbitHoverCandidate? selected)
    {
        using var profiled = FrameProfiler.LineHover.Measure();
        try
        {
            if (!KsaConics.Hidden(vehicle))
            {
                return true;
            }
            if (OrbitHover.OnVessel(vehicle, inViewPort, camera, mousePosScreen))
            {
                return false;
            }
            if (Settings.ShowPrediction && Predictor.Latest is { } prediction && prediction.Vehicle == vehicle)
            {
                UniverseTime now = Universe.GetElapsedTime();
                FlightPlan plan = vehicle.FlightPlan;
                if (OrbitHover.Find(prediction, plan.Patches, now, inViewPort, camera, mousePosScreen,
                                    grabRadiusPercent) is { } position)
                {
                    var candidate = new OrbitHoverCandidate(
                        position, vehicle, plan.HoveredMarker,
                        position.IsReachedWithin(now, vehicle.BurnPlan.FindBranchEndTime(-1)));
                    if (candidate.Wins(in selected, camera, mousePosScreen))
                    {
                        selected = candidate;
                    }
                }
            }
            return false;
        }
        catch (Exception e)
        {
            Log.WarnOnce("[hover] prediction hover", e);
            return false;
        }
    }
}

// Each burn's post-burn patches: the flight plan once a burn with Δv has
// come, else the prediction.
[HarmonyPatch(typeof(BurnPlan), nameof(BurnPlan.FindNearestBurnPoint))]
internal static class BurnHoverPatch
{
    [HarmonyPrefix]
    private static bool Prefix(BurnPlan __instance, Vehicle owner, IViewport inViewPort, Camera camera,
                               float2 mousePos, bool hoversMarker, ref OrbitHoverCandidate? selected)
    {
        using var profiled = FrameProfiler.NodeHover.Measure();
        try
        {
            if (!KsaConics.Hidden(owner))
            {
                return true;
            }
            if (OrbitHover.OnVessel(owner, inViewPort, camera, mousePos))
            {
                return false;
            }
            Prediction? coast = Settings.ShowPrediction && Predictor.Latest is { } c && c.Vehicle == owner ? c : null;
            Prediction? plan =
                Settings.ShowFlightPlan && Predictor.LatestPlan is { } p && p.Vehicle == owner ? p : null;
            for (int i = 0; i < __instance.BurnCount; i++)
            {
                if (!__instance.TryGetBurn(i, out Burn? burn) || burn == null || burn.ParentDepartureBurn)
                {
                    continue;  // A departure's preview: KSA's conic, hidden.
                }
                Prediction? line = AnyDeltaVBy(__instance, burn.Time) ? plan : coast;
                if (line == null)
                {
                    continue;
                }
                UniverseTime branchEnd =
                    burn.ParentDepartureBurn ? UniverseTime.EndOfTime : __instance.FindBranchEndTime(i);
                if (OrbitHover.Find(line, burn.FlightPlan.Patches, burn.Time, inViewPort, camera, mousePos,
                                    BurnContextMenu.GetGrabRadiusPercent(owner)) is { } position)
                {
                    var candidate = new OrbitHoverCandidate(position, owner, hoversMarker,
                                                            position.IsReachedWithin(burn.Time, branchEnd));
                    if (candidate.Wins(in selected, camera, mousePos))
                    {
                        selected = candidate;
                    }
                }
            }
            return false;
        }
        catch (Exception e)
        {
            Log.WarnOnce("[hover] flight plan hover", e);
            return false;
        }
    }

    // A burn with Δv at or before `time`.
    private static bool AnyDeltaVBy(BurnPlan burns, UniverseTime time)
    {
        for (int i = 0; i < burns.BurnCount; i++)
        {
            if (burns.TryGetBurn(i, out Burn? b) && b != null && b.HasDeltaV && !b.IsInterstellar &&
                Predictor.VesselTime(b) <= time)
            {
                return true;
            }
        }
        return false;
    }
}

// The right-click menu's "At Periapsis" / "At Apoapsis", offered within
// 44 px of the next Pe and Ap: Principia's markers and times, not the conic's.
[HarmonyPatch(typeof(BurnContextMenu), "ComputeSnapState")]
internal static class BurnMenuSnapPatch
{
    private static readonly AccessTools.FieldRef<bool> NearPeriapsis =
        AccessTools.StaticFieldRefAccess<bool>(AccessTools.Field(typeof(BurnContextMenu), "_burnMenuNearPeriapsis"));
    private static readonly AccessTools.FieldRef<bool> NearApoapsis =
        AccessTools.StaticFieldRefAccess<bool>(AccessTools.Field(typeof(BurnContextMenu), "_burnMenuNearApoapsis"));
    private static readonly AccessTools.FieldRef<UniverseTime?> PeriapsisTime =
        AccessTools.StaticFieldRefAccess<UniverseTime?>(
            AccessTools.Field(typeof(BurnContextMenu), "_burnMenuPeriapsisTime"));
    private static readonly AccessTools.FieldRef<UniverseTime?> ApoapsisTime =
        AccessTools.StaticFieldRefAccess<UniverseTime?>(
            AccessTools.Field(typeof(BurnContextMenu), "_burnMenuApoapsisTime"));

    [HarmonyPostfix]
    private static void Postfix(in CelestialPosition point, IViewport viewport)
    {
        using var profiled = FrameProfiler.BurnMenuSnap.Measure();
        try
        {
            if (point.Patch is not { } patch || !KsaConics.Hidden(Program.ControlledVehicle))
            {
                return;
            }
            NearPeriapsis() = NearApoapsis() = false;
            PeriapsisTime() = ApoapsisTime() = null;
            Camera camera = viewport.GetCamera();
            float2 mouse = Cursor.GetPosition(viewport);
            double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
            double start = Math.Max(now, GameTime.ToSecondsSinceJ2000(patch.StartTime));
            double end = GameTime.ToSecondsSinceJ2000(patch.EndTime);
            // The prediction for the vessel's own patches, the plan for a burn's.
            Vehicle vehicle = Program.ControlledVehicle!;
            Prediction? line = vehicle.FlightPlan.Patches.Contains(patch)
                ? (Settings.ShowPrediction ? Predictor.Latest : null)
                : (Settings.ShowFlightPlan ? Predictor.LatestPlan : null) ??
                  (Settings.ShowPrediction ? Predictor.Latest : null);
            if (line == null || line.Vehicle != vehicle)
            {
                return;
            }
            FrameMapper[] mappers = line.Line.Mappers();
            bool pe = false, ap = false;
            foreach (Prediction.Marker m in line.Markers)
            {
                bool isPe = m.Kind == Prediction.MarkerKind.Periapsis;
                bool isAp = m.Kind == Prediction.MarkerKind.Apoapsis;
                if (!(isPe || isAp) || m.Time < now || (isPe && pe) || (isAp && ap))
                {
                    continue;  // Only the next of each, as drawn.
                }
                pe |= isPe;
                ap |= isAp;
                if (m.Time < start || m.Time > end ||
                    !(float2.Distance(mouse, camera.EclToScreen(mappers[m.Stretch].ToEcl(m.Position))) < 44f))
                {
                    continue;
                }
                UniverseTime time = GameTime.FromSecondsSinceJ2000(m.Time);
                // KSA builds the menu's point at this time on its patch.
                OrbitHover.Remember(patch, patch.Orbit.GetPointAt(time).PositionCce, time);
                if (isPe)
                {
                    NearPeriapsis() = true;
                    PeriapsisTime() = time;
                }
                else
                {
                    NearApoapsis() = true;
                    ApoapsisTime() = time;
                }
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[hover] burn menu Pe/Ap", e);
        }
    }
}

// The time KSA gives a hovered point: ours, exactly, for the points we handed
// it. (This private method, as TryGetOrbiterCrossingTime may be inlined.)
[HarmonyPatch(typeof(CelestialPosition), "TryGetVehicleCrossingTime")]
internal static class CrossingTimePatch
{
    [HarmonyPrefix]
    private static bool Prefix(ref CelestialPosition __instance, ref UniverseTime gameTime, ref bool __result)
    {
        using var profiled = FrameProfiler.NodeTime.Measure();
        try
        {
            if (OrbitHover.TryExact(in __instance, out UniverseTime exact))
            {
                gameTime = exact;
                __result = exact > Universe.GetElapsedTime();
                return false;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[hover] burn time of a hovered point", e);
        }
        return true;
    }
}
