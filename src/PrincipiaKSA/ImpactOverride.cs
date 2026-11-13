using System.Diagnostics;
using System.Reflection;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// KSA's flight plan, and with it every burn, ends where a patched conic hits
// its primary body. On a close pass, KSA's conic can crash where the n-body
// path flies by. For the controlled vessel, a crash KSA finds on a later patch
// is dropped when Principia's path (the prediction for the vessel's own plan,
// the flight plan for a burn's, or a node's track) is still flying a minute
// after it; KSA then carries its plan on as after a miss.
//
// The current patch always keeps KSA's terrain-aware answer: it is the only
// one KSA's physics acts on, and near the body it is accurate. A later patch
// is checked again when it becomes current. Plans KSA builds for other uses
// keep its crashes, even under the vessel's id (see VesselPlanScope).
internal static class ImpactOverride
{
    // Whether the plan being computed on this thread is a burn's.
    [ThreadStatic] public static bool BurnPatch;

    // How deep this thread is in computing the vessel's own plans.
    [ThreadStatic] public static int VesselPlans;

    private const double Margin = 60;  // Seconds Principia's path must fly on.

    private static long _nextLog;

    // Whether Principia's path for this plan is still flying `Margin` after t.
    public static bool PrincipiaClears(Vehicle vehicle, bool burnPatch, double t)
    {
        double need = t + Margin;
        bool Past(double[]? times) => times is { Length: > 1 } && times[0] <= t && times[^1] >= need;

        Prediction? line = burnPatch ? Predictor.PlanFor(vehicle) : Predictor.PredictionFor(vehicle);
        if (line != null && Past(line.Times))
        {
            return true;
        }
        if (Predictor.NodesFor(vehicle) is { } nodes)
        {
            foreach (NodeTrack track in nodes.ByBurn.Values)
            {
                // Segment 0 is before any burn, as the vessel's own plan.
                if ((track.Segment > 0) == burnPatch && Past(track.Times))
                {
                    return true;
                }
            }
        }
        return false;
    }

    public static void Logged(PatchedConic patch, double t)
    {
        long now = Stopwatch.GetTimestamp();
        if (now < _nextLog)
        {
            return;
        }
        _nextLog = now + 10 * Stopwatch.Frequency;
        double ahead = (t - GameTime.Now) / 3600;
        Log.Info($"[impact] KSA's conics put the vessel into {(patch.PrimaryBody as IObjectId)?.Id} in {ahead:F1} h; " +
                 "Principia's path flies on, so KSA's plan carries on past it");
    }
}

[HarmonyPatch(typeof(FlightPlan), "CalculateConicPatch")]
internal static class CalculateConicPatchContext
{
    [HarmonyPrefix]
    private static void Prefix(bool burnPatch) => ImpactOverride.BurnPatch = burnPatch;

    [HarmonyFinalizer]
    private static void Finalizer() => ImpactOverride.BurnPatch = false;
}

// Where KSA computes the vessel's own plan and its burns' plans. KSA builds
// other plans under the vessel's id too (the transfer planner's grid, burn
// refinement, interstellar replanning), which Principia's path says nothing
// about.
[HarmonyPatch]
internal static class VesselPlanScope
{
    [HarmonyTargetMethods]
    private static IEnumerable<MethodBase> TargetMethods() =>
    [
        AccessTools.Method(typeof(VehicleUpdateState), nameof(VehicleUpdateState.RecalculateFlightPlan)),
        AccessTools.Method(typeof(BurnPlan), nameof(BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly)),
    ];

    [HarmonyPrefix]
    private static void Prefix() => ImpactOverride.VesselPlans++;

    [HarmonyFinalizer]
    private static void Finalizer() => ImpactOverride.VesselPlans--;
}

[HarmonyPatch(typeof(PatchedConic), nameof(PatchedConic.SearchForParentCollision))]
internal static class SearchForParentCollisionPatch
{
    [HarmonyPrefix]
    private static void Prefix(PatchedConic __instance,
                               out (UniverseTime End, TrueAnomaly EndAnomaly, PatchTransition Transition,
                                    UniverseTime Frontier) __state)
    {
        __state = (__instance.EndTime, __instance.EndTrueAnomaly, __instance.EndTransition,
                   __instance.ImpactSearchFrontier);
    }

    [HarmonyPostfix]
    private static void Postfix(PatchedConic __instance, ref ImpactSearchResult __result,
                                ref UniverseTime impactSearchFrontier,
                                (UniverseTime End, TrueAnomaly EndAnomaly, PatchTransition Transition,
                                 UniverseTime Frontier) __state)
    {
        using var profiled = FrameProfiler.ImpactOverride.Measure();
        try
        {
            bool burnPatch = ImpactOverride.BurnPatch;
            if (__result != ImpactSearchResult.Impact || ImpactOverride.VesselPlans == 0 ||
                (!burnPatch && __instance.StartTransition == PatchTransition.Initial) ||
                Program.ControlledVehicle is not { } vehicle || !__instance.IdHash.Equals(vehicle.Hash) ||
                !KsaConics.Hidden(vehicle))
            {
                return;
            }
            double t = GameTime.ToSecondsSinceJ2000(__instance.EndTime);
            if (!double.IsFinite(t) || !ImpactOverride.PrincipiaClears(vehicle, burnPatch, t))
            {
                return;
            }
            (__instance.EndTime, __instance.EndTrueAnomaly, __instance.EndTransition,
             __instance.ImpactSearchFrontier) = __state;
            __result = ImpactSearchResult.Clear;
            impactSearchFrontier = UniverseTime.EndOfTime;
            ImpactOverride.Logged(__instance, t);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[impact] checking KSA's crash against Principia", e);
        }
    }
}
