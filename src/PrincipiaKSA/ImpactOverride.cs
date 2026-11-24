using System.Diagnostics;
using System.Reflection;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// KSA's flight plan ends where a patched conic hits its body, but on a close pass the n-body path can fly by. A crash on
// a later patch of the controlled vessel's own plans is dropped while Principia's path flies on a minute past it. The
// current patch keeps KSA's terrain-aware answer, the one its physics uses, and is checked again when it becomes current.
internal static class ImpactOverride
{
    // This thread's plan is a burn's; how deep it is in the vessel's own plans.
    [ThreadStatic] public static bool BurnPatch;
    [ThreadStatic] public static int VesselPlans;

    private const double Margin = 60;  // Seconds Principia's path must fly on.

    private static long _nextLog;

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
                // Segment 0 is before any burn, like the vessel's own plan.
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

// The vessel's own plans; KSA builds others under its id too (transfer grid, refinement), which keep their crashes.
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
