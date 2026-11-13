using System.Reflection;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// Guards against KSA's own bugs that n-body paths reach and that would
// otherwise abort the game.

// KSA's time to leave a sphere of influence can come out NaN at the edge of
// its orbit formulas (an escape near the asymptote of a hyperbola, an orbit
// near parabolic), and constructing a UniverseTime from it throws. On the
// transfer planner's worker threads nothing catches it and the game aborts.
// A NaN true anomaly a few lines earlier already ends the plan as an error;
// a NaN time now ends it the same way.
[HarmonyPatch(typeof(FlightPlan), nameof(FlightPlan.CalculateEscapePatch))]
internal static class EscapePatchGuard
{
    [HarmonyFinalizer]
    private static Exception? Finalizer(Exception? __exception, PatchedConic patch, ref PatchedConic? nextPatch,
                                        ref bool error, ref bool __result)
    {
        if (__exception is not ArgumentException)
        {
            return __exception;
        }
        patch.EndTransition = PatchTransition.Final;
        nextPatch = null;
        error = true;
        __result = false;
        Log.WarnOnce("[conics] KSA's escape from a sphere of influence (plan ended there)", __exception);
        return null;
    }
}

// KSA classes an orbit as parabolic when its energy is within a millionth of
// zero, and gives it no period, but elsewhere tells a closed orbit by
// eccentricity < 1. Near periapsis that window reaches down to e ≈ 0.999996;
// an orbit there takes the closed-orbit path, divides by the missing period
// and throws "Cannot construct a UniverseTime from NaN" from
// Vehicle.UpdatePerFrameData, aborting the game, when a burn or the n-body
// path crosses escape. A parabolic orbit gets its exact eccentricity, 1, so
// both tests agree; its shape comes from the periapsis and doesn't change.
[HarmonyPatch]
internal static class ParabolicOrbitPatch
{
    private static readonly AccessTools.FieldRef<Orbit, OrbitData> Data =
        AccessTools.FieldRefAccess<Orbit, OrbitData>("_data");

    [HarmonyTargetMethods]
    private static IEnumerable<MethodBase> TargetMethods() =>
    [
        AccessTools.Method(typeof(Orbit), nameof(Orbit.Create)),
        AccessTools.Method(typeof(Orbit), nameof(Orbit.CreateFromStateCci)),
    ];

    [HarmonyPostfix]
    private static void Postfix(Orbit __result)
    {
        if (__result is not Orbit.Parabolic || !(__result.Eccentricity < 1))
        {
            return;
        }
        ref OrbitData d = ref Data(__result);
        d = new OrbitData(d.TimeAtPeriapsis, 1.0, d.SemiMajorAxis, d.SemiMinorAxis, d.Inclination,
                          d.LongitudeOfAscendingNode, d.ArgumentOfPeriapsis, d.Period, d.Periapsis, d.Apoapsis,
                          d.Orb2ParentCci);
    }
}
