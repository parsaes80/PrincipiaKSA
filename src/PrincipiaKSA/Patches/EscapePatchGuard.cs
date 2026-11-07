using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

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
