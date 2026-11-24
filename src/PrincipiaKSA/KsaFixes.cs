using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// Guards against KSA bugs that n-body paths reach and that would abort the game.

// A NaN escape time (near a hyperbola's asymptote or a parabola) throws on the transfer planner's threads, uncaught;
// it now ends the plan as an error, as KSA already does for a NaN true anomaly.
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
