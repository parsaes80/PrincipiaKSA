using System.Reflection;
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

// KSA calls an orbit parabolic (no period) by its energy but closed by e < 1; down to e ≈ 0.999996 both hold, and the
// closed-orbit path divides by the missing period and aborts the game. A parabolic orbit gets e = 1 exactly.
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
