using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

// KSA's per-body job computes the body's next state from its conic, on worker
// threads; the state is replaced with Principia's.
[HarmonyPatch(typeof(CelestialUpdateTask), "DoWorkAndStageResults")]
internal static class CelestialUpdatePatch
{
    [HarmonyPostfix]
    // Private fields, injected by name ("___" + field).
    private static void Postfix(CelestialUpdateTask __instance,
                                IOrbiter ____readOnlyCelestial,
                                SimStep ____simStep)
    {
        using var profiled = FrameProfiler.PlanetState.Measure();
        if (____simStep.DeltaTime <= 0)
        {
            return;  // Paused: KSA leaves the state as it is.
        }
        if (CelestialDriver.StateFor(____readOnlyCelestial, ____simStep.NextTime) is { } state)
        {
            __instance.NewStateVectors = state;
        }
    }
}
