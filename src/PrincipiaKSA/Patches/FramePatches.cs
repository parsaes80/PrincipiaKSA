using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

// KSA picks the step its jobs will simulate: integrate that far first, and
// keep a few real seconds ahead at the current warp.
[HarmonyPatch(typeof(Universe), nameof(Universe.GetJobSimStep))]
internal static class GetJobSimStepPatch
{
    public static long LastProlongTicks;

    [HarmonyPostfix]
    private static void Postfix(double dtPlayer, SimStep __result)
    {
        PrincipiaSystem? system = PrincipiaSystem.Current;
        if (system == null || system.Failed)
        {
            return;
        }
        try
        {
            double next = GameTime.ToSecondsSinceJ2000(__result.NextTime);
            long start = System.Diagnostics.Stopwatch.GetTimestamp();
            using (FrameProfiler.ProlongToNextStep.Measure())
            {
                system.Prolonger.EnsureProlonged(next);
            }
            LastProlongTicks = System.Diagnostics.Stopwatch.GetTimestamp() - start;
            double simSecondsPerRealSecond = __result.DeltaTime / Math.Max(dtPlayer, 1e-3);
            // A few real seconds ahead at this warp, and the planets kept
            // integrated well ahead for the lines (Prolonger.Horizon).
            system.Prolonger.Request(next + Math.Max(Prolonger.Horizon, 3 * simSecondsPerRealSecond));
            using (FrameProfiler.BurnTargets.Measure())
            {
                BurnTargetRefresher.Tick();
            }
        }
        catch (Exception e)
        {
            system.Fail($"prolonging to the next step: {e.Message}");
        }
    }
}

// Between frames, when no job runs: conics can be replaced safely.
[HarmonyPatch(typeof(Universe), nameof(Universe.ApplyOrbitSolvers))]
internal static class ApplyOrbitSolversPatch
{
    [HarmonyPostfix]
    private static void Postfix()
    {
        try
        {
            long start = System.Diagnostics.Stopwatch.GetTimestamp();
            using (FrameProfiler.PlanetConics.Measure())
            {
                CelestialDriver.RefreshOrbits();
            }
            Diagnostics.RecordFrame(GetJobSimStepPatch.LastProlongTicks,
                                    System.Diagnostics.Stopwatch.GetTimestamp() - start);
            using (FrameProfiler.StatusAndSettings.Measure())
            {
                Multiplayer.Tick();
                Diagnostics.Tick();
            }
            using (FrameProfiler.Snapshot.Measure())
            {
                Predictor.Tick();
            }
            using (FrameProfiler.StatusAndSettings.Measure())
            {
                SettingsStore.Tick();
            }
        }
        catch (Exception e)
        {
            // Nothing may escape into KSA's frame.
            Log.Warn($"[frame] {e.Message}");
        }
    }
}

// Loading a save puts the bodies on their conics; put them on Principia's.
[HarmonyPatch(typeof(Universe), nameof(Universe.DeserializeSave))]
internal static class DeserializeSavePatch
{
    [HarmonyPostfix]
    private static void Postfix()
    {
        try
        {
            UniverseTime t = Universe.GetElapsedTime();
            Log.Info($"Save loaded at t = {t.Seconds():F0} s ({GameTime.ToDateTime(t):yyyy-MM-dd HH:mm}); " +
                     "moving the bodies to Principia's states");
            CelestialDriver.JumpTo(t);
        }
        catch (Exception e)
        {
            PrincipiaSystem.Current?.Fail($"after loading a save: {e.Message}");
        }
    }
}
