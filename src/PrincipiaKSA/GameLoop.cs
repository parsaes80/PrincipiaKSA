using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// The mod has no loop of its own; KSA's frame calls these hooks: a system loads (LoadSystemPatch); each frame the jobs'
// step is chosen (GetJobSimStepPatch), jobs run on workers (CelestialDriver, VesselDriver), and between frames
// results are applied (ApplyOrbitSolversPatch); a save loads (DeserializeSavePatch).

// Integrate as far as the step KSA's jobs will simulate, and ask for more ahead.
[HarmonyPatch(typeof(Universe), nameof(Universe.GetJobSimStep))]
internal static class GetJobSimStepPatch
{
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
            using (FrameProfiler.ProlongToNextStep.Measure())
            {
                system.Prolonger.EnsureProlonged(next);
            }
            double simSecondsPerRealSecond = __result.DeltaTime / Math.Max(dtPlayer, 1e-3);
            // A few real seconds ahead at this warp, and at least the lines' horizon.
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
            using (FrameProfiler.PlanetConics.Measure())
            {
                CelestialDriver.RefreshOrbits();
            }
            using (FrameProfiler.StatusAndSettings.Measure())
            {
                Multiplayer.Tick();
                Diagnostics.Tick();
                SettingsStore.Tick();
            }
            using (FrameProfiler.Snapshot.Measure())
            {
                Predictor.Tick();
            }
        }
        catch (Exception e)
        {
            // Nothing may escape into KSA's frame.
            Log.WarnOnce("[frame] the mod's work between frames", e);
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

[HarmonyPatch(typeof(Universe), nameof(Universe.LoadSystem))]
internal static class LoadSystemPatch
{
    [HarmonyPostfix]
    private static void Postfix(string id)
    {
        try
        {
            CelestialSystem? system = Universe.CurrentSystem;
            if (system == null)
            {
                Log.Warn($"LoadSystem('{id}') finished but CurrentSystem is null");
                return;
            }
            ListBodies(id, system);
            FrameProbe.Run(system);

            var sw = System.Diagnostics.Stopwatch.StartNew();
            PrincipiaSystem principia = PrincipiaSystem.Build(system);
            CelestialDriver.JumpTo(Universe.GetElapsedTime());
            Log.Info($"System '{id}': Principia drives {principia.Links.Count()} bodies " +
                     $"({principia.Bodies.Count} massive, {principia.Massless.Count} massless left on KSA's " +
                     $"conics), built in {sw.Elapsed.TotalMilliseconds:F0} ms");
        }
        catch (Exception e)
        {
            Log.Error($"LoadSystemPatch failed: {e}");
        }
    }

    private static void ListBodies(string id, CelestialSystem system)
    {
        Log.Info($"LoadSystem('{id}') finished. Celestials:");
        int count = 0;
        foreach (Celestial c in system.BodiesOf<Celestial>())
        {
            string parent = c.HasOrbit() ? (c.Orbit.Parent as IObjectId)?.Id ?? "?" : "(root)";
            Log.Info($"  {c.Id,-16} class={c.Class,-14} parent={parent,-10} " +
                     $"mass={c.Mass:E4} kg  radius={c.MeanRadius / 1000:F1} km");
            count++;
        }
        Log.Info($"  {count} celestials total");
    }
}
