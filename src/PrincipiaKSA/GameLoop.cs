using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// Where the mod runs in KSA's frame. There is no loop of the mod's own: KSA
// calls these hooks at fixed points of its frame (Program.OnFrame), in this
// order: a star system loads (LoadSystemPatch); each frame, the step KSA's
// jobs will simulate is chosen (GetJobSimStepPatch), the jobs run on worker
// threads (CelestialDriver, VesselDriver), and between frames their results
// are applied (ApplyOrbitSolversPatch); a save loads (DeserializeSavePatch).

// KSA picks the step its jobs will simulate: integrate that far first, and
// keep a few real seconds ahead at the current warp.
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

// After KSA loads a star system, builds the Principia ephemeris from it. With
// startup checks on, first lists the bodies and checks KSA's frames.
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
            if (Settings.StartupChecks)
            {
                ListBodies(id, system);
                FrameProbe.Run(system);
            }

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
