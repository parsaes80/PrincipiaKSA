using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

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
        var celestials = system.All.OfType<Celestial>().GetEnumerator();
        while (celestials.MoveNext())
        {
            Celestial c = celestials.Current;
            string parent = c.HasOrbit() ? (c.Orbit.Parent as IObjectId)?.Id ?? "?" : "(root)";
            Log.Info($"  {c.Id,-16} class={c.Class,-14} parent={parent,-10} " +
                     $"mass={c.Mass:E4} kg  radius={c.MeanRadius / 1000:F1} km");
            count++;
        }
        Log.Info($"  {count} celestials total");
    }
}
