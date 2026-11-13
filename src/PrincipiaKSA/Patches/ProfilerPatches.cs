using System.Reflection;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

// The start of each of KSA's frames: FrameProfiler takes the last frame's
// times and logs it if it stuttered.
[HarmonyPatch(typeof(Program), "OnFrame")]
internal static class FrameStartPatch
{
    [HarmonyPrefix]
    private static void Prefix()
    {
        try
        {
            FrameProfiler.FrameStart();
        }
        catch (Exception e)
        {
            Log.WarnOnce("[stutter] frame start", e);
        }
    }
}

// Times KSA's own per-frame work, so a stutter report shows KSA's part next to
// the mod's. Each method is wrapped whole: its time includes the mod's hooks
// on it and anything it calls.
[HarmonyPatch]
internal static class KsaTimingPatches
{
    private static readonly (Type Type, string Method)[] Targets =
    [
        // The frame, in KSA's order (Program.OnFrame).
        (typeof(Program), "PrepareFrame"),        // Waits for the last frame's jobs, applies their results.
        (typeof(Universe), nameof(Universe.ApplyOrbitSolvers)),
        (typeof(Universe), nameof(Universe.ApplyVehicleSolvers)),
        (typeof(Universe), nameof(Universe.GetJobSimStep)),
        (typeof(Universe), nameof(Universe.ExecuteNextVehicleSolvers)),
        (typeof(Universe), nameof(Universe.ExecuteNextOrbitSolvers)),
        (typeof(Program), "OnFrameCelestials"),
        (typeof(Program), "UpdateGauges"),
        (typeof(Program), "OnPreRender"),
        (typeof(CelestialSystem), nameof(CelestialSystem.OnPreRender)),
        (typeof(Program), "Render"),
        (typeof(Universe), nameof(Universe.OnDrawUi)),
        (typeof(Program), "PostRender"),
        // Orbit work that can run long: flight plans and their impact search.
        (typeof(VehicleUpdateState), nameof(VehicleUpdateState.RecalculateFlightPlan)),
        (typeof(BurnPlan), nameof(BurnPlan.CalculateNewFlightPlansFromFlightComputerOnly)),
        (typeof(FlightPlan), "CalculateConicPatch"),
        (typeof(PatchedConic), nameof(PatchedConic.SearchForParentCollision)),
        (typeof(Vehicle), nameof(Vehicle.UpdatePerFrameData)),
        (typeof(GroundTrackWindow), nameof(GroundTrackWindow.DrawGroundTrack)),
    ];

    private static readonly Dictionary<MethodBase, FrameProfiler.Section> Sections = new();

    private static IEnumerable<MethodBase> TargetMethods()
    {
        foreach (var (type, name) in Targets)
        {
            var methods = AccessTools.GetDeclaredMethods(type)
                                     .Where(m => m.Name == name && !m.IsAbstract && !m.ContainsGenericParameters)
                                     .ToList();
            if (methods.Count == 0)
            {
                Log.Warn($"[stutter] KSA has no {type.Name}.{name} to time");
                continue;
            }
            FrameProfiler.Section section = FrameProfiler.Define($"KSA {type.Name}.{name}");
            foreach (MethodInfo method in methods)
            {
                Sections[method] = section;
                yield return method;
            }
        }
    }

    // First in, and the finalizer last out, so the mod's hooks are inside.
    [HarmonyPrefix]
    [HarmonyPriority(Priority.First)]
    private static void Prefix(MethodBase __originalMethod, out FrameProfiler.Measurement __state)
    {
        __state = Sections.TryGetValue(__originalMethod, out FrameProfiler.Section? section)
            ? section.Measure()
            : default;
    }

    // Runs also when the method throws.
    [HarmonyFinalizer]
    private static void Finalizer(FrameProfiler.Measurement __state) => __state.Dispose();
}
