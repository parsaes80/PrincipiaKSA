using System.Reflection;
using System.Reflection.Emit;
using HarmonyLib;
using KSA;
using StarMap.API;

namespace PrincipiaKSA;

// StarMap's entry point: the [StarMapMod] class named after the DLL.
[StarMapMod]
public class PrincipiaKSA
{
    public const string HarmonyId = "principia.ksa";

    private Harmony? _harmony;

    [StarMapImmediateLoad]
    public void Init(Mod definingMod)
    {
        // First: whether the log also goes to the terminal (log2terminal).
        SettingsStore.Load();
        Log.Info("Hello from PrincipiaKSA!");
        Log.Info($"KSA.dll version: {typeof(Mod).Assembly.GetName().Version}");
        Log.Info($"Running on .NET {Environment.Version}, {Environment.OSVersion}");
        // Only the AVX/FMA build ships; without them, leave KSA unmodded.
        if (!Native.CpuHasAvxFma)
        {
            Log.Error("PrincipiaKSA needs a CPU with AVX and FMA (Intel Haswell / AMD Zen or later); " +
                      "this one lacks them, so the mod is disabled");
            return;
        }

        // The native library, with Principia's log routed into ours.
        try
        {
            Native.RegisterResolver();
            Native.EnableLogging();
            Log.Info(Native.Version());
            string model = Path.Combine(Path.GetDirectoryName(typeof(PrincipiaKSA).Assembly.Location) ?? ".",
                                        "sol_gravity_model.proto.txt");
            Native.Check(Native.LoadGravityModel(model));
        }
        catch (Exception e)
        {
            Log.Error($"Loading Principia failed: {e}");
        }

        _harmony = new Harmony(HarmonyId);
        Patching.ApplyAll(_harmony);
        Predictor.Start();
        Log.Info($"Harmony {typeof(Harmony).Assembly.GetName().Version} ready");
        foreach (var method in _harmony.GetPatchedMethods())
        {
            Log.Info($"  patched {method.DeclaringType?.Name}.{method.Name}");
        }
    }

    [StarMapUnload]
    public void Unload()
    {
        if (_harmony == null)
        {
            return;  // Disabled in Init: nothing was loaded.
        }
        Predictor.Stop();
        SettingsStore.Save();
        _harmony?.UnpatchAll(HarmonyId);
        Log.Info($"Driver totals: {CelestialDriver.StatesServed} states served, " +
                 $"{CelestialDriver.OrbitsRefreshed} orbits rebuilt");
        PrincipiaSystem.Clear();
        Native.DisableLogging();
        Log.Info("Unloaded");
    }
}

// How the mod hooks into KSA (Harmony), and the one helper its transpilers
// share.
internal static class Patching
{
    // Every patch class in the mod, class by class, so a hook broken by a
    // game update only disables itself (PatchAll stops at the first failure).
    public static void ApplyAll(Harmony harmony)
    {
        foreach (Type type in AccessTools.GetTypesFromAssembly(typeof(PrincipiaKSA).Assembly))
        {
            try
            {
                harmony.CreateClassProcessor(type).Patch();
            }
            catch (Exception e)
            {
                Log.Error($"Patch {type.Name} failed and is disabled: {e.InnerException?.Message ?? e.Message}");
            }
        }
    }

    // A transpiler's work: every call to one of `swaps`' originals becomes a
    // call to its replacement (a static method with the same arguments, the
    // instance first). Logs `missing` where there are no such calls, or not
    // `expected` of them: KSA's code changed under the patch.
    public static IEnumerable<CodeInstruction> ReplaceCalls(IEnumerable<CodeInstruction> instructions,
                                                            string missing,
                                                            params (MethodInfo From, MethodInfo To)[] swaps) =>
        ReplaceCalls(instructions, missing, null, swaps);

    public static IEnumerable<CodeInstruction> ReplaceCalls(IEnumerable<CodeInstruction> instructions,
                                                            string missing, int? expected,
                                                            params (MethodInfo From, MethodInfo To)[] swaps)
    {
        int replaced = 0;
        foreach (CodeInstruction instruction in instructions)
        {
            foreach (var (from, to) in swaps)
            {
                if (instruction.Calls(from))
                {
                    instruction.opcode = OpCodes.Call;
                    instruction.operand = to;
                    replaced++;
                    break;
                }
            }
            yield return instruction;
        }
        if (expected is int n ? replaced != n : replaced == 0)
        {
            Log.Warn($"{missing} ({replaced} found)");
        }
    }
}
