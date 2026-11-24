using System.Reflection;
using System.Reflection.Emit;
using HarmonyLib;
using KSA;
using StarMap.API;
using StarMap.API.Services;

namespace PrincipiaKSA;

// StarMap's entry point: the [StarMapMod] class named after the DLL.
[StarMapMod]
public class PrincipiaKSA
{
    public const string HarmonyId = "principia.ksa";

    private Harmony? _harmony;
    private readonly IModMenuService _modMenu;
    private IDisposable? _modMenuItem;

    // StarMap passes its services to the constructor.
    public PrincipiaKSA(IModMenuService modMenu) => _modMenu = modMenu;

    [StarMapImmediateLoad]
    public void Init(Mod definingMod)
    {
        // First, as it decides whether the log also goes to the terminal.
        SettingsStore.Load();
        Log.Info("Hello from PrincipiaKSA!");
        Log.Info($"KSA.dll version: {typeof(Mod).Assembly.GetName().Version}");
        Log.Info($"Running on .NET {Environment.Version}, {Environment.OSVersion}");

        try
        {
            Native.RegisterResolver();
            Native.EnableLogging();
            Log.Info(Native.Version());
            string model = Path.Combine(Path.GetDirectoryName(typeof(PrincipiaKSA).Assembly.Location) ?? ".",
                                        "sol_gravity_model.proto.txt");
            Native.Check(Native.LoadGravityModel(model));
        }
        catch (DllNotFoundException e)
        {
            // Unsupported CPU or missing library: leave KSA unmodded.
            Log.Error($"{e.Message}; the mod is disabled");
            return;
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

        // StarMap's Mods menu in KSA's top bar shows or hides the window.
        try
        {
            _modMenuItem = _modMenu.AddModMenuItem("PrincipiaKSA", () => Settings.WindowEnabled = !Settings.WindowEnabled);
        }
        catch (Exception e)
        {
            Log.Warn($"PrincipiaKSA not added to StarMap's Mods menu: {e.Message}");
        }
    }

    [StarMapUnload]
    public void Unload()
    {
        if (_harmony == null)
        {
            return;  // Disabled in Init: nothing was loaded.
        }
        _modMenuItem?.Dispose();
        Predictor.Stop();
        SettingsStore.Save();
        _harmony.UnpatchAll(HarmonyId);
        Log.Info($"Driver totals: {CelestialDriver.StatesServed} states served, " +
                 $"{CelestialDriver.OrbitsRefreshed} orbits rebuilt");
        PrincipiaSystem.Clear();
        Native.DisableLogging();
        Log.Info("Unloaded");
    }
}

internal static class Patching
{
    // Class by class, so a hook broken by a game update only disables itself (PatchAll stops at the first failure).
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

    // Each call to a `swaps` original becomes a call to its static replacement (instance first); logs `missing` when
    // there are none, or not `expected` of them, as KSA's code changed under the patch.
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
