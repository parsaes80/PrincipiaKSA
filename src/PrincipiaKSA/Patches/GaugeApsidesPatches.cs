using System.Reflection.Emit;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

// KSA's Ap and Pe gauges and its warp-to-Ap/Pe buttons read the vessel's
// osculating conic, whose apsides n-body gravity keeps moving (the Sun and
// Moon shift a high orbit's perigee by thousands of km). While Principia
// drives, the controlled vessel's show the next apsides of its prediction
// about its parent; none ahead (or no prediction): the gauge reads 0, which
// KSA shows blank, and the button has nothing to warp to.
internal static class NBodyApsides
{
    public static bool Applies(Vehicle vehicle) =>
        PrincipiaSystem.Current is { Driving: true } && vehicle == Program.ControlledVehicle;

    public static (double Time, double Altitude)? Next(Vehicle vehicle, bool apoapsis)
    {
        if (Predictor.Latest is not { } prediction || prediction.Vehicle != vehicle ||
            !ReferenceEquals(prediction.ParentApsides.Parent, vehicle.Orbit.Parent))
        {
            return null;
        }
        return prediction.ParentApsides.Next(apoapsis, GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime()));
    }

    // Replace Orbit.PeriapsisAltitude and ApoapsisAltitude in Program.UpdateGauges.
    public static double PeriapsisAltitude(Orbit orbit) => Altitude(orbit, apoapsis: false);

    public static double ApoapsisAltitude(Orbit orbit) => Altitude(orbit, apoapsis: true);

    private static double Altitude(Orbit orbit, bool apoapsis)
    {
        try
        {
            if (Program.ControlledVehicle is { } vehicle && ReferenceEquals(vehicle.Orbit, orbit) &&
                Applies(vehicle))
            {
                return Next(vehicle, apoapsis)?.Altitude ?? 0;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[gauges] n-body Ap/Pe", e);
        }
        return apoapsis ? orbit.ApoapsisAltitude : orbit.PeriapsisAltitude;
    }
}

[HarmonyPatch(typeof(Program), "UpdateGauges")]
internal static class GaugeApsidesPatch
{
    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var periapsis = AccessTools.PropertyGetter(typeof(Orbit), nameof(Orbit.PeriapsisAltitude));
        var apoapsis = AccessTools.PropertyGetter(typeof(Orbit), nameof(Orbit.ApoapsisAltitude));
        int replaced = 0;
        foreach (CodeInstruction instruction in instructions)
        {
            if (instruction.Calls(periapsis) || instruction.Calls(apoapsis))
            {
                instruction.operand = AccessTools.Method(typeof(NBodyApsides),
                    instruction.Calls(periapsis) ? nameof(NBodyApsides.PeriapsisAltitude)
                                                 : nameof(NBodyApsides.ApoapsisAltitude));
                instruction.opcode = OpCodes.Call;
                replaced++;
            }
            yield return instruction;
        }
        if (replaced != 2)
        {
            Log.Warn($"[gauges] Program.UpdateGauges reads Ap/Pe {replaced} times, not 2; " +
                     "the gauges may show KSA's conic");
        }
    }
}

// The warp-to-Ap/Pe buttons' times, set each frame from the conic.
[HarmonyPatch(typeof(Vehicle), nameof(Vehicle.UpdatePerFrameData))]
internal static class WarpToApsisPatch
{
    private static readonly AccessTools.FieldRef<Vehicle, UniverseTime> NextPeriapsisTime =
        AccessTools.FieldRefAccess<Vehicle, UniverseTime>("_nextPeriapsisTime");
    private static readonly AccessTools.FieldRef<Vehicle, UniverseTime> NextApoapsisTime =
        AccessTools.FieldRefAccess<Vehicle, UniverseTime>("_nextApoapsisTime");

    [HarmonyPostfix]
    private static void Postfix(Vehicle __instance)
    {
        using var profiled = FrameProfiler.WarpApsides.Measure();
        try
        {
            if (!NBodyApsides.Applies(__instance))
            {
                return;
            }
            UniverseTime Time((double Time, double Altitude)? apsis) =>
                apsis is { } a ? GameTime.FromSecondsSinceJ2000(a.Time) : UniverseTime.EndOfTime;
            NextPeriapsisTime(__instance) = Time(NBodyApsides.Next(__instance, apoapsis: false));
            NextApoapsisTime(__instance) = Time(NBodyApsides.Next(__instance, apoapsis: true));
        }
        catch (Exception e)
        {
            Log.WarnOnce("[gauges] n-body warp-to-Ap/Pe times", e);
        }
    }
}
