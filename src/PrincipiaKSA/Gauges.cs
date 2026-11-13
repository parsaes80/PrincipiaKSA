using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

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
        if (Predictor.PredictionFor(vehicle) is not { } prediction ||
            !ReferenceEquals(prediction.ParentApsides.Parent, vehicle.Orbit.Parent))
        {
            return null;
        }
        return prediction.ParentApsides.Next(apoapsis, GameTime.Now);
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
        return Patching.ReplaceCalls(instructions, "[gauges] Program.UpdateGauges doesn't read Ap and Pe once each; " +
                                     "the gauges may show KSA's conic", expected: 2,
            (AccessTools.PropertyGetter(typeof(Orbit), nameof(Orbit.PeriapsisAltitude)),
             AccessTools.Method(typeof(NBodyApsides), nameof(NBodyApsides.PeriapsisAltitude))),
            (AccessTools.PropertyGetter(typeof(Orbit), nameof(Orbit.ApoapsisAltitude)),
             AccessTools.Method(typeof(NBodyApsides), nameof(NBodyApsides.ApoapsisAltitude))));
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
