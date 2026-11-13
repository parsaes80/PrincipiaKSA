using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// KSA's ground track window (F3) samples the vessel's patched conics. While
// Principia drives, it samples the vessel's n-body path instead: the flight
// plan when there is one (it starts with the coast to the first burn), else
// the prediction. Past their end it holds the last point, so the track ends
// there rather than going on along KSA's conic. KSA keeps its drawing, burn
// and impact markers and atmosphere colours.
internal static class GroundTrack
{
    // Replaces PatchedConic.GetStateVectorsAt in GroundTrackWindow.DrawGroundTrack.
    public static StateVectors StateVectorsAt(PatchedConic patch, UniverseTime time)
    {
        StateVectors conic = patch.GetStateVectorsAt(time);
        try
        {
            if (Relative(patch.PrimaryBody, time) is { } relative)
            {
                doubleQuat cce2Cci = patch.PrimaryBody.GetCci2Cce().Inverse();
                return new StateVectors(time, relative.Q.Transform(cce2Cci), relative.V.Transform(cce2Cci),
                                        conic.TrueAnomaly);
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[ground track] n-body path", e);
        }
        return conic;
    }

    // The controlled vessel's n-body state relative to `parent` (Cce axes).
    private static (double3 Q, double3 V)? Relative(IParentBody parent, UniverseTime time)
    {
        if (Program.ControlledVehicle is not { } vehicle || PrincipiaSystem.Current is not { Driving: true } system ||
            parent is not Astronomical body || system.IndexOf(body) is not int index)
        {
            return null;
        }
        Prediction? line = Predictor.PlanFor(vehicle) is { StateTimes.Length: > 1 } plan ? plan
                         : Predictor.PredictionFor(vehicle) is { StateTimes.Length: > 1 } coast ? coast
                         : null;
        if (line == null || !system.Enter())
        {
            return null;
        }
        try
        {
            double t = Math.Clamp(GameTime.ToSecondsSinceJ2000(time), line.StateTimes[0], line.StateTimes[^1]);
            QP vessel = Hermite.At(line.StateTimes, line.States, t);
            QP centre = system.Ephemeris.State(index, t);
            return (vessel.Q.ToDouble3() - centre.Q.ToDouble3(), vessel.P.ToDouble3() - centre.P.ToDouble3());
        }
        finally
        {
            system.Exit();
        }
    }
}

[HarmonyPatch(typeof(GroundTrackWindow), nameof(GroundTrackWindow.DrawGroundTrack))]
internal static class GroundTrackPatch
{
    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        return Patching.ReplaceCalls(instructions, "[ground track] GroundTrackWindow.DrawGroundTrack no longer reads " +
                                     "the conics; it shows KSA's track",
            (AccessTools.Method(typeof(PatchedConic), nameof(PatchedConic.GetStateVectorsAt), [typeof(UniverseTime)]),
             AccessTools.Method(typeof(GroundTrack), nameof(GroundTrack.StateVectorsAt))));
    }
}
