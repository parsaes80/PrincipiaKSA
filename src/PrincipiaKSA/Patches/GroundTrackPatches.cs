using System.Reflection.Emit;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

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
        Prediction? line = Predictor.LatestPlan is { } plan && plan.Vehicle == vehicle && plan.StateTimes.Length > 1
            ? plan
            : Predictor.Latest is { } coast && coast.Vehicle == vehicle && coast.StateTimes.Length > 1 ? coast : null;
        if (line == null || !system.Enter())
        {
            return null;
        }
        try
        {
            double t = Math.Clamp(GameTime.ToSecondsSinceJ2000(time), line.StateTimes[0], line.StateTimes[^1]);
            QP vessel = Interpolate(line.StateTimes, line.States, t);
            QP centre = system.Ephemeris.State(index, t);
            return (vessel.Q.ToDouble3() - centre.Q.ToDouble3(), vessel.P.ToDouble3() - centre.P.ToDouble3());
        }
        finally
        {
            system.Exit();
        }
    }

    // Cubic Hermite between the integrated samples, and its derivative.
    private static QP Interpolate(double[] times, QP[] states, double t)
    {
        int i = Array.BinarySearch(times, t);
        if (i >= 0)
        {
            return states[i];
        }
        return Interpolate(times, states, Math.Clamp(~i, 1, times.Length - 1), t);
    }

    // Between samples i - 1 and i.
    internal static QP Interpolate(IReadOnlyList<double> times, IReadOnlyList<QP> states, int i, double t)
    {
        double h = times[i] - times[i - 1], u = (t - times[i - 1]) / h, u2 = u * u, u3 = u2 * u;
        double3 q0 = states[i - 1].Q.ToDouble3(), q1 = states[i].Q.ToDouble3();
        double3 v0 = states[i - 1].P.ToDouble3(), v1 = states[i].P.ToDouble3();
        double3 q = q0 * (2 * u3 - 3 * u2 + 1) + v0 * ((u3 - 2 * u2 + u) * h) + q1 * (-2 * u3 + 3 * u2) +
                    v1 * ((u3 - u2) * h);
        double3 v = q0 * ((6 * u2 - 6 * u) / h) + v0 * (3 * u2 - 4 * u + 1) + q1 * ((6 * u - 6 * u2) / h) +
                    v1 * (3 * u2 - 2 * u);
        return new QP(q, v);
    }
}

[HarmonyPatch(typeof(GroundTrackWindow), nameof(GroundTrackWindow.DrawGroundTrack))]
internal static class GroundTrackPatch
{
    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions)
    {
        var conic = AccessTools.Method(typeof(PatchedConic), nameof(PatchedConic.GetStateVectorsAt),
                                       [typeof(UniverseTime)]);
        int replaced = 0;
        foreach (CodeInstruction instruction in instructions)
        {
            if (instruction.Calls(conic))
            {
                instruction.opcode = OpCodes.Call;
                instruction.operand = AccessTools.Method(typeof(GroundTrack), nameof(GroundTrack.StateVectorsAt));
                replaced++;
            }
            yield return instruction;
        }
        if (replaced == 0)
        {
            Log.Warn("[ground track] GroundTrackWindow.DrawGroundTrack no longer reads the conics; " +
                     "it shows KSA's track");
        }
    }
}
