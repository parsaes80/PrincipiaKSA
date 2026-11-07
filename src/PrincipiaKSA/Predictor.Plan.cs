using System.Diagnostics;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

internal static partial class Predictor
{
    // Coasts to each burn's ignition, burns until its Δv is reached (staging
    // through the engine phases), and coasts on for the plan's length after
    // the last burn. Also computes the nodes' approach tracks.
    private static Prediction? ComputePlan(PrincipiaSystem system, Request r, Buffers b, out NodeTracks? nodes)
    {
        var sw = Stopwatch.StartNew();
        Ephemeris ephemeris = system.Ephemeris;
        // Apsides only once the plan departs from the prediction.
        var trajectory = new Trajectory(r.State, r.Time) { CollectApsides = false };
        // Segment j holds the nodes before burn j, the last those after all.
        var segments = new List<NodeBurn>?[r.Burns.Length + 1];
        foreach (NodeBurn node in r.Nodes)
        {
            int j = 0;
            while (j < r.Burns.Length && r.Burns[j].Impulse < node.Time)
            {
                j++;
            }
            (segments[j] ??= new List<NodeBurn>()).Add(node);
        }
        var tracks = new Dictionary<Burn, NodeTrack>(ReferenceEqualityComparer.Instance);
        // Segment, its first sample, and until when the plan shares its path.
        var pending = new List<(int Segment, int StartIndex, double Shared)>();
        var windows = new Dictionary<Burn, double>(ReferenceEqualityComparer.Instance);
        foreach (NodeBurn node in r.Nodes)
        {
            windows[node.Burn] = node.Window;
        }
        int segmentStart = 0;
        double span = (r.Burns.Length > 0 ? r.Burns[^1].Impulse : r.Time) - r.Time + r.PlanHorizon;
        double interval = SampleInterval(Math.Max(span, 1));
        string? note = r.Phases.Length == 0 ? "no active engine: burns shown as impulses" : null;
        int phase = 0;
        double mass = r.Phases.Length > 0 ? r.Phases[0].StartMass : r.Mass;
        double phaseUsed = 0;  // Seconds burned in the current phase.
        int firstIgnition = -1;
        int burnNumber = 0;
        foreach (PlannedBurn burn in r.Burns)
        {
            burnNumber++;
            double duration = Duration(r.Phases, phase, mass, phaseUsed, burn.DeltaV);
            double ignition = burn.Ignition ?? burn.Impulse - duration / 2;
            segmentStart = trajectory.Times.Count - 1;
            bool reached = trajectory.Extend(ephemeris, Math.Max(ignition, r.Time), Settings.MaxSteps, interval,
                                             r.ApsisBodies, b);
            pending.Add((burnNumber - 1, segmentStart, trajectory.LastTime));
            if (!reached)
            {
                break;
            }
            trajectory.CollectApsides = true;
            double3 direction = BurnDirection(ephemeris, burn, trajectory);
            int start = trajectory.Times.Count - 1;
            firstIgnition = firstIgnition < 0 ? start : firstIgnition;
            trajectory.Burns.Add((trajectory.LastTime, trajectory.Last,
                                  $"Burn {burnNumber}: Δv {burn.DeltaV:F1} m/s" +
                                  (r.Phases.Length > 0 ? $", {duration:F0} s" : "")));
            if (r.Phases.Length == 0)
            {
                // No engine to model: apply the Δv at once.
                trajectory.Last = new QP(trajectory.Last.Q.ToDouble3(),
                                         trajectory.Last.P.ToDouble3() + burn.DeltaV * direction);
                trajectory.Times.Add(trajectory.LastTime);
                trajectory.States.Add(trajectory.Last);
                trajectory.BurnArcs.Add((start, trajectory.Times.Count - 1));
                continue;
            }
            // Burn phase by phase until the Δv is reached.
            double remaining = burn.DeltaV;
            bool ok = true;
            while (remaining > 1e-6 && phase < r.Phases.Length && ok)
            {
                EnginePhase e = r.Phases[phase];
                double left = e.MaxSeconds - phaseUsed;
                double seconds = BurnSeconds(e.Thrust, e.MassFlow, mass, remaining);
                bool phaseEnds = seconds >= left;
                if (phaseEnds)
                {
                    seconds = left;
                }
                if (e.MassFlow > 0)
                {
                    // Never all the mass: the native side checks it.
                    seconds = Math.Min(seconds, 0.999999 * mass / e.MassFlow);
                }
                if (!double.IsFinite(seconds) || seconds <= 0)
                {
                    break;
                }
                ok = trajectory.Extend(ephemeris, trajectory.LastTime + seconds, Settings.MaxSteps, interval,
                                       r.ApsisBodies, b, direction, e.Thrust, mass, e.MassFlow);
                remaining -= DeltaV(e.Thrust, e.MassFlow, mass, seconds);
                mass -= e.MassFlow * seconds;
                phaseUsed += seconds;
                if (phaseEnds)
                {
                    phase++;
                    phaseUsed = 0;
                    if (phase < r.Phases.Length)
                    {
                        mass = r.Phases[phase].StartMass;  // Staging.
                    }
                }
            }
            trajectory.BurnArcs.Add((start, trajectory.Times.Count - 1));
            if (!ok)
            {
                break;
            }
        }
        if (trajectory.End == PredictionEnd.ReachedEnd)
        {
            segmentStart = trajectory.Times.Count - 1;
            if (r.Burns.Length > 0)
            {
                trajectory.Extend(ephemeris, trajectory.LastTime + r.PlanHorizon, Settings.MaxSteps, interval,
                                  r.ApsisBodies, b);
            }
            if (burnNumber == r.Burns.Length)
            {
                pending.Add((r.Burns.Length, segmentStart, trajectory.LastTime));
            }
        }
        foreach (var (segment, startIndex, shared) in pending)
        {
            AddNodeTracks(system, r, segments[segment], segment, trajectory, startIndex, shared, b, tracks);
        }
        nodes = tracks.Count > 0 ? new NodeTracks(r.Vehicle, r.Frame, tracks, windows) : null;
        return r.Burns.Length > 0
            ? Finish(system, r, trajectory, isFlightPlan: true, Math.Max(firstIgnition, 0), note, sw, b)
            : null;
    }

    // The approach track shared by a segment's nodes: integrated without burns
    // from the plan's last coast sample before their windows, through them.
    // Up to `shared` (the next burn's ignition) it is the plan itself; past
    // it, the path without that burn, which nothing else draws.
    private static void AddNodeTracks(PrincipiaSystem system, Request r, List<NodeBurn>? nodes, int segment,
                                      Trajectory trajectory, int segmentStart, double shared, Buffers b,
                                      Dictionary<Burn, NodeTrack> tracks)
    {
        if (nodes == null)
        {
            return;
        }
        Ephemeris ephemeris = system.Ephemeris;
        double start, t0;
        QP s0;
        if (trajectory.Times.Count == 0)
        {
            // Never extended: no burn, so the plan's start.
            start = t0 = trajectory.LastTime;
            s0 = trajectory.Last;
        }
        else
        {
            int first = Math.Clamp(segmentStart, 0, trajectory.Times.Count - 1);
            start = trajectory.Times[first];
            double lo = Math.Max(start, Math.Min(nodes.Min(n => n.Time - n.Window), shared));
            int k = first;
            while (k + 1 < trajectory.Times.Count && trajectory.Times[k + 1] <= lo)
            {
                k++;
            }
            t0 = trajectory.Times[k];
            s0 = trajectory.States[k];
        }
        double hi = nodes.Max(n => n.Time + n.Window);
        if (!(hi > t0))
        {
            return;
        }
        (int count, _, _) = ephemeris.Predict(
            s0, t0, hi, Settings.MaxSteps, double3.Zero, 0, 0, 0, SampleInterval(hi - t0),
            b.Times, b.States, ReadOnlySpan<int>.Empty, b.Apsides);
        if (count < 2)
        {
            return;
        }
        double[] times = b.Times.AsSpan(0, count).ToArray();
        QP[] states = b.States.AsSpan(0, count).ToArray();
        (Line line, _, _) = Draw(system, r.Frame, times, states, count);
        var track = new NodeTrack(start, line.Times, line.Coordinates, line.Stretches, segment, shared, times,
                                  states);
        foreach (NodeBurn node in nodes)
        {
            tracks[node.Burn] = track;
        }
    }

    // A burn's direction: its Δv in the prograde/normal/radial frame of
    // Principia's state at the burn time, about the burn's body (where
    // Principia's path is then, BurnAnchor). The plan has reached the
    // ignition; the burn time is a short coast on. A burn KSA already targets
    // keeps KSA's direction.
    private static double3 BurnDirection(Ephemeris ephemeris, PlannedBurn burn, Trajectory trajectory)
    {
        if (burn.DeltaVVlf is not { } vlf || burn.ParentIndex < 0)
        {
            return burn.Direction;
        }
        try
        {
            QP state = burn.Impulse > trajectory.LastTime + 1e-3
                ? ephemeris.FlowMassless(trajectory.Last, trajectory.LastTime, burn.Impulse)
                : trajectory.Last;
            QP parent = ephemeris.State(burn.ParentIndex, Math.Max(burn.Impulse, trajectory.LastTime));
            double3 q = state.Q.ToDouble3() - parent.Q.ToDouble3();
            double3 v = state.P.ToDouble3() - parent.P.ToDouble3();
            if (StateVectors.GetVlf2GivenFrame(q.Normalized(), v.Normalized()) is { } vlf2Ecl)
            {
                double3 direction = vlf.Transform(vlf2Ecl);
                if (direction.Length() > 0)
                {
                    return direction.Normalized();
                }
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[predict] burn direction", e);
        }
        return burn.Direction;
    }

    // Rocket equation, phase by phase: the time to gain `deltaV`.
    private static double Duration(EnginePhase[] phases, int phase, double mass, double phaseUsed, double deltaV)
    {
        double total = 0;
        while (deltaV > 1e-6 && phase < phases.Length)
        {
            EnginePhase e = phases[phase];
            double left = e.MaxSeconds - phaseUsed;
            double seconds = BurnSeconds(e.Thrust, e.MassFlow, mass, deltaV);
            if (seconds < left)
            {
                return total + seconds;
            }
            total += left;
            deltaV -= DeltaV(e.Thrust, e.MassFlow, mass, left);
            phase++;
            phaseUsed = 0;
            mass = phase < phases.Length ? phases[phase].StartMass : mass;
        }
        return total;
    }

    private static double BurnSeconds(double thrust, double massFlow, double mass, double deltaV)
    {
        if (massFlow <= 0)
        {
            return deltaV * mass / thrust;
        }
        double exhaust = thrust / massFlow;
        return mass / massFlow * (1 - Math.Exp(-deltaV / exhaust));
    }

    private static double DeltaV(double thrust, double massFlow, double mass, double seconds) =>
        massFlow <= 0 ? thrust * seconds / mass
                      : thrust / massFlow * Math.Log(mass / Math.Max(mass - massFlow * seconds, 1e-9));
}
