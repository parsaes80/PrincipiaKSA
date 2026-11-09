using System.Diagnostics;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

internal static partial class Predictor
{
    // Coasts to each burn's ignition (each coast up to the plan's steps),
    // burns until its Δv is reached (staging through the engine phases), and
    // after the last burn coasts on until its steps run out, as Principia's
    // flight plan. A coast that runs out of steps before the next burn ends
    // the plan there, as Principia's anomalous manœuvres. Also computes the
    // nodes' approach tracks.
    private static Prediction? ComputePlan(PrincipiaSystem system, Request r, out NodeTracks? nodes)
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
        var notes = new List<string>();
        if (r.Phases.Length == 0)
        {
            notes.Add("no active engine: burns shown as impulses");
        }
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
            trajectory.Extend(ephemeris, Math.Max(ignition, r.Time), r.PlanSteps, r.PlanTolerance, r.ApsisBodies);
            pending.Add((burnNumber - 1, segmentStart, trajectory.LastTime));
            if (!trajectory.Reached)
            {
                if (trajectory.End == PredictionEnd.MaxSteps)
                {
                    notes.Add($"ran out of steps before burn {burnNumber}: raise Steps");
                }
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
                trajectory.Extend(ephemeris, trajectory.LastTime + seconds, BurnSteps, BurnTolerance, r.ApsisBodies,
                                  direction, e.Thrust, mass, e.MassFlow);
                ok = trajectory.Reached;
                if (!ok && trajectory.End == PredictionEnd.MaxSteps)
                {
                    notes.Add($"burn {burnNumber} needs over {BurnSteps:N0} steps: the plan stops there");
                }
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
                trajectory.Extend(ephemeris, r.Time + MaxSpan, r.PlanSteps, r.PlanTolerance, r.ApsisBodies);
            }
            if (burnNumber == r.Burns.Length)
            {
                pending.Add((r.Burns.Length, segmentStart, trajectory.LastTime));
            }
        }
        // Each node's track at once, a thread-pool task each: the plan they
        // start from is done.
        var jobs = new List<(NodeBurn Node, Task<NodeTrack?> Track)>();
        foreach (var (segment, startIndex, shared) in pending)
        {
            foreach (NodeBurn node in segments[segment] ?? [])
            {
                jobs.Add((node, Task.Run(() => NodeTrackOf(system, r, node, segment, trajectory, startIndex,
                                                           shared))));
            }
        }
        foreach (var (node, job) in jobs)
        {
            try
            {
                if (job.Result is { } track)
                {
                    tracks[node.Burn] = track;
                }
            }
            catch (AggregateException e)
            {
                Log.WarnOnce("[predict] a maneuver node's track", e.InnerException ?? e);
            }
        }
        nodes = tracks.Count > 0 ? new NodeTracks(r.Vehicle, r.Frame, tracks, windows) : null;
        return r.Burns.Length > 0
            ? Finish(system, r, trajectory, isFlightPlan: true, Math.Max(firstIgnition, 0),
                     notes.Count > 0 ? string.Join("; ", notes) : null, sw)
            : null;
    }

    // A node's approach track: integrated without burns from the plan's last
    // coast sample before its window, through it. Up to `shared` (the next
    // burn's ignition) it is the plan itself; past it, the path without that
    // burn, which nothing else draws. The plan is only read, by any number
    // of these at once.
    private static NodeTrack? NodeTrackOf(PrincipiaSystem system, Request r, NodeBurn node, int segment,
                                          Trajectory trajectory, int segmentStart, double shared)
    {
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
            double lo = Math.Max(start, Math.Min(node.Time - node.Window, shared));
            int k = first;
            while (k + 1 < trajectory.Times.Count && trajectory.Times[k + 1] <= lo)
            {
                k++;
            }
            t0 = trajectory.Times[k];
            s0 = trajectory.States[k];
        }
        double hi = node.Time + node.Window;
        if (!(hi > t0))
        {
            return null;
        }
        // At the plan's tolerance, to the end of the window.
        Ephemeris.PredictionResult p = ephemeris.Predict(s0, t0, hi, Settings.MaxSteps, r.PlanTolerance,
                                                         double3.Zero, 0, 0, 0, ReadOnlySpan<int>.Empty);
        int count = p.Times.Length;
        if (count < 2)
        {
            return null;
        }
        (Line line, _, _) = Draw(system, r.Frame, p.Times, p.States, count);
        return new NodeTrack(start, line.Times, line.Coordinates, line.Stretches, segment, shared, p.Times,
                             p.States);
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
