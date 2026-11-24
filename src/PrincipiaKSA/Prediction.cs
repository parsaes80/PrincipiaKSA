using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// A line's part in one sphere of influence (KSA's rules). An encounter is drawn about the body met, as KSA draws its
// patches, with a dimmer copy in the plotting frame (Ghost); the rest in the plotting frame. Closed: it ends at a
// sphere crossing, not where the line ends.
internal sealed record Stretch(int Start, int End, FrameSpec Frame, Astronomical Body, double From, double To,
                               bool Closed, double[]? GhostTimes = null, double3[]? Ghost = null);

// Points in their stretch's frame; at a crossing a point at the same time ends one stretch and starts the next.
internal readonly record struct Line(double[] Times, double3[] Coordinates, Stretch[] Stretches)
{
    // Main thread.
    public FrameMapper[] Mappers()
    {
        var mappers = new FrameMapper[Stretches.Length];
        for (int s = 0; s < Stretches.Length; s++)
        {
            mappers[s] = new FrameMapper(Stretches[s].Frame);
        }
        return mappers;
    }

    // The stretch at time t: the later at a crossing.
    public int StretchAt(double t)
    {
        int s = 0;
        while (s + 1 < Stretches.Length && Stretches[s + 1].From <= t)
        {
            s++;
        }
        return s;
    }

    public int StretchOf(int i)
    {
        int s = 0;
        while (s + 1 < Stretches.Length && Stretches[s + 1].Start <= i)
        {
            s++;
        }
        return s;
    }

    public double3 ToEcl(FrameMapper[] mappers, int i) => mappers[StretchOf(i)].ToEcl(Coordinates[i]);

    public double3 EclAt(FrameMapper[] mappers, double t)
    {
        int s = StretchAt(t);
        Stretch stretch = Stretches[s];
        int length = stretch.End - stretch.Start + 1;
        if (length <= 0)
        {
            return mappers[s].ToEcl(double3.Zero);  // No points: never covers t.
        }
        int i = Array.BinarySearch(Times, stretch.Start, length, t);
        double3 c;
        if (i >= 0)
        {
            c = Coordinates[i];
        }
        else if (length < 2)
        {
            c = Coordinates[stretch.Start];
        }
        else
        {
            i = Math.Clamp(~i, stretch.Start + 1, stretch.End);
            double dt = Times[i] - Times[i - 1];
            double f = dt > 0 ? (t - Times[i - 1]) / dt : 1;
            c = double3.Lerp(Coordinates[i - 1], Coordinates[i], Math.Clamp(f, 0, 1));
        }
        return mappers[s].ToEcl(c);
    }

    public double3 EclAt(double t) => EclAt(Mappers(), t);
}

// A finished prediction or flight plan. Immutable once published.
internal sealed class Prediction
{
    public required Vehicle Vehicle { get; init; }
    public required bool IsFlightPlan { get; init; }
    public required FrameSpec Frame { get; init; }
    public required double[] Times { get; init; }          // s since J2000
    public required double3[] FrameCoordinates { get; init; }
    public required Stretch[] Stretches { get; init; }
    public Line Line => new(Times, FrameCoordinates, Stretches);
    public required (int Start, int End)[] BurnArcs { get; init; }  // Point ranges under thrust.
    public required int FirstIgnition { get; init; }       // Index; 0 for a prediction.
    public required PredictionEnd End { get; init; }
    // Ends where the planets are integrated so far; it grows as they are.
    public bool AwaitingPlanets { get; init; }
    public required long Steps { get; init; }              // Integrator steps taken.
    public required bool Impact { get; init; }
    public required List<Marker> Markers { get; init; }
    public required double ComputeMs { get; init; }
    public required string? Note { get; init; }            // E.g. "no engine: burns shown as impulses".
    public required ParentApsides ParentApsides { get; init; }
    public required double[] StateTimes { get; init; }     // The integrated samples, undensified,
    public required QP[] States { get; init; }             // absolute: for directions along the path.
    // Runs of points below the frame centre's near-surface radius, relative to it (Cce axes), for KSA's danger track.
    public required (double Time, double3 Relative)[][] Danger { get; init; }

    // Body: where the body met at an encounter will be then.
    public enum MarkerKind { Periapsis, Apoapsis, ClosestApproach, Burn, Sphere, Body, AscendingNode, DescendingNode }

    // Position in stretch Stretch's frame. Label is for lists; the map shows Name, and Altitude or Detail on hover.
    public sealed record Marker(MarkerKind Kind, double Time, int Stretch, double3 Position, string Label,
                                string Name, double? Altitude = null, string? Detail = null, bool Listed = true);
}

// About the vessel's parent, in time order, for KSA's Ap/Pe gauges and warp buttons.
internal sealed record ParentApsides(Astronomical Parent, (double Time, double Altitude)[] Periapsides,
                                     (double Time, double Altitude)[] Apoapsides)
{
    public (double Time, double Altitude)? Next(bool apoapsis, double now)
    {
        foreach (var a in apoapsis ? Apoapsides : Periapsides)
        {
            if (a.Time >= now)
            {
                return a;
            }
        }
        return null;
    }
}

// The path before a node's burn, which it is drawn and dragged on. Segment: 0 before the first burn with Δv, j after
// the j-th. Shared: until when the flight plan follows the same path.
internal sealed record NodeTrack(double Start, double[] Times, double3[] FrameCoordinates, Stretch[] Stretches,
                                 int Segment, double Shared, double[] StateTimes, QP[] States)
{
    public double End => Times[^1];
    public Line Line => new(Times, FrameCoordinates, Stretches);
}

// Each node's track, and how far a drag step may move it.
internal sealed record NodeTracks(Vehicle Vehicle, FrameSpec Frame, IReadOnlyDictionary<Burn, NodeTrack> ByBurn,
                                  IReadOnlyDictionary<Burn, double> Windows)
{
    // Stopwatch ticks, for the drag log.
    public long Computed { get; } = System.Diagnostics.Stopwatch.GetTimestamp();

    // Each track once (each node has its own, computed in parallel).
    public IReadOnlyList<NodeTrack> Distinct { get; } =
        ByBurn.Values.Distinct(ReferenceEqualityComparer.Instance).Cast<NodeTrack>().ToArray();
}

// The path between samples, as Principia's downsampling assumes: cubic Hermite through positions and velocities.
internal static class Hermite
{
    // At fraction u of the interval h (s) from (q0, v0) to (q1, v1).
    public static double3 Position(double3 q0, double3 v0, double3 q1, double3 v1, double h, double u)
    {
        double u2 = u * u, u3 = u2 * u;
        return q0 * (2 * u3 - 3 * u2 + 1) + v0 * ((u3 - 2 * u2 + u) * h) + q1 * (-2 * u3 + 3 * u2) +
               v1 * ((u3 - u2) * h);
    }

    public static double3 Velocity(double3 q0, double3 v0, double3 q1, double3 v1, double h, double u)
    {
        double u2 = u * u;
        return q0 * ((6 * u2 - 6 * u) / h) + v0 * (3 * u2 - 4 * u + 1) + q1 * ((6 * u - 6 * u2) / h) +
               v1 * (3 * u2 - 2 * u);
    }

    // The state at t between samples i - 1 and i.
    public static QP Between(IReadOnlyList<double> times, IReadOnlyList<QP> states, int i, double t)
    {
        double h = times[i] - times[i - 1], u = (t - times[i - 1]) / h;
        double3 q0 = states[i - 1].Q.ToDouble3(), q1 = states[i].Q.ToDouble3();
        double3 v0 = states[i - 1].P.ToDouble3(), v1 = states[i].P.ToDouble3();
        return new QP(Position(q0, v0, q1, v1, h, u), Velocity(q0, v0, q1, v1, h, u));
    }

    public static QP At(double[] times, QP[] states, double t)
    {
        int i = Array.BinarySearch(times, t);
        return i >= 0 ? states[i] : Between(times, states, Math.Clamp(~i, 1, times.Length - 1), t);
    }
}
