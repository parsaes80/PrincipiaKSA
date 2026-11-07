using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// A part of a drawn line inside one body's sphere of influence (KSA's
// rules), points Start to End, drawn in Frame: the plotting frame where the
// body is its centre, else about the body itself, non-rotating, as KSA draws
// its patches. From and To are its times; Closed, that it ends at a sphere
// crossing (not where the line ends). Drawn about another body, it also has
// its own points in the plotting frame (GhostTimes, Ghost), drawn dimmer, so
// the path shows through in the plotting frame too.
internal sealed record Stretch(int Start, int End, FrameSpec Frame, Astronomical Body, double From, double To,
                               bool Closed, double[]? GhostTimes = null, double3[]? Ghost = null);

// A drawn line: its points, each in its stretch's frame. Where the path
// changes sphere of influence, a point at the same time ends one stretch and
// starts the next, so a line is never joined across stretches.
internal readonly record struct Line(double[] Times, double3[] Coordinates, Stretch[] Stretches)
{
    // Each stretch's frame mapped to Ecl now. Main thread.
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

    // The stretch of point i.
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

    // The point at time t, in Ecl now, within its stretch.
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

// A finished prediction or flight plan, in the coordinates of the plotting
// frame it was computed for, and around other bodies where it is in their
// spheres of influence (Stretches). Immutable once published.
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
    public required bool Impact { get; init; }
    public required List<Marker> Markers { get; init; }
    public required double ComputeMs { get; init; }
    public required string? Note { get; init; }            // E.g. "no engine: burns shown as impulses".
    public required ParentApsides ParentApsides { get; init; }
    public required double[] StateTimes { get; init; }     // The integrated samples, undensified,
    public required QP[] States { get; init; }             // absolute: for directions along the path.
    // Runs of points inside the frame centre's near-surface radius (the top of
    // its atmosphere), relative to it (Cce axes), for KSA's danger track.
    public required (double Time, double3 Relative)[][] Danger { get; init; }

    // Sphere: where the line leaves or enters a sphere of influence; Body:
    // where the body entered will be then.
    public enum MarkerKind { Periapsis, Apoapsis, ClosestApproach, Burn, Sphere, Body }

    // An event at Time and Position, in the frame of stretch Stretch. Label
    // is the full text for lists; the map shows Name, and Altitude or Detail
    // with a countdown when hovered. Listed: in the window's list too (not a
    // crossing's second marker, on the next stretch, nor a body's position).
    public sealed record Marker(MarkerKind Kind, double Time, int Stretch, double3 Position, string Label,
                                string Name, double? Altitude = null, string? Detail = null, bool Listed = true);
}

// A prediction's apsides about the vessel's parent, in time order, for KSA's
// Ap and Pe gauges and warp buttons.
internal sealed record ParentApsides(Astronomical Parent, (double Time, double Altitude)[] Periapsides,
                                     (double Time, double Altitude)[] Apoapsides)
{
    // The first at or after `now` (s since J2000).
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

// A maneuver node's approach: the path before its burn, from the previous
// burn to a little past the node, which it is drawn and dragged on. Segment:
// 0 before the first burn with Δv, j after the j-th. Shared: until when the
// flight plan follows the same path.
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
    // Each track once (nodes between two burns share one).
    public IReadOnlyList<NodeTrack> Distinct { get; } =
        ByBurn.Values.Distinct(ReferenceEqualityComparer.Instance).Cast<NodeTrack>().ToArray();
}

// The bodies a plotting frame is attached to.
internal sealed record FrameSpec(PlotFrame Kind, Astronomical Centre, int CentreIndex,
                                 Astronomical? Secondary, int SecondaryIndex)
{
    public string Describe() => Kind switch
    {
        PlotFrame.BodyCentred => $"centred on {Centre.Id}",
        PlotFrame.BodySurface => $"surface of {Centre.Id}",
        _ => $"rotating with {Centre.Id}–{Secondary?.Id}",
    };
}
