using System.Diagnostics;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

internal static partial class Predictor
{
    // Frame coordinates, impact, apsides and closest approaches.
    private static Prediction Finish(PrincipiaSystem system, Request r, Trajectory trajectory, bool isFlightPlan,
                                     int firstIgnition, string? note, Stopwatch sw, Buffers b)
    {
        Ephemeris ephemeris = system.Ephemeris;
        FrameSpec frame = r.Frame;
        double radius = frame.Centre.MeanRadius;
        // End the line where it meets the centre body's surface.
        bool impact = Impact.Truncate(ephemeris, frame, trajectory, b);
        int n = trajectory.Times.Count;
        var relative = new QP[n];
        for (int i = 0; i < n; i++)
        {
            QP centre = ephemeris.State(frame.CentreIndex, trajectory.Times[i]);
            relative[i] = new QP(trajectory.States[i].Q.ToDouble3() - centre.Q.ToDouble3(),
                                 trajectory.States[i].P.ToDouble3() - centre.P.ToDouble3());
        }
        (Line line, int[] denseIndex, List<Crossing> crossings) =
            Draw(system, frame, trajectory.Times, trajectory.States, n, ghosts: true);
        var markers = new List<Prediction.Marker>();
        double endTime = n > 0 ? trajectory.Times[n - 1] : r.Time;
        // A point in the frame of stretch s (by default the one at time t).
        (int Stretch, double3 Position) Place(double t, double3 q, int s = -1)
        {
            s = s < 0 ? line.StretchAt(t) : s;
            FrameSpec f = line.Stretches[s].Frame;
            QP centre = ephemeris.State(f.CentreIndex, t);
            return (s, ToFrame(ephemeris, f, t, q, centre, q - centre.Q.ToDouble3()));
        }
        Prediction.Marker Marker(Prediction.MarkerKind kind, double t, double3 q, string label, string name,
                                 double? altitude = null, string? detail = null, int s = -1, bool listed = true)
        {
            (int stretch, double3 position) = Place(t, q, s);
            return new(kind, t, stretch, position, label, name, altitude, detail, listed);
        }
        foreach (var (time, state, label) in trajectory.Burns)
        {
            if (time <= endTime)
            {
                // "Burn 2: Δv 12.3 m/s, 45 s": the name, then the detail.
                int colon = label.IndexOf(": ", StringComparison.Ordinal);
                markers.Add(Marker(Prediction.MarkerKind.Burn, time, state.Q.ToDouble3(), label,
                                   colon < 0 ? label : label[..colon], detail: colon < 0 ? null : label[(colon + 2)..]));
            }
        }
        // Where the line leaves one sphere of influence (at the end of its
        // stretch) and enters the next (at the start of the next), and on an
        // encounter, where the body will be then, as KSA marks its patches.
        Spheres spheres = Spheres.Of(system);
        foreach (Crossing c in crossings)
        {
            int into = line.StretchAt(c.Time);
            if (into == 0)
            {
                continue;
            }
            Astronomical from = system.Bodies[c.From], to = system.Bodies[c.To];
            // As KSA names them: into a moon's sphere, "Luna Encounter"; out
            // of it, "Luna Escape" (and "Earth Escape" out to the Sun).
            bool encounter = spheres.IsChild(c.To, c.From);
            string name = encounter ? $"{to.Id} Encounter" : $"{from.Id} Escape";
            string detail = encounter ? $"into {to.Id}'s sphere of influence"
                                      : $"out of {from.Id}'s sphere of influence, into {to.Id}'s";
            double3 q = c.State.Q.ToDouble3();
            markers.Add(Marker(Prediction.MarkerKind.Sphere, c.Time, q, $"{name}: {detail}", name, detail: detail,
                               s: into - 1));
            markers.Add(Marker(Prediction.MarkerKind.Sphere, c.Time, q, $"{name}: {detail}", name, detail: detail,
                               s: into, listed: false));
            if (encounter && line.Stretches[into - 1].Frame != line.Stretches[into].Frame)
            {
                markers.Add(Marker(Prediction.MarkerKind.Body, c.Time, ephemeris.State(c.To, c.Time).Q.ToDouble3(),
                                   $"{to.Id} at the encounter", to.Id, detail: "at the encounter", s: into - 1,
                                   listed: false));
            }
        }

        // Apsides about the centre, and closest approaches within twice an SOI.
        var closest = new Dictionary<int, Apsis>();
        foreach (Apsis a in trajectory.Apsides)
        {
            if (a.Time > endTime)
            {
                continue;
            }
            if (a.Body == frame.CentreIndex)
            {
                string name = a.IsApoapsis != 0 ? "Ap" : "Pe";
                markers.Add(Marker(a.IsApoapsis != 0 ? Prediction.MarkerKind.Apoapsis
                                                     : Prediction.MarkerKind.Periapsis,
                                   a.Time, a.State.Q.ToDouble3(), $"{name} {Altitude(a.Distance - radius)}", name,
                                   a.Distance - radius));
            }
            else if (a.IsApoapsis == 0 && (!closest.TryGetValue(a.Body, out Apsis best) || a.Distance < best.Distance))
            {
                closest[a.Body] = a;
            }
        }
        foreach (Astronomical body in r.Candidates)
        {
            if (system.IndexOf(body) is int index && closest.TryGetValue(index, out Apsis a))
            {
                double soi = body is IParentBody pb && double.IsFinite(pb.SphereOfInfluence) ? pb.SphereOfInfluence : 0;
                if (a.Distance < 2 * soi)
                {
                    markers.Add(Marker(Prediction.MarkerKind.ClosestApproach, a.Time, a.State.Q.ToDouble3(),
                                       $"{body.Id} {Altitude(a.Distance - body.MeanRadius)}", body.Id,
                                       a.Distance - body.MeanRadius));
                }
            }
        }
        markers.Sort((x, y) => x.Time.CompareTo(y.Time));

        int Dense(int i) => denseIndex[Math.Clamp(i, 0, Math.Max(n - 1, 0))];
        return new Prediction
        {
            Vehicle = r.Vehicle,
            IsFlightPlan = isFlightPlan,
            Frame = frame,
            Times = line.Times,
            FrameCoordinates = line.Coordinates,
            Stretches = line.Stretches,
            BurnArcs = trajectory.BurnArcs.Where(a => a.Item1 < n)
                                 .Select(a => (Dense(a.Item1), Dense(Math.Min(a.Item2, n - 1)))).ToArray(),
            FirstIgnition = n > 0 ? Dense(Math.Min(firstIgnition, n - 1)) : 0,
            End = trajectory.End,
            Impact = impact,
            Markers = markers,
            ComputeMs = sw.Elapsed.TotalMilliseconds,
            Note = note,
            ParentApsides = new ParentApsides(r.Parent, Apsides(0), Apsides(1)),
            Danger = Danger(frame, trajectory.Times, relative),
            StateTimes = trajectory.Times.ToArray(),
            States = trajectory.States.ToArray(),
        };

        (double, double)[] Apsides(int apoapsis) => trajectory.Apsides
            .Where(a => a.Body == r.ParentIndex && a.IsApoapsis == apoapsis && a.Time <= endTime)
            .OrderBy(a => a.Time)
            .Select(a => (a.Time, a.Distance - r.Parent.MeanRadius))
            .ToArray();
    }

    // Long predictions are sampled coarsely, which would draw polygonal, so
    // each interval is split along the Hermite curve of its end points until
    // the path turns at most MaxTurn per piece (within Budget points). Returns
    // the dense times and points, and each sample's index among them.
    private const double MaxTurn = 3 * Math.PI / 180;
    private const int Budget = 60000;

    // The runs of the path inside the centre's near-surface radius, as KSA's
    // danger zone (IParentBody.GetNearSurfaceRadius), every few seconds.
    private const double DangerStep = 4;

    private static (double, double3)[][] Danger(FrameSpec frame, List<double> times, QP[] relative)
    {
        var runs = new List<(double, double3)[]>();
        if (frame.Centre is not IParentBody body || body.GetNearSurfaceRadius() is not (> 0 and var limit))
        {
            return runs.ToArray();
        }
        var run = new List<(double, double3)>();
        void Close()
        {
            if (run.Count > 1)
            {
                runs.Add(run.ToArray());
            }
            run.Clear();
        }
        for (int i = 1; i < times.Count; i++)
        {
            if (!(relative[i - 1].Q.ToDouble3().Length() < limit || relative[i].Q.ToDouble3().Length() < limit))
            {
                Close();
                continue;
            }
            double t0 = times[i - 1], t1 = times[i];
            int steps = Math.Clamp((int)Math.Ceiling((t1 - t0) / DangerStep), 1, 256);
            for (int k = run.Count == 0 ? 0 : 1; k <= steps; k++)
            {
                double t = t0 + (t1 - t0) * k / steps;
                double3 q = Patches.GroundTrack.Interpolate(times, relative, i, t).Q.ToDouble3();
                if (q.Length() < limit)
                {
                    run.Add((t, q));
                }
                else
                {
                    Close();
                }
            }
        }
        Close();
        return runs.ToArray();
    }

    // Where a path goes from one sphere of influence into another.
    private readonly record struct Crossing(double Time, QP State, int From, int To);

    // A path's points to draw, as KSA draws its patches: split where it
    // changes sphere of influence, at the crossing found between the
    // samples, each stretch in its frame (Stretch: the plotting frame in the
    // centre's sphere, else about the stretch's body), densified; with
    // `ghosts`, the latter also in the plotting frame. Also each sample's
    // index among the points.
    private static (Line Line, int[] Index, List<Crossing> Crossings) Draw(
        PrincipiaSystem system, FrameSpec plot, IReadOnlyList<double> times, IReadOnlyList<QP> states, int n,
        bool ghosts = false)
    {
        Ephemeris ephemeris = system.Ephemeris;
        Spheres spheres = Spheres.Of(system);
        var body = new int[n];
        for (int i = 0; i < n; i++)
        {
            body[i] = spheres.Of(ephemeris, i == 0 ? plot.CentreIndex : body[i - 1], states[i].Q.ToDouble3(),
                                 times[i]);
        }

        var points = new List<double3>(n);
        var pointTimes = new List<double>(n);
        var stretches = new List<Stretch>();
        var crossings = new List<Crossing>();
        var index = new int[Math.Max(n, 1)];
        // The stretch being built: its samples, and which of them are the
        // trajectory's (-1: a crossing).
        var pieceTimes = new List<double>();
        var pieceStates = new List<QP>();
        var pieceSamples = new List<int>();
        int pieceBody = n > 0 ? body[0] : plot.CentreIndex;

        void Add(double t, QP state, int sample)
        {
            pieceTimes.Add(t);
            pieceStates.Add(state);
            pieceSamples.Add(sample);
        }
        void Close(bool closed)
        {
            FrameSpec frame = pieceBody != plot.CentreIndex
                ? new FrameSpec(PlotFrame.BodyCentred, system.Bodies[pieceBody], pieceBody, null, -1)
                : plot;
            int m = pieceTimes.Count;
            var coordinates = new double3[m];
            var velocities = new double3[m];
            for (int k = 0; k < m; k++)
            {
                QP centre = ephemeris.State(frame.CentreIndex, pieceTimes[k]);
                (coordinates[k], velocities[k]) = FrameState(ephemeris, frame, pieceTimes[k], pieceStates[k], centre);
            }
            int budget = Math.Max(1000, (int)((long)Budget * m / Math.Max(n, 1)));
            (double[] dt, double3[] dp, int[] di) = Densify(pieceTimes, coordinates, velocities, m, budget);
            // The same stretch in the plotting frame, to draw dimmer.
            double[]? ghostTimes = null;
            double3[]? ghost = null;
            if (ghosts && frame != plot && m > 1)
            {
                for (int k = 0; k < m; k++)
                {
                    QP centre = ephemeris.State(plot.CentreIndex, pieceTimes[k]);
                    (coordinates[k], velocities[k]) =
                        FrameState(ephemeris, plot, pieceTimes[k], pieceStates[k], centre);
                }
                (ghostTimes, ghost, _) = Densify(pieceTimes, coordinates, velocities, m, budget / 2);
            }
            int offset = points.Count;
            pointTimes.AddRange(dt);
            points.AddRange(dp);
            for (int k = 0; k < m; k++)
            {
                if (pieceSamples[k] >= 0)
                {
                    index[pieceSamples[k]] = offset + di[k];
                }
            }
            double from = m > 0 ? pieceTimes[0] : double.NaN, to = m > 0 ? pieceTimes[m - 1] : double.NaN;
            stretches.Add(new Stretch(offset, offset + dt.Length - 1, frame, system.Bodies[pieceBody], from, to,
                                      closed, ghostTimes, ghost));
            pieceTimes.Clear();
            pieceStates.Clear();
            pieceSamples.Clear();
        }

        for (int i = 0; i < n; i++)
        {
            if (i > 0 && body[i] != body[i - 1])
            {
                // Bisected on the Hermite curve between the samples.
                double lo = times[i - 1], hi = times[i];
                QP at = states[i];
                if (hi > lo)
                {
                    for (int k = 0; k < 64 && hi - lo > 1e-3; k++)
                    {
                        double mid = 0.5 * (lo + hi);
                        QP q = Patches.GroundTrack.Interpolate(times, states, i, mid);
                        if (spheres.Of(ephemeris, body[i - 1], q.Q.ToDouble3(), mid) == body[i - 1])
                        {
                            lo = mid;
                        }
                        else
                        {
                            hi = mid;
                        }
                    }
                    at = Patches.GroundTrack.Interpolate(times, states, i, hi);
                }
                crossings.Add(new Crossing(hi, at, body[i - 1], body[i]));
                Add(hi, at, -1);
                Close(closed: true);
                pieceBody = body[i];
                Add(hi, at, -1);
            }
            Add(times[i], states[i], i);
        }
        Close(closed: false);
        return (new Line(pointTimes.ToArray(), points.ToArray(), stretches.ToArray()), index, crossings);
    }

    private static (double[] Times, double3[] Positions, int[] Index) Densify(
        List<double> times, double3[] positions, double3[] velocities, int n, int budget)
    {
        var index = new int[Math.Max(n, 1)];
        if (n < 2)
        {
            return (times.Take(n).ToArray(), positions[..n], index);
        }
        var pieces = new int[n - 1];
        double maxTurn = MaxTurn;
        for (int attempt = 0; attempt < 8; attempt++)
        {
            long total = 1;
            for (int i = 0; i < n - 1; i++)
            {
                double dt = times[i + 1] - times[i];
                double a = Angle(velocities[i], velocities[i + 1]);
                pieces[i] = dt > 0 && double.IsFinite(a) ? (int)Math.Clamp(Math.Ceiling(a / maxTurn), 1, 64) : 1;
                total += pieces[i];
            }
            if (total <= budget)
            {
                break;
            }
            maxTurn *= (double)total / budget;  // Coarser, evenly, to fit.
        }
        int count = 1 + pieces.Sum();
        var denseTimes = new double[count];
        var densePositions = new double3[count];
        int k = 0;
        denseTimes[k] = times[0];
        densePositions[k++] = positions[0];
        for (int i = 0; i < n - 1; i++)
        {
            double t0 = times[i], dt = times[i + 1] - t0;
            for (int j = 1; j < pieces[i]; j++)
            {
                double u = (double)j / pieces[i], u2 = u * u, u3 = u2 * u;
                denseTimes[k] = t0 + u * dt;
                densePositions[k++] = (2 * u3 - 3 * u2 + 1) * positions[i] + (u3 - 2 * u2 + u) * dt * velocities[i] +
                                      (-2 * u3 + 3 * u2) * positions[i + 1] + (u3 - u2) * dt * velocities[i + 1];
            }
            index[i + 1] = k;
            denseTimes[k] = times[i + 1];
            densePositions[k++] = positions[i + 1];
        }
        return (denseTimes, densePositions, index);
    }

    private static double Angle(double3 a, double3 b)
    {
        double norms = a.Length() * b.Length();
        return norms > 0 ? Math.Acos(Math.Clamp(double3.Dot(a, b) / norms, -1, 1)) : double.NaN;
    }

    // An absolute state in the frame's coordinates, its velocity as seen in
    // the frame (less Ω × c in a rotating one).
    private static (double3 Position, double3 Velocity) FrameState(Ephemeris ephemeris, FrameSpec frame, double t,
                                                                  QP state, QP centre)
    {
        double3 q = state.Q.ToDouble3(), v = state.P.ToDouble3();
        double3 rel = q - centre.Q.ToDouble3(), relV = v - centre.P.ToDouble3();
        switch (frame.Kind)
        {
            case PlotFrame.BodySurface:
            {
                doubleQuat cce2Ccf = ((IParentBody)frame.Centre).GetCce2Ccf(GameTime.FromSecondsSinceJ2000(t));
                double3 c = rel.Transform(cce2Ccf);
                double omega = frame.Centre is Celestial celestial ? celestial.GetAngularVelocity() : 0;
                return (c, relV.Transform(cce2Ccf) - double3.Cross(new double3(0, 0, omega), c));
            }
            case PlotFrame.BarycentricRotating:
            {
                QP secondary = ephemeris.State(frame.SecondaryIndex, t);
                double mu1 = ((IParentBody)frame.Centre).Mu, mu2 = ((IParentBody)frame.Secondary!).Mu;
                double3 q1 = centre.Q.ToDouble3(), v1 = centre.P.ToDouble3();
                double3 q2 = secondary.Q.ToDouble3(), v2 = secondary.P.ToDouble3();
                (double3 origin, double3 x, double3 y, double3 z) = RotatingAxes(mu1, q1, v1, mu2, q2, v2);
                double3 vOrigin = (mu1 * v1 + mu2 * v2) / (mu1 + mu2);
                double3 d = q - origin, dv = v - vOrigin;
                var c = new double3(double3.Dot(d, x), double3.Dot(d, y), double3.Dot(d, z));
                var cv = new double3(double3.Dot(dv, x), double3.Dot(dv, y), double3.Dot(dv, z));
                // The axes turn about z at the pair's orbital rate.
                double3 r12 = q2 - q1;
                double rate = double3.Cross(r12, v2 - v1).Length() / double3.Dot(r12, r12);
                return (c, cv - double3.Cross(new double3(0, 0, rate), c));
            }
            default:
                return (rel, relV);
        }
    }

    // An absolute position in the frame: about the centre in Ecl axes, or in
    // its rotating axes, or about the pair's barycentre with x towards the
    // secondary and z along their orbital angular momentum.
    private static double3 ToFrame(Ephemeris ephemeris, FrameSpec frame, double t, double3 q, QP centre,
                                   double3 relativeToCentre)
    {
        switch (frame.Kind)
        {
            case PlotFrame.BodySurface:
                return relativeToCentre.Transform(((IParentBody)frame.Centre)
                                                  .GetCce2Ccf(GameTime.FromSecondsSinceJ2000(t)));
            case PlotFrame.BarycentricRotating:
            {
                QP secondary = ephemeris.State(frame.SecondaryIndex, t);
                (double3 origin, double3 x, double3 y, double3 z) =
                    RotatingAxes(((IParentBody)frame.Centre).Mu, centre.Q.ToDouble3(), centre.P.ToDouble3(),
                                 ((IParentBody)frame.Secondary!).Mu, secondary.Q.ToDouble3(), secondary.P.ToDouble3());
                double3 d = q - origin;
                return new double3(double3.Dot(d, x), double3.Dot(d, y), double3.Dot(d, z));
            }
            default:
                return relativeToCentre;
        }
    }

    public static (double3 Origin, double3 X, double3 Y, double3 Z) RotatingAxes(
        double mu1, double3 q1, double3 v1, double mu2, double3 q2, double3 v2)
    {
        double3 origin = (mu1 * q1 + mu2 * q2) / (mu1 + mu2);
        double3 x = (q2 - q1).Normalized();
        double3 z = double3.Cross(q2 - q1, v2 - v1).Normalized();
        double3 y = double3.Cross(z, x);
        return (origin, x, y, z);
    }

    public static string Altitude(double metres) => Math.Abs(metres) switch
    {
        < 1e4 => $"{metres:F0} m",
        < 1e7 => $"{metres / 1000:F1} km",
        _ => $"{metres / 1000:N0} km",
    };
}
