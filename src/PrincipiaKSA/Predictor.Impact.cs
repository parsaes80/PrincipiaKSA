using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

internal static partial class Predictor
{
    // Where a trajectory first meets the surface of the frame's centre body as
    // KSA draws it. Principia integrates through bodies and the samples can be
    // minutes apart, so each stretch that dips below the highest terrain is
    // integrated again on a fine grid and the crossing interpolated.
    private static class Impact
    {
        private const int Points = 4000;       // Per stretch: under 0.3 s apart.
        private const int MaxStretches = 64;   // An orbit skimming the terrain dips every revolution.

        // Ends `trajectory` at its first impact; true if there is one.
        public static bool Truncate(Ephemeris ephemeris, FrameSpec frame, Trajectory trajectory, Buffers b)
        {
            int n = trajectory.Times.Count;
            if (n == 0)
            {
                return false;
            }
            var surface = new Surface(frame.Centre);
            int centre = frame.CentreIndex;
            var periapsides = trajectory.Apsides
                .Where(a => a.Body == centre && a.IsApoapsis == 0 && a.Distance < surface.Highest)
                .Select(a => a.Time).Order().ToList();
            double Distance(int i) =>
                (trajectory.States[i].Q.ToDouble3() - ephemeris.State(centre, trajectory.Times[i]).Q.ToDouble3())
                .Length();

            if (Distance(0) < surface.Highest &&
                surface.Height(ephemeris, centre, trajectory.Times[0], trajectory.States[0]) < 0)
            {
                Cut(trajectory, 1, trajectory.Times[0], trajectory.States[0]);
                return true;
            }
            int p = 0, stretches = 0;
            for (int i = 1; i < n && stretches < MaxStretches; i++)
            {
                double t0 = trajectory.Times[i - 1], t1 = trajectory.Times[i];
                while (p < periapsides.Count && periapsides[p] < t0)
                {
                    p++;
                }
                if (!(Distance(i) < surface.Highest || (p < periapsides.Count && periapsides[p] <= t1)))
                {
                    continue;
                }
                stretches++;
                if (Crossing(ephemeris, centre, surface, trajectory, i, b) is { } hit)
                {
                    Cut(trajectory, i, hit.Time, hit.State);
                    return true;
                }
            }
            // A collision Principia stopped at without a sample below ground.
            return trajectory.End == PredictionEnd.Collision;
        }

        // Keeps the first `keep` samples and appends the impact.
        private static void Cut(Trajectory trajectory, int keep, double time, QP state)
        {
            int n = trajectory.Times.Count;
            if (keep < n)
            {
                trajectory.Times.RemoveRange(keep, n - keep);
                trajectory.States.RemoveRange(keep, n - keep);
            }
            if (keep == 0 || trajectory.Times[^1] < time)
            {
                trajectory.Times.Add(time);
                trajectory.States.Add(state);
            }
        }

        // The first crossing of the surface between samples i - 1 and i.
        private static (double Time, QP State)? Crossing(Ephemeris ephemeris, int centre, Surface surface,
                                                         Trajectory trajectory, int i, Buffers b)
        {
            double t0 = trajectory.Times[i - 1], t1 = trajectory.Times[i];
            QP s0 = trajectory.States[i - 1];
            int count;
            if (trajectory.BurnArcs.Any(a => i > a.Item1 && i <= a.Item2))
            {
                // Under thrust: just the two samples.
                b.Times[0] = t0;
                b.States[0] = s0;
                b.Times[1] = t1;
                b.States[1] = trajectory.States[i];
                count = 2;
            }
            else
            {
                (count, _, _) = ephemeris.Predict(
                    s0, t0, t1, Settings.MaxSteps, double3.Zero, 0, 0, 0, (t1 - t0) / Points,
                    b.Times, b.States, ReadOnlySpan<int>.Empty, b.Apsides);
            }
            double previousHeight = double.NaN;
            for (int k = 0; k < count; k++)
            {
                double height = surface.Height(ephemeris, centre, b.Times[k], b.States[k]);
                if (height >= 0)
                {
                    previousHeight = height;
                    continue;
                }
                if (double.IsNaN(previousHeight))
                {
                    return (b.Times[k], b.States[k]);
                }
                double f = Math.Clamp(previousHeight / (previousHeight - height), 0, 1);
                QP a = b.States[k - 1], c = b.States[k];
                return (b.Times[k - 1] + f * (b.Times[k] - b.Times[k - 1]),
                        new QP(double3.Lerp(a.Q.ToDouble3(), c.Q.ToDouble3(), f),
                               double3.Lerp(a.P.ToDouble3(), c.P.ToDouble3(), f)));
            }
            return null;
        }
    }

    // A body's surface as KSA draws it: the mean radius plus the height map in
    // that direction, never below sea level where there is an ocean.
    private readonly struct Surface
    {
        private readonly Astronomical _body;
        private readonly Celestial? _terrain;
        private readonly bool _ocean;
        private readonly double _meanRadius;
        public readonly double Highest;

        public Surface(Astronomical body)
        {
            _body = body;
            _meanRadius = body.MeanRadius;
            Highest = _meanRadius;
            _terrain = null;
            _ocean = false;
            if (body is IParentBody parent)
            {
                Highest = Math.Max(_meanRadius, parent.MaxTerrainRadius) + 1;
                _ocean = parent.GetOceanReference() != null;
            }
            if (body is Celestial { HasTerrainHeightmap: true } celestial)
            {
                _terrain = celestial;
            }
        }

        // Height above the surface at `state`; negative below it.
        public double Height(Ephemeris ephemeris, int centre, double t, QP state)
        {
            double3 relative = state.Q.ToDouble3() - ephemeris.State(centre, t).Q.ToDouble3();
            double r = relative.Length();
            double ground = _meanRadius;
            if (_terrain != null && r < Highest && r > 0)
            {
                try
                {
                    doubleQuat ccf2Cce = ((IParentBody)_body).GetCcf2Cce(GameTime.FromSecondsSinceJ2000(t));
                    double3 directionCcf = (relative / r).Transform(ccf2Cce.Inverse());
                    double terrain = _terrain.GetTerrainHeightFromDirCcf(directionCcf, accurate: false);
                    ground += _ocean ? Math.Max(terrain, 0) : terrain;
                }
                catch (Exception e)
                {
                    Log.WarnOnce("[predict] terrain height", e);
                }
            }
            return r - ground;
        }
    }
}
