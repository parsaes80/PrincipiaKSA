using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

internal static partial class Predictor
{
    // Where a trajectory first meets the surface of the frame's centre body as
    // KSA draws it. Principia integrates through bodies and the samples can be
    // minutes apart, so each stretch that dips below the highest terrain is
    // walked on a fine grid along the cubic Hermite spline between its two
    // samples, which keeps within the points' 10 m tolerance of the path
    // (Ephemeris.DownsamplingTolerance), and the crossing interpolated.
    private static class Impact
    {
        private const int Points = 4000;       // Per stretch between two samples.
        private const int MaxStretches = 64;   // An orbit skimming the terrain dips every revolution.

        // Ends `trajectory` at its first impact; true if there is one.
        public static bool Truncate(Ephemeris ephemeris, FrameSpec frame, Trajectory trajectory)
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
                if (Crossing(ephemeris, centre, surface, trajectory, i) is { } hit)
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
                                                         Trajectory trajectory, int i)
        {
            double t0 = trajectory.Times[i - 1], t1 = trajectory.Times[i];
            int points = t1 > t0 ? Points : 1;
            double previousHeight = double.NaN;
            double previousTime = t0;
            QP previousState = trajectory.States[i - 1];
            for (int k = 0; k <= points; k++)
            {
                double t = k == points ? t1 : t0 + (t1 - t0) * k / points;
                QP state = k == 0 ? trajectory.States[i - 1]
                         : k == points ? trajectory.States[i]
                         : Patches.GroundTrack.Interpolate(trajectory.Times, trajectory.States, i, t);
                double height = surface.Height(ephemeris, centre, t, state);
                if (height < 0)
                {
                    if (double.IsNaN(previousHeight))
                    {
                        return (t, state);
                    }
                    double f = Math.Clamp(previousHeight / (previousHeight - height), 0, 1);
                    return (previousTime + f * (t - previousTime),
                            new QP(double3.Lerp(previousState.Q.ToDouble3(), state.Q.ToDouble3(), f),
                                   double3.Lerp(previousState.P.ToDouble3(), state.P.ToDouble3(), f)));
                }
                (previousHeight, previousTime, previousState) = (height, t, state);
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
