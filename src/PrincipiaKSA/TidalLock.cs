using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// KSA spins a tidally locked body at its Kepler rate, but its n-body mean
// motion differs (Luna: 27.07 d against the real 27.32 d), so the near side
// would turn away. Each locked pair's mean motion is fitted from a point-mass
// integration over up to a year and becomes the spin rate, with the epoch
// angle shifted so the same face points at the partner.
internal static class TidalLock
{
    private static readonly AccessTools.FieldRef<Celestial, double> AngularVelocity =
        AccessTools.FieldRefAccess<Celestial, double>("AngularVelocity");
    private static readonly AccessTools.FieldRef<Celestial, double> InitialRotation =
        AccessTools.FieldRefAccess<Celestial, double>("InitialRotation");

    private const double Year = 365.25 * 86400;

    public static void Correct(IReadOnlyList<Astronomical> bodies, QP[] states, double t0, UniverseTime epoch)
    {
        var pairs = FindLockedPairs(bodies);
        if (pairs.Count == 0)
        {
            return;
        }

        var index = bodies.Select((b, i) => (b, i)).ToDictionary(x => x.b, x => x.i);
        var specs = bodies.Select(b => new BodySpec(b.Id, ((IParentBody)b).Mu)).ToList();
        using Ephemeris pointMasses = Ephemeris.Create(specs, states, t0);

        foreach (var (body, partner, kspPeriod) in pairs)
        {
            double horizon = Math.Clamp(10 * kspPeriod, 30 * 86400, Year);
            if (horizon < 3 * kspPeriod)
            {
                // Too slow to fit over a year.
                Log.Info($"[tidal lock] {body.Id}: period {kspPeriod / 86400:F1} d too long to measure; left unchanged");
                continue;
            }
            pointMasses.Prolong(t0 + horizon);
            int samples = (int)Math.Clamp(16 * horizon / kspPeriod, 64, 20000);
            doubleQuat cce2Cci = body.GetCci2Cce().Inverse();

            double Direction(double t)
            {
                QP b = pointMasses.State(index[body], t), p = pointMasses.State(index[partner], t);
                double3 d = (p.Q.ToDouble3() - b.Q.ToDouble3()).Transform(cce2Cci);
                return Math.Atan2(d.Y, d.X);
            }

            // Unwrap the partner's direction and fit a line: slope = mean motion.
            double sumT = 0, sumA = 0, sumTT = 0, sumTA = 0, previous = Direction(t0), unwrapped = previous;
            for (int k = 0; k <= samples; k++)
            {
                double dt = horizon * k / samples;
                double angle = Direction(t0 + dt);
                double step = Math.IEEERemainder(angle - previous, 2 * Math.PI);
                unwrapped += k == 0 ? 0 : step;
                previous = angle;
                sumT += dt; sumA += unwrapped; sumTT += dt * dt; sumTA += dt * unwrapped;
            }
            int n = samples + 1;
            double meanMotion = (n * sumTA - sumT * sumA) / (n * sumTT - sumT * sumT);

            double oldOmega = AngularVelocity(body);
            if (Math.Sign(meanMotion) != Math.Sign(oldOmega) || Math.Abs(meanMotion / oldOmega - 1) > 0.2)
            {
                Log.Warn($"[tidal lock] {body.Id}: measured mean motion {meanMotion:E6} rad/s is far from " +
                         $"KSA's {oldOmega:E6}; left unchanged");
                continue;
            }

            // Shift by how much the corrected initial state moved the partner.
            double3 ksaDirection = KsaDirection(body, partner, epoch).Transform(cce2Cci);
            double shift = Math.IEEERemainder(Direction(t0) - Math.Atan2(ksaDirection.Y, ksaDirection.X),
                                              2 * Math.PI);
            AngularVelocity(body) = meanMotion;
            InitialRotation(body) += shift;
            Log.Info($"[tidal lock] {body.Id} faces {partner.Id}: period " +
                     $"{2 * Math.PI / oldOmega / 86400:F4} d → {2 * Math.PI / meanMotion / 86400:F4} d " +
                     $"(n-body mean over {horizon / 86400:F0} d), epoch rotation shifted {shift * 180 / Math.PI:F3}°");
        }
    }

    // Bodies whose spin KSA ties to an orbit: moons locked to their parent
    // (IsTidallyLocked) and planets locked to a moon (Pluto to Charon).
    private static List<(Celestial Body, Astronomical Partner, double Period)> FindLockedPairs(
        IReadOnlyList<Astronomical> bodies)
    {
        var simulated = bodies.ToHashSet();
        var pairs = new List<(Celestial, Astronomical, double)>();
        foreach (Celestial c in bodies.OfType<Celestial>())
        {
            double omega = AngularVelocity(c);
            if (omega == 0)
            {
                continue;
            }
            if ((bool)c.BodyTemplate.Rotation.IsTidallyLocked && c.Parent is Astronomical parent &&
                simulated.Contains(parent) && double.IsFinite(c.Orbit.Period))
            {
                pairs.Add((c, parent, c.Orbit.Period));
                continue;
            }
            foreach (IOrbiter child in c.Children)
            {
                if (child is Celestial moon && simulated.Contains(moon) && double.IsFinite(moon.Orbit.Period) &&
                    Math.Abs(Math.Abs(omega) * moon.Orbit.Period / (2 * Math.PI) - 1) < 1e-9)
                {
                    pairs.Add((c, moon, moon.Orbit.Period));
                    break;
                }
            }
        }
        return pairs;
    }

    // Direction from `body` to `partner` in Ecl axes per KSA's own orbits.
    private static double3 KsaDirection(Celestial body, Astronomical partner, UniverseTime t)
    {
        if (body.Parent == partner)
        {
            return -body.GetPositionCce(t);  // body relative to parent
        }
        return ((IOrbiter)partner).GetPositionCce(t);  // moon relative to body
    }
}
