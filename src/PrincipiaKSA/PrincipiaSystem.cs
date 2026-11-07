using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// KSA's star system as a Principia ephemeris: every body with a mass, with
// its μ, rotation and, where Principia has one, geopotential.
//
// KSA's orbits are JPL Horizons elements, which use μ = G(M_parent + M_body);
// KSA evaluates them with G·M_parent alone, misplacing the Moon by ~2,800 km.
// The initial states are rebuilt with the summed μ (the Moon is then within
// 71 km of Principia's JPL ephemeris, see native/tools/sol_reference).
//
// The ephemeris starts at UniverseTime 0, so the sky depends only on game
// time. Coordinates are KSA's ecliptic axes about the barycentre, at rest.
internal sealed class PrincipiaSystem : IDisposable
{
    public static PrincipiaSystem? Current { get; private set; }

    private const double NegligibleAngularFrequency = 1e-10;  // rad/s

    // A body whose parent is also integrated: KSA's state for it, relative to
    // the parent, comes from Principia.
    public readonly record struct Link(int Index, int ParentIndex, IParentBody Parent);

    public Ephemeris Ephemeris { get; }
    public Prolonger Prolonger { get; }
    public IReadOnlyList<Astronomical> Bodies { get; }    // In ephemeris order.
    public IReadOnlyList<Astronomical> Massless { get; }  // Mass 0: left on KSA's orbits.
    public IReadOnlyDictionary<Astronomical, Orbit> OriginalOrbits { get; }

    // Set once on any failure; the bodies then coast on their last orbits.
    public string? FailureReason => _failureReason;
    public bool Failed => _failureReason != null;

    // Every client running its own n-body physics would desync a multiplayer
    // session, so nothing is driven while online.
    public static bool Online => KSA.Networking.Network.IsOnline;
    public bool Driving => !Failed && !Online;

    private readonly Dictionary<Astronomical, int> _index;
    private readonly Dictionary<Astronomical, Link> _links;
    private string? _failureReason;
    private int _inFlight;
    private volatile bool _disposed;

    private PrincipiaSystem(Ephemeris ephemeris, List<Astronomical> bodies, List<Astronomical> massless)
    {
        Ephemeris = ephemeris;
        Bodies = bodies;
        Massless = massless;
        _index = bodies.Select((b, i) => (b, i)).ToDictionary(x => x.b, x => x.i);
        _links = new Dictionary<Astronomical, Link>();
        foreach (Astronomical body in bodies)
        {
            if (body is IOrbiter orbiter && orbiter.Parent is Astronomical parent &&
                _index.TryGetValue(parent, out int parentIndex))
            {
                _links[body] = new Link(_index[body], parentIndex, orbiter.Parent);
            }
        }
        OriginalOrbits = bodies.OfType<IOrbiter>().ToDictionary(o => (Astronomical)o, o => o.Orbit);
        Prolonger = new Prolonger(this);
    }

    public int? IndexOf(Astronomical body) => _index.TryGetValue(body, out int i) ? i : null;

    public bool TryGetLink(Astronomical body, out Link link) => _links.TryGetValue(body, out link);

    public IEnumerable<KeyValuePair<Astronomical, Link>> Links => _links;

    public void Fail(string reason)
    {
        if (Interlocked.CompareExchange(ref _failureReason, reason, null) == null)
        {
            Log.Error($"Principia stopped driving KSA: {reason}. The bodies continue on their " +
                      "last osculating orbits.");
        }
    }

    // Guards native calls from worker threads against Dispose.
    public bool Enter()
    {
        Interlocked.Increment(ref _inFlight);
        if (_disposed || Failed)
        {
            Interlocked.Decrement(ref _inFlight);
            return false;
        }
        return true;
    }

    public void Exit() => Interlocked.Decrement(ref _inFlight);

    // Principia's name for a KSA body in its gravity model.
    public static string GravityModelName(string ksaId) => ksaId switch
    {
        "Sol" => "Sun",
        "Luna" => "Moon",
        _ => ksaId,
    };

    // Replaces Current with an ephemeris of `system` from UniverseTime 0.
    public static PrincipiaSystem Build(CelestialSystem system)
    {
        UniverseTime epoch = UniverseTime.FromNanoseconds(0);
        double t0 = GameTime.ToSecondsSinceJ2000(epoch);

        var bodies = new List<Astronomical>();
        var massless = new List<Astronomical>();
        var all = system.All.OfType<Astronomical>().GetEnumerator();
        while (all.MoveNext())
        {
            Astronomical a = all.Current;
            if (a is Barycenter || a is not IParentBody parentBody)
            {
                continue;  // Barycenters are bookkeeping points.
            }
            (parentBody.Mass > 0 ? bodies : massless).Add(a);
        }

        var absolute = new Dictionary<Astronomical, (double3 Q, double3 P)>();
        var states = new QP[bodies.Count];
        double totalMu = 0;
        double3 muQ = double3.Zero, muP = double3.Zero;
        for (int i = 0; i < bodies.Count; i++)
        {
            double mu = ((IParentBody)bodies[i]).Mu;
            (double3 q, double3 p) = AbsoluteState(bodies[i], epoch, absolute);
            states[i] = new QP(q, p);
            totalMu += mu;
            muQ += q * mu;
            muP += p * mu;
        }
        double3 barycentreQ = muQ / totalMu, barycentreP = muP / totalMu;
        for (int i = 0; i < states.Length; i++)
        {
            states[i] = new QP(states[i].Q.ToDouble3() - barycentreQ,
                               states[i].P.ToDouble3() - barycentreP);
        }

        // Before reading the rotations: locked moons turn at their n-body rate.
        TidalLock.Correct(bodies, states, t0, epoch);

        var specs = new List<BodySpec>(bodies.Count);
        var mismatched = new List<string>();
        int modelled = 0;
        foreach (Astronomical a in bodies)
        {
            var parentBody = (IParentBody)a;
            (double3 pole, double3 meridian, double omega) = Rotation(parentBody, epoch);
            ModelBody? model = MatchModel(a, parentBody, mismatched);
            modelled += model != null ? 1 : 0;
            if (omega == 0)
            {
                // KSA doesn't spin this body (the Sun, comets), but Principia
                // needs a rate: one turn in ~2,000 years. Orient its field by
                // the model's pole when it has one.
                omega = NegligibleAngularFrequency;
                if (model?.Pole is { } modelPole)
                {
                    double3 reference = Math.Abs(modelPole.Z) < 0.9 ? double3.UnitZ : double3.UnitX;
                    pole = modelPole;
                    meridian = double3.Cross(reference, modelPole).Normalized();
                }
            }
            specs.Add(new BodySpec(a.Id, parentBody.Mu, a.MeanRadius, pole, meridian, t0, omega, model?.Name));
        }
        Log.Info($"Gravity model: used for {modelled} of {bodies.Count} bodies; the others are point masses" +
                 (mismatched.Count > 0 ? $"; not used for {string.Join(", ", mismatched)}, unlike KSA's" : ""));

        Ephemeris ephemeris = Ephemeris.Create(specs, states, t0);

        Current?.Dispose();
        Current = new PrincipiaSystem(ephemeris, bodies, massless);
        return Current;
    }

    // KSA's data comes first. Principia's gravity model (the real solar
    // system) only fills in what KSA has none of: a body's gravity field
    // beyond a point mass and, for a body KSA doesn't spin, its pole. It is
    // used only for its own bodies: the same name, and μ and mean radius
    // within 10% of KSA's, so another map's "Earth" stays a point mass.
    private const double ModelTolerance = 0.1;

    private readonly record struct ModelBody(string Name, double3? Pole);

    private static ModelBody? MatchModel(Astronomical a, IParentBody body, List<string> mismatched)
    {
        string name = GravityModelName(a.Id);
        if (Native.GravityModelBody(name, out double mu, out double radius, out double ra, out double dec) !=
            Status.Ok)
        {
            return null;
        }
        bool Near(double model, double ksa) => !double.IsFinite(model) || Math.Abs(model / ksa - 1) <= ModelTolerance;
        if (!Near(mu, body.Mu) || !Near(radius, a.MeanRadius))
        {
            mismatched.Add($"{a.Id} (μ ×{mu / body.Mu:G3}, radius ×{radius / a.MeanRadius:G3})");
            return null;
        }
        return new ModelBody(name, double.IsFinite(ra) && double.IsFinite(dec) ? EclipticPole(ra, dec) : null);
    }

    // IAU 1976 obliquity at J2000, as Horizons' ecliptic uses.
    private const double Obliquity = 84381.448 / 3600 * Math.PI / 180;

    // An ICRS right ascension and declination as a direction in ecliptic axes.
    private static double3 EclipticPole(double ra, double dec)
    {
        double x = Math.Cos(dec) * Math.Cos(ra), y = Math.Cos(dec) * Math.Sin(ra), z = Math.Sin(dec);
        return new double3(x, y * Math.Cos(Obliquity) + z * Math.Sin(Obliquity),
                           -y * Math.Sin(Obliquity) + z * Math.Cos(Obliquity));
    }

    // A body's absolute Ecl state: its parent's plus the two-body state about
    // it. Fixed stars and bodies orbiting a Barycenter keep KSA's state.
    private static (double3 Q, double3 P) AbsoluteState(
        Astronomical a, UniverseTime t, Dictionary<Astronomical, (double3 Q, double3 P)> memo)
    {
        if (memo.TryGetValue(a, out var known))
        {
            return known;
        }
        (double3 Q, double3 P) state;
        if (a is IOrbiter orbiter && orbiter.Parent is Astronomical parent &&
            parent is not Barycenter && orbiter.Parent.Mass > 0)
        {
            (double3 q, double3 p) = TwoBodyStateCce(orbiter.Orbit, ((IParentBody)a).Mass, t);
            (double3 parentQ, double3 parentP) = AbsoluteState(parent, t, memo);
            state = (parentQ + q, parentP + p);
        }
        else
        {
            state = EclState(a, t);
        }
        memo[a] = state;
        return state;
    }

    // State relative to the parent, ecliptic axes, from KSA's elements with
    // μ = G(M_parent + M_body). Orbits with their own central mass in the
    // system definition are used as they are.
    private static (double3 Q, double3 P) TwoBodyStateCce(Orbit orbit, double bodyMass, UniverseTime t)
    {
        IParentBody parent = orbit.Parent;
        doubleQuat cci2Cce = parent.GetCci2Cce();
        if (Math.Abs(orbit.Mu - parent.Mu) > 1e-12 * parent.Mu)
        {
            StateVectors own = orbit.GetStateVectorsAt(t);
            return (own.PositionCci.Transform(cci2Cce), own.VelocityCci.Transform(cci2Cce));
        }
        OrbitData d = orbit.GetData();
        double totalMass = parent.Mass + bodyMass;
        double period = d.IsBound() ? Orbit.Bound.GetPeriod(totalMass, d.SemiMajorAxis) : double.NaN;
        var data = new OrbitData(d.TimeAtPeriapsis, d.Eccentricity, d.SemiMajorAxis, d.SemiMinorAxis,
                                 d.Inclination, d.LongitudeOfAscendingNode, d.ArgumentOfPeriapsis,
                                 period, d.Periapsis, d.Apoapsis, d.Orb2ParentCci);
        Orbit twoBody = Orbit.Create(parent, data, orbit.OrbitLineColor, null, totalMass);
        StateVectors sv = twoBody.GetStateVectorsAt(t);
        return (sv.PositionCci.Transform(cci2Cce), sv.VelocityCci.Transform(cci2Cce));
    }

    // The rotation KSA draws: pole and prime meridian in Ecl at t, and the
    // signed spin rate about the pole.
    private static (double3 Pole, double3 PrimeMeridian, double AngularFrequency) Rotation(
        IParentBody body, UniverseTime t)
    {
        const double dt = 600;
        doubleQuat q0 = body.GetCcf2Cce(t);
        doubleQuat q1 = body.GetCcf2Cce(t + dt);
        double3 pole = double3.UnitZ.Transform(q0);
        double3 m0 = double3.UnitX.Transform(q0);
        double3 m1 = double3.UnitX.Transform(q1);
        double angle = Math.Atan2(double3.Dot(double3.Cross(m0, m1), pole), double3.Dot(m0, m1));
        return (pole, m0, angle / dt);
    }

    private static (double3 Position, double3 Velocity) EclState(Astronomical a, UniverseTime t) =>
        a is IOrbiter orbiter
            ? (orbiter.GetPositionEcl(t), orbiter.GetVelocityEcl(t))
            : (a.GetPositionEcl(), a.GetVelocityEcl());  // A fixed star.

    public static void Clear()
    {
        Current?.Dispose();
        Current = null;
    }

    public void Dispose()
    {
        _disposed = true;
        Prolonger.Dispose();
        // Let worker threads finish their native calls first.
        var wait = System.Diagnostics.Stopwatch.StartNew();
        while (Volatile.Read(ref _inFlight) > 0 && wait.ElapsedMilliseconds < 2000)
        {
            Thread.Yield();
        }
        Ephemeris.Dispose();
    }
}
