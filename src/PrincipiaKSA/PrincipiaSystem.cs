using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// The n-body system: KSA's bodies as a Principia ephemeris (PrincipiaSystem),
// integrated ahead of game time in the background (Prolonger), with KSA's
// spheres of influence over it (Spheres), tidally locked moons fitted to it
// (TidalLock), and paused in multiplayer (Multiplayer).

// KSA's star system as LINQ can read it.
internal static class CelestialSystems
{
    // Every body of type T. (CelestialSystem.All.OfType is KSA's own, not
    // LINQ's: a stack-only enumerator, which can only be walked.)
    public static List<T> BodiesOf<T>(this CelestialSystem system) where T : class
    {
        var bodies = new List<T>();
        var e = system.All.OfType<T>().GetEnumerator();
        while (e.MoveNext())
        {
            bodies.Add(e.Current);
        }
        return bodies;
    }
}

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
    public Spheres Spheres { get; }  // KSA's spheres of influence over Bodies.

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
        Spheres = new Spheres(this);
        Prolonger = new Prolonger(this);
    }

    public int? IndexOf(Astronomical body) => _index.TryGetValue(body, out int i) ? i : null;

    // A body by its id; null for none (or a null id).
    public Astronomical? Find(string? id) => id == null ? null : Bodies.FirstOrDefault(b => b.Id == id);

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
        foreach (Astronomical a in system.BodiesOf<Astronomical>())
        {
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

// Integrates the ephemeris ahead of game time on a background thread, so
// that time warp doesn't stall frames.
internal sealed class Prolonger : IDisposable
{
    // Work in bounded chunks so that Dispose doesn't wait for a huge prolong.
    private const double ChunkSeconds = 30 * 86400;

    // How far ahead of game time the planets are kept integrated, so that
    // lines rarely wait for them: for KSA's 46 bodies a year takes ~2 s of
    // this thread's time and ~30 MB (0.6 MB per body-year). Lines reaching
    // further ask for more as they go (Predictor.AheadOfPlanets).
    public const double Horizon = 365.25 * 86400;

    private readonly PrincipiaSystem _system;
    private readonly Thread _thread;
    private readonly AutoResetEvent _wake = new(false);
    private double _target;  // s since J2000; read and written with Volatile/Interlocked.
    private volatile bool _stop;

    public Prolonger(PrincipiaSystem system)
    {
        _system = system;
        // Below the game's threads: what the game needs next it integrates
        // itself if this thread is behind (EnsureProlonged).
        _thread = new Thread(Run)
        {
            IsBackground = true, Name = "Principia prolong", Priority = ThreadPriority.BelowNormal,
        };
        _thread.Start();
    }

    // Asks the background thread to integrate up to `t`. Never blocks.
    public void Request(double t)
    {
        double current = Volatile.Read(ref _target);
        while (t > current)
        {
            double seen = Interlocked.CompareExchange(ref _target, t, current);
            if (seen == current)
            {
                _wake.Set();
                return;
            }
            current = seen;
        }
    }

    // Integrates up to `t` on this thread if the background one hasn't.
    public void EnsureProlonged(double t)
    {
        if (_system.Ephemeris.TimeRange.TMax < t)
        {
            _system.Ephemeris.Prolong(t);
        }
    }

    private void Run()
    {
        FrameProfiler.MarkBackgroundThread();
        while (!_stop)
        {
            _wake.WaitOne(250);
            try
            {
                double target = Volatile.Read(ref _target);
                double tMax = _system.Ephemeris.TimeRange.TMax;
                while (!_stop && tMax < target)
                {
                    using (FrameProfiler.ProlongAhead.Measure())
                    {
                        _system.Ephemeris.Prolong(Math.Min(target, tMax + ChunkSeconds));
                    }
                    tMax = _system.Ephemeris.TimeRange.TMax;
                }
            }
            catch (Exception e)
            {
                if (!_stop)
                {
                    _system.Fail($"background prolongation failed: {e.Message}");
                }
                return;
            }
        }
    }

    public void Dispose()
    {
        _stop = true;
        _wake.Set();
        _thread.Join();
        _wake.Dispose();
    }
}

// KSA's spheres of influence over Principia's bodies, as KSA moves a vessel
// between them (FiniteBurnPredictor.CrossedAt): out of a body's sphere into
// its parent's, into a child's within it. Indices are the ephemeris'.
internal sealed class Spheres
{
    private readonly int[] _parent;      // -1: a root, or a parent Principia doesn't have.
    private readonly double[] _radius;
    private readonly int[][] _children;  // Those with a finite sphere.

    public Spheres(PrincipiaSystem system)
    {
        int n = system.Bodies.Count;
        _parent = new int[n];
        _radius = new double[n];
        var children = new List<int>[n];
        for (int i = 0; i < n; i++)
        {
            children[i] = new List<int>();
        }
        for (int i = 0; i < n; i++)
        {
            Astronomical body = system.Bodies[i];
            _radius[i] = body is IParentBody p ? p.SphereOfInfluence : 0;
            _parent[i] = body is not IIndependentRoot && body is IOrbiter { Parent: Astronomical up } &&
                         system.IndexOf(up) is int u ? u : -1;
            if (_parent[i] >= 0 && _radius[i] > 0 && double.IsFinite(_radius[i]))
            {
                children[_parent[i]].Add(i);
            }
        }
        _children = children.Select(c => c.ToArray()).ToArray();
    }

    // The body a vessel at `position` (absolute, at t) is about, starting from
    // the one it was about.
    public int Of(Ephemeris ephemeris, int from, double3 position, double t)
    {
        int body = from;
        double Distance(int i) => (position - ephemeris.State(i, t).Q.ToDouble3()).Length();
        while (_parent[body] >= 0 && Distance(body) > _radius[body])
        {
            body = _parent[body];
        }
        for (bool deeper = true; deeper;)
        {
            deeper = false;
            foreach (int child in _children[body])
            {
                if (Distance(child) < _radius[child])
                {
                    body = child;
                    deeper = true;
                    break;
                }
            }
        }
        return body;
    }

    // Whether `body` is a child of `of`, whose sphere it is entered from.
    public bool IsChild(int body, int of) => _parent[body] == of;

    // The body whose sphere `body`'s is in; -1 for none.
    public int ParentOf(int body) => _parent[body];
}

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

// While online, n-body driving pauses (the other players follow KSA's
// conics); back offline, the bodies move to Principia's states again.
internal static class Multiplayer
{
    private static bool? _wasOnline;

    // Main thread, once per frame.
    public static void Tick()
    {
        bool online = PrincipiaSystem.Online;
        if (_wasOnline == online)
        {
            return;
        }
        if (_wasOnline != null || online)
        {
            Log.Info(online ? "Multiplayer session: n-body driving paused" : "Offline: n-body driving resumed");
        }
        bool resumed = _wasOnline == true;
        _wasOnline = online;
        if (resumed)
        {
            CelestialDriver.JumpTo(Universe.GetElapsedTime());
        }
    }
}
