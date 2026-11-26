using System.Diagnostics;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// The controlled vessel's n-body lines in the background: the prediction on one thread, the flight plan through KSA's
// burns on another, then each node's track as a pool task. The main thread posts a snapshot each frame; each thread
// works on the latest (Principia's ephemeris can be flowed from several threads at once).
internal static class Predictor
{
    // ── The results the threads publish, and starting and stopping them ─────────

    public static volatile Prediction? Latest;      // The prediction.
    public static volatile Prediction? LatestPlan;  // The flight plan.
    public static volatile NodeTracks? LatestNodes; // The maneuver nodes' approaches.

    // What everything else reads: null while hidden, not computed yet, or for another vessel.
    public static Prediction? PredictionFor(Vehicle? vehicle) =>
        Settings.ShowPrediction && Latest is { } p && p.Vehicle == vehicle ? p : null;

    public static Prediction? PlanFor(Vehicle? vehicle) =>
        Settings.ShowFlightPlan && LatestPlan is { } p && p.Vehicle == vehicle ? p : null;

    public static NodeTracks? NodesFor(Vehicle? vehicle) => LatestNodes is { } n && n.Vehicle == vehicle ? n : null;

    // Until it next succeeds.
    private static string? _snapshotError;
    public static string? LastError => _snapshotError ?? CoastWorker.Error ?? PlanWorker.Error;

    // For the status log.
    public static long PredictionsComputed => Interlocked.Read(ref CoastWorker.Computed);
    public static long PlansComputed => Interlocked.Read(ref PlanWorker.Computed);

    public static void Start()
    {
        _stop = false;
        CoastWorker.Start();
        PlanWorker.Start();
    }

    public static void Stop()
    {
        _stop = true;
        CoastWorker.Stop();
        PlanWorker.Stop();
        Clear();
    }

    // Shown in the window; logged when it changes, in full the first time.
    private static void Failed(ref string? slot, string where, Exception e)
    {
        if (e.Message != slot)
        {
            Log.Warn($"[predict] prediction {where} failed: {e.Message}");
            Log.WarnOnce($"[predict] prediction {where}: {e.Message}", e);
        }
        slot = e.Message;
    }

    private static void Clear()
    {
        Latest = LatestPlan = null;
        LatestNodes = null;
    }

    // ── The request: what the main thread posts each frame for the threads ──────

    private sealed record Request(Vehicle Vehicle,
                                  QP State, double Time, FrameSpec Frame,
                                  int[] ApsisBodies, Astronomical[] Candidates, long CoastSteps,
                                  double CoastTolerance, PlannedBurn[] Burns, EnginePhase[] Phases, double Mass,
                                  long PlanSteps, double PlanTolerance, NodeBurn[] Nodes, bool ShowPlan,
                                  int ParentIndex, Astronomical Parent, TargetPlane? Target)
    {
        // Stopwatch ticks.
        public long Requested { get; } = Stopwatch.GetTimestamp();
    }

    // For aN and dN: the target's orbital plane about its parent, as a unit normal in Ecl axes.
    private sealed record TargetPlane(string Name, double3 Normal, int ParentIndex, Astronomical Parent);

    // As KSA: nodes only with a target, on its plane about the body it orbits.
    private static TargetPlane? PlaneOf(PrincipiaSystem system, Vehicle vehicle)
    {
        if (vehicle.Target is not { } target || target.Orbit is not { } orbit ||
            orbit.Parent is not Astronomical parent || system.IndexOf(parent) is not int parentIndex)
        {
            return null;
        }
        double3 normal = orbit.GetOrbitNormalCci().Transform(orbit.Parent.GetCci2Cce());
        double length = normal.Length();
        return length > 0 && double.IsFinite(length)
            ? new TargetPlane(target.Id, normal / length, parentIndex, parent)
            : null;
    }

    // As the flight computer flies it: from Ignition along a fixed direction; a burn not yet targeted also carries its
    // Δv in KSA's axes about ParentIndex, to be aimed from Principia's state (BurnDirection).
    private sealed record PlannedBurn(double Impulse, double? Ignition, double DeltaV, double3 Direction,
                                      double3? DeltaVVlf = null, int ParentIndex = -1);

    // Staging, as KSA's finite-burn planning has it.
    private sealed record EnginePhase(double Thrust, double MassFlow, double StartMass, double MaxSeconds);

    private sealed record NodeBurn(Burn Burn, double Time, double Window);

    private static readonly long MinInterval = Stopwatch.Frequency / 120;
    // Coasting with nothing changed gives the same path again; the map draws only what is ahead of now.
    private static readonly long IdleInterval = Stopwatch.Frequency / 2;
    private static int _lastInputs;

    // Main thread, once per frame. Never throws.
    public static void Tick()
    {
        try
        {
            Snapshot();
            _snapshotError = null;
        }
        catch (Exception e)
        {
            Failed(ref _snapshotError, "snapshot", e);
            Clear();
        }
    }

    private static void Snapshot()
    {
        if (!Settings.ShowPrediction && !Settings.ShowFlightPlan)
        {
            Clear();
            return;
        }
        long now = Stopwatch.GetTimestamp();
        if (now - _lastRequest < MinInterval)
        {
            return;
        }
        PrincipiaSystem? system = PrincipiaSystem.Current;
        if (system == null || system.Failed || Program.ControlledVehicle is not { } vehicle ||
            vehicle.Situation.HasAnyContact())
        {
            Clear();  // Nothing to predict, or landed.
            return;
        }
        Orbit orbit = vehicle.Orbit;
        if (orbit.Parent is not Astronomical parent || system.IndexOf(parent) is not int parentIndex ||
            FrameSpec.FromSettings(system, parent) is not { } frame)
        {
            Clear();
            return;
        }
        int inputs = Inputs(vehicle, frame);
        bool growing = Latest is { AwaitingPlanets: true } || LatestPlan is { AwaitingPlanets: true };
        if (inputs == _lastInputs && vehicle.Situation == Situation.Freefall && !growing &&
            now - _lastRequest < IdleInterval)
        {
            return;
        }
        _lastInputs = inputs;
        _lastRequest = now;
        StateVectors sv = orbit.StateVectors;
        double t = GameTime.ToSecondsSinceJ2000(sv.StateTime);
        QP p = system.Ephemeris.State(parentIndex, t);
        doubleQuat cci2Cce = orbit.Parent.GetCci2Cce();
        var fromKsa = new QP(p.Q.ToDouble3() + sv.PositionCci.Transform(cci2Cce),
                             p.P.ToDouble3() + sv.VelocityCci.Transform(cci2Cce));
        // The exact state: KSA's conic is only within the rebuild tolerances, enough to move a circular orbit's apsides.
        QP state = fromKsa;
        if (VesselDriver.TryGetTrack(vehicle, sv.StateTime.Nanoseconds, out QP track, out UniverseTime trackTime))
        {
            state = track;
            t = GameTime.ToSecondsSinceJ2000(trackTime);
        }
        FlightComputer fc = vehicle.FlightComputer;
        Astronomical[] candidates = Candidates(system, frame.Centre);
        // The frame's centre for markers, the parent for KSA's gauges, the others for closest approaches.
        int[] apsisBodies = new[] { frame.CentreIndex, parentIndex }
            .Concat(candidates.Select(c => system.IndexOf(c) ?? -1).Where(i => i >= 0)).Distinct().ToArray();
        Volatile.Write(ref _request, new Request(
            vehicle, state, t, frame, apsisBodies, candidates,
            Settings.ShowPrediction ? Settings.PredictionSteps : 0, Settings.PredictionTolerance,
            Burns(vehicle, t), Phases(fc), fc.TotalMassPropsBody.Mass,
            Settings.PlanSteps, Settings.PlanTolerance, Nodes(vehicle, t), Settings.ShowFlightPlan,
            parentIndex, parent, PlaneOf(system, vehicle)));
        CoastWorker.Wake();
        PlanWorker.Wake();
    }

    // What the lines depend on besides the vessel's own motion: any change asks for them at once.
    private static int Inputs(Vehicle vehicle, FrameSpec frame)
    {
        var hash = new HashCode();
        hash.Add(vehicle);
        hash.Add(vehicle.Orbit.Parent);
        hash.Add(frame);
        hash.Add(vehicle.Target);
        hash.Add((Settings.ShowPrediction, Settings.ShowFlightPlan, Settings.PredictionSteps,
                  Settings.PredictionTolerance, Settings.PlanSteps, Settings.PlanTolerance));
        foreach (Burn burn in vehicle.BurnPlan.All())
        {
            hash.Add((burn.Time.Nanoseconds, burn.DeltaVVlf.X, burn.DeltaVVlf.Y, burn.DeltaVVlf.Z));
        }
        FlightComputer fc = vehicle.FlightComputer;
        if (fc.Burn is { } target)
        {
            hash.Add((target.ImpulsiveInstant.Nanoseconds, target.StartedAt.HasValue));
        }
        hash.Add(Math.Round(Math.Log(Math.Max(fc.TotalMassPropsBody.Mass, 1e-9)) * 1000));  // To 0.1%: life support.
        return hash.ToHashCode();
    }

    // Still ahead or being flown, along the flight computer's target, else the burn's (anchored) patch.
    private static PlannedBurn[] Burns(Vehicle vehicle, double now)
    {
        var burns = new List<PlannedBurn>();
        FlightComputer fc = vehicle.FlightComputer;
        BurnTarget? target = fc.Burn;
        foreach (Burn burn in vehicle.BurnPlan.All())
        {
            if (!burn.HasDeltaV || burn.IsInterstellar)
            {
                continue;
            }
            try
            {
                AddBurn(burns, vehicle, burn, target, now);
            }
            catch (Exception e)
            {
                // Without it, rather than no lines at all (its node holds on its last anchor).
                if (BurnFailures.Add($"{e.GetType().Name}: {e.Message}"))
                {
                    Log.Warn($"[predict] burn {Describe(burn, now)} left out of the plan: {e}");
                }
            }
        }
        burns.Sort((a, b) => a.Impulse.CompareTo(b.Impulse));
        return burns.ToArray();
    }

    private static readonly HashSet<string> BurnFailures = new();  // Each kind logged once. Main thread.

    private static string Describe(Burn burn, double now)
    {
        string text = $"in {(GameTime.ToSecondsSinceJ2000(burn.Time) - now) / 3600:F1} h, Δv {burn.DeltaVVlf}";
        try
        {
            PatchedConic patch = burn.Patch;
            Orbit o = patch.Orbit;
            bool anchored = BurnAnchor.Get(burn) is { } a && ReferenceEquals(a.Patch, patch);
            text += $", patch about {(o.Parent as IObjectId)?.Id} ({(anchored ? "Principia's" : "KSA's")}): " +
                    $"{o.GetType().Name}, e {o.Eccentricity}, period {o.Period} s, " +
                    $"{GameTime.ToSecondsSinceJ2000(patch.StartTime) - now:F0}–" +
                    $"{GameTime.ToSecondsSinceJ2000(patch.EndTime) - now:F0} s";
        }
        catch (Exception e)
        {
            text += $", patch unavailable ({e.GetType().Name})";
        }
        return text;
    }

    private static void AddBurn(List<PlannedBurn> burns, Vehicle vehicle, Burn burn, BurnTarget? target, double now)
    {
        if (burn.ParentDepartureBurn)
        {
            // As KSA previews it until it is made a vessel burn: on the vessel's path at ParentDepartureTime.
            double departure = GameTime.ToSecondsSinceJ2000(burn.ParentDepartureTime);
            if (departure > now && vehicle.FlightPlan.TryFindPatch(burn.ParentDepartureTime) is { } own)
            {
                StateVectors state = own.Orbit.GetStateVectorsAt(burn.ParentDepartureTime);
                double3 along = burn.DeltaVVlf.Transform(state.GetVlf2ParentCci().OrIdentity())
                    .Transform(own.Orbit.Parent.GetCci2Cce());
                int ownParent = own.Orbit.Parent is Astronomical body &&
                                PrincipiaSystem.Current?.IndexOf(body) is int ownIndex ? ownIndex : -1;
                burns.Add(new PlannedBurn(departure, null, burn.DeltaVVlf.Length(), along.Normalized(),
                                          burn.DeltaVVlf, ownParent));
            }
            return;
        }
        double impulse = GameTime.ToSecondsSinceJ2000(burn.ImpulseTime);
        PatchedConic patch = burn.Patch;
        doubleQuat cci2Cce = patch.Orbit.Parent.GetCci2Cce();
        if (target != null && target.ImpulsiveInstant == burn.ImpulseTime)
        {
            if (target.StartedAt.HasValue)
            {
                // Being flown: the rest of it, from now.
                float3 toGo = target.DeltaVToGoCci;
                double3 toGoCce = new double3(toGo.X, toGo.Y, toGo.Z).Transform(cci2Cce);
                if (toGoCce.Length() > 1e-3)
                {
                    burns.Add(new PlannedBurn(impulse, now, toGoCce.Length(), toGoCce.Normalized()));
                }
                return;
            }
            float3 dv = target.DeltaVTargetCci;
            // A target loaded while the burn had no Δv keeps none inside the alignment window: plan the burn itself.
            if (dv.Length() > 0)
            {
                double ignition = GameTime.ToSecondsSinceJ2000(target.IgnitionTime);
                burns.Add(new PlannedBurn(impulse, double.IsFinite(ignition) && ignition < impulse ? ignition : null,
                                          dv.Length(), new double3(dv.X, dv.Y, dv.Z).Transform(cci2Cce).Normalized()));
                return;
            }
        }
        if (impulse <= now)
        {
            return;
        }
        StateVectors at = patch.Orbit.GetStateVectorsAt(burn.ImpulseTime);
        double3 direction = burn.DeltaVVlf.Transform(at.GetVlf2ParentCci().OrIdentity()).Transform(cci2Cce);
        int parentIndex = patch.Orbit.Parent is Astronomical parentBody &&
                          PrincipiaSystem.Current?.IndexOf(parentBody) is int index ? index : -1;
        burns.Add(new PlannedBurn(impulse, null, burn.DeltaVVlf.Length(), direction.Normalized(),
                                  burn.DeltaVVlf, parentIndex));
    }

    // How far a node's track reaches either side (a drag step: 0.9 of it): half the orbit, so it can go anywhere at once;
    // unbound, half the time to it. 10 min to 60 days, as the track is integrated every update.
    public static double NodeWindow(Orbit orbit, double ahead)
    {
        double period = orbit.Period;
        double half = orbit.IsBound() && double.IsFinite(period) && period > 0 ? period / 2 : ahead / 2;
        return double.IsFinite(half) ? Math.Clamp(half, 600, 60 * 86400.0) : 3600;
    }

    // With or without Δv.
    private static NodeBurn[] Nodes(Vehicle vehicle, double now)
    {
        var nodes = new List<NodeBurn>();
        foreach (Burn burn in vehicle.BurnPlan.All())
        {
            if (burn.IsInterstellar || burn.ParentDepartureBurn)
            {
                continue;  // A departure's node is on its parent's orbit: KSA's.
            }
            double time = GameTime.ToSecondsSinceJ2000(burn.Time);
            if (!(time > now))
            {
                continue;
            }
            nodes.Add(new NodeBurn(burn, time, NodeWindow(burn.Patch.Orbit, time - now)));
        }
        return nodes.ToArray();
    }

    // KSA's staged-burn phases, else the active engines at full throttle.
    private static EnginePhase[] Phases(FlightComputer fc)
    {
        var phases = new List<EnginePhase>();
        foreach (FiniteBurnPhase phase in fc.StagedBurn)
        {
            if (phase.Thrust > 0)
            {
                phases.Add(new EnginePhase(phase.Thrust, phase.MassFlowRate, phase.StartMass, phase.MaxSeconds));
            }
        }
        if (phases.Count == 0 && fc.ActiveEnginePerformanceMax.Thrust > 0)
        {
            phases.Add(new EnginePhase(fc.ActiveEnginePerformanceMax.Thrust, fc.ActiveEnginePerformanceMax.MassFlowRate,
                                       fc.TotalMassPropsBody.Mass, double.PositiveInfinity));
        }
        return phases.ToArray();
    }

    // Closest approaches worth showing: the centre's moons, and for a moon, its planet and siblings.
    private static Astronomical[] Candidates(PrincipiaSystem system, Astronomical centre)
    {
        var result = new List<Astronomical>();
        void AddChildren(IParentBody body)
        {
            foreach (IOrbiter child in body.Children)
            {
                if (child is Astronomical a && a != centre && system.IndexOf(a) != null && !result.Contains(a))
                {
                    result.Add(a);
                }
            }
        }
        if (centre is IParentBody centreBody)
        {
            AddChildren(centreBody);
        }
        if (centre is IOrbiter { Parent: Astronomical planet and not StellarBody } && system.IndexOf(planet) != null)
        {
            result.Add(planet);
            AddChildren((IParentBody)planet);
        }
        return result.ToArray();
    }

    // ── The threads ─────────────────────────────────────────────────────────────

    // Permanent, as they run all the time (a thread costs ~50 µs to start, a pool task ~0.5 µs).
    private static readonly Worker CoastWorker = new("Principia prediction", "prediction", FrameProfiler.Prediction,
                                                     ComputeAndPublishCoast, () => Latest = null);
    private static readonly Worker PlanWorker = new("Principia flight plan", "flight plan", FrameProfiler.FlightPlan,
                                                    ComputeAndPublishPlan, () => (LatestPlan, LatestNodes) = (null, null));
    private static volatile bool _stop;
    private static Request? _request;
    private static long _lastRequest;

    // Computes each new request; `compute` publishes its result and returns it for the log.
    private sealed class Worker(string threadName, string what, FrameProfiler.Section profiled,
                                Func<PrincipiaSystem, Request, Prediction?> compute, Action clear)
    {
        private readonly AutoResetEvent _wake = new(false);
        private Thread? _thread;

        // Until it next succeeds.
        public string? Error;
        public long Computed;

        public void Start()
        {
            if (_thread != null)
            {
                return;
            }
            _thread = new Thread(Run) { IsBackground = true, Name = threadName };
            _thread.Start();
        }

        public void Wake() => _wake.Set();

        public void Stop()
        {
            _wake.Set();
            _thread?.Join();
            _thread = null;
        }

        private void Run()
        {
            FrameProfiler.MarkBackgroundThread();
            Request? done = null;
            while (!_stop)
            {
                _wake.WaitOne(500);
                Request? request = Volatile.Read(ref _request);
                if (request == null || ReferenceEquals(request, done) || PrincipiaSystem.Current is not { } system ||
                    !system.Enter())
                {
                    continue;
                }
                done = request;
                try
                {
                    long started = Stopwatch.GetTimestamp();
                    PredictionTiming.Begin();
                    Prediction? result;
                    using (profiled.Measure())
                    {
                        result = compute(system, request);
                    }
                    PredictionTiming.End(what, Stopwatch.GetElapsedTime(started).TotalMilliseconds,
                                         Stopwatch.GetElapsedTime(request.Requested, started).TotalMilliseconds,
                                         result);
                    Error = null;
                    Interlocked.Increment(ref Computed);
                }
                catch (Exception e)
                {
                    Failed(ref Error, "computation", e);
                    clear();
                }
                finally
                {
                    system.Exit();
                }
            }
        }
    }

    // The vessel coasting until its steps run out.
    private static Prediction? ComputeAndPublishCoast(PrincipiaSystem system, Request r)
    {
        if (r.CoastSteps <= 0)
        {
            return Latest = null;
        }
        var sw = Stopwatch.StartNew();
        var trajectory = new Trajectory(r.State, r.Time);
        trajectory.Extend(system.Ephemeris, r.Time + MaxSpan, r.CoastSteps, r.CoastTolerance, r.ApsisBodies);
        return Latest = Finish(system, r, trajectory, isFlightPlan: false, firstIgnition: 0, note: null, sw);
    }

    // With the nodes' tracks.
    private static Prediction? ComputeAndPublishPlan(PrincipiaSystem system, Request r)
    {
        NodeTracks? nodes = null;
        Prediction? plan = r.Burns.Length > 0 || r.Nodes.Length > 0 ? ComputePlan(system, r, out nodes) : null;
        LatestPlan = r.ShowPlan ? plan : null;
        LatestNodes = nodes;
        return plan;
    }

    // ── Integrating: the prediction, the flight plan and the nodes' tracks ──────

    // As Principia's DefaultBurnParameters, whatever the coasts' settings: per engine phase.
    private const long BurnSteps = 1000;
    private const double BurnTolerance = 1;

    // Steps left or not, as the ephemeris it needs costs memory.
    public const double MaxSpanYears = 20;
    private const double MaxSpan = MaxSpanYears * 365.25 * 86400;

    // How far past the integrated planets a line may go in one computation (they cost ~6 ms per day), and how far
    // ahead of where it stopped it asks the background thread for.
    private const double AheadOfPlanets = 86400;
    private const double PlanetRequestSeconds = 30 * 86400;

    private static double PlanetsReadyUntil(Ephemeris ephemeris, double wanted) =>
        Math.Min(wanted, ephemeris.TimeRange.TMax + AheadOfPlanets);

    // Only for a line stopped short for the planets: asking for every line's MaxSpan would integrate twenty years.
    private static void WantPlanets(double reached, double wanted) =>
        PrincipiaSystem.Current?.Prolonger.Request(Math.Min(wanted, reached + PlanetRequestSeconds));

    // Where the path crosses the target's plane while in its parent's sphere, and its inclination there (degrees).
    private static List<(double Time, double3 Q, bool Ascending, double Inclination)> PlaneCrossings(
        Ephemeris ephemeris, Line line, Trajectory trajectory, TargetPlane plane)
    {
        var nodes = new List<(double, double3, bool, double)>();
        double3 n = plane.Normal;
        int count = trajectory.Times.Count;
        double[] times = new double[2];
        QP[] about = new QP[2];
        QP Relative(int i)
        {
            QP parent = ephemeris.State(plane.ParentIndex, trajectory.Times[i]);
            return new QP(trajectory.States[i].Q.ToDouble3() - parent.Q.ToDouble3(),
                          trajectory.States[i].P.ToDouble3() - parent.P.ToDouble3());
        }
        bool InParentSphere(double t) => line.Stretches.Length > 0 && line.Stretches[line.StretchAt(t)].Body == plane.Parent;
        double previous = double.NaN;
        QP previousState = default;
        for (int i = 0; i < count; i++)
        {
            double t = trajectory.Times[i];
            if (!InParentSphere(t))
            {
                previous = double.NaN;
                continue;
            }
            QP state = Relative(i);
            double side = double3.Dot(state.Q.ToDouble3(), n);
            if (double.IsFinite(previous) && (previous < 0) != (side < 0) && t > trajectory.Times[i - 1])
            {
                // Bisected on the Hermite curve between the samples.
                (times[0], times[1]) = (trajectory.Times[i - 1], t);
                (about[0], about[1]) = (previousState, state);
                double lo = times[0], hi = times[1];
                for (int k = 0; k < 60 && hi - lo > 1e-6; k++)
                {
                    double mid = 0.5 * (lo + hi);
                    double s = double3.Dot(Hermite.Between(times, about, 1, mid).Q.ToDouble3(), n);
                    (lo, hi) = (s < 0) == (previous < 0) ? (mid, hi) : (lo, mid);
                }
                double at = 0.5 * (lo + hi);
                QP node = Hermite.Between(times, about, 1, at);
                double3 h = double3.Cross(node.Q.ToDouble3(), node.P.ToDouble3());
                double inclination = Math.Acos(Math.Clamp(double3.Dot(h.Normalized(), n), -1, 1)) * (180 / Math.PI);
                double3 absolute = node.Q.ToDouble3() + ephemeris.State(plane.ParentIndex, at).Q.ToDouble3();
                nodes.Add((at, absolute, side >= 0, inclination));
            }
            previous = side;
            previousState = state;
        }
        return nodes;
    }

    private sealed class Trajectory
    {
        public readonly List<double> Times = new();
        public readonly List<QP> States = new();
        public readonly List<(int, int)> BurnArcs = new();
        public readonly List<Apsis> Apsides = new();
        public readonly List<(double Time, QP State, string Label)> Burns = new();
        public PredictionEnd End = PredictionEnd.ReachedEnd;
        // Stopped with steps left where the planets end; the next computation goes further.
        public bool AwaitingPlanets;
        public long Steps;
        public QP Last;
        public double LastTime;
        public bool CollectApsides = true;

        public Trajectory(QP state, double t) => (Last, LastTime) = (state, t);

        // Until `maxSteps`, time `t` (+∞: none) or a body; End says why. Returns the steps taken.
        public long Extend(Ephemeris ephemeris, double t, long maxSteps, double tolerance, int[] apsisBodies,
                           double3 direction = default, double thrust = 0, double mass = 0, double massFlow = 0)
        {
            AwaitingPlanets = false;
            if (!(t > LastTime))
            {
                End = PredictionEnd.ReachedEnd;
                return 0;
            }
            double until = PlanetsReadyUntil(ephemeris, t);
            if (!(until > LastTime))
            {
                End = PredictionEnd.ReachedEnd;
                AwaitingPlanets = true;
                WantPlanets(LastTime, t);
                return 0;
            }
            Ephemeris.PredictionResult p = ephemeris.Predict(Last, LastTime, until, maxSteps, tolerance, direction,
                                                             thrust, mass, massFlow,
                                                             CollectApsides ? apsisBodies : []);
            End = p.End;
            AwaitingPlanets = p.End == PredictionEnd.ReachedEnd && until < t;
            if (AwaitingPlanets)
            {
                WantPlanets(until, t);
            }
            Steps += p.Steps;
            for (int i = Times.Count == 0 ? 0 : 1; i < p.Times.Length; i++)
            {
                Times.Add(p.Times[i]);
                States.Add(p.States[i]);
            }
            Apsides.AddRange(p.Apsides);
            Last = p.States[^1];
            LastTime = p.Times[^1];
            return p.Steps;
        }

        public bool Reached => End == PredictionEnd.ReachedEnd && !AwaitingPlanets;
    }

    // As Principia's flight plan: coasts to each ignition, burns to its Δv through the engine phases, coasts on after
    // the last; a coast out of steps before a burn ends the plan, as Principia's anomalous manœuvres.
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
        var engines = new Engines(r.Phases, r.Phases.Length > 0 ? r.Phases[0].StartMass : r.Mass);
        int firstIgnition = -1;
        int burnNumber = 0;
        foreach (PlannedBurn burn in r.Burns)
        {
            burnNumber++;
            double duration = engines.Duration(burn.DeltaV);
            double ignition = burn.Ignition ?? burn.Impulse - duration / 2;
            segmentStart = trajectory.Times.Count - 1;
            trajectory.Extend(ephemeris, Math.Max(ignition, r.Time), r.PlanSteps, r.PlanTolerance, r.ApsisBodies);
            pending.Add((burnNumber - 1, segmentStart, trajectory.LastTime));
            if (!trajectory.Reached)
            {
                if (trajectory.AwaitingPlanets)
                {
                    notes.Add($"integrating the planets ahead, up to burn {burnNumber}");
                }
                else if (trajectory.End == PredictionEnd.MaxSteps)
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
            while (remaining > 1e-6 && !engines.Done && ok)
            {
                var (e, seconds, phaseEnds) = engines.Next(remaining);
                if (e.MassFlow > 0)
                {
                    // Never all the mass: the native side checks it.
                    seconds = Math.Min(seconds, 0.999999 * engines.Mass / e.MassFlow);
                }
                if (!double.IsFinite(seconds) || seconds <= 0)
                {
                    break;
                }
                trajectory.Extend(ephemeris, trajectory.LastTime + seconds, BurnSteps, BurnTolerance, r.ApsisBodies,
                                  direction, e.Thrust, engines.Mass, e.MassFlow);
                ok = trajectory.Reached;
                if (!ok && trajectory.AwaitingPlanets)
                {
                    notes.Add($"integrating the planets ahead, through burn {burnNumber}");
                }
                else if (!ok && trajectory.End == PredictionEnd.MaxSteps)
                {
                    notes.Add($"burn {burnNumber} needs over {BurnSteps:N0} steps: the plan stops there");
                }
                remaining -= engines.Burn(e, seconds, phaseEnds);
            }
            trajectory.BurnArcs.Add((start, trajectory.Times.Count - 1));
            if (!ok)
            {
                break;
            }
        }
        if (trajectory.Reached)
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
        // A pool task per node track, now that the plan they start from is done.
        var jobs = new List<(NodeBurn Node, Task<NodeTrack?> Track)>();
        foreach (var (segment, startIndex, shared) in pending)
        {
            foreach (NodeBurn node in segments[segment] ?? [])
            {
                jobs.Add((node, Task.Run(() =>
                {
                    using var background = new FrameProfiler.BackgroundScope();
                    using var profiled = FrameProfiler.NodeTrack.Measure();
                    return NodeTrackOf(system, r, node, segment, trajectory, startIndex, shared);
                })));
            }
        }
        var waiting = PredictionTiming.Stage("waiting for node tracks");
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
        waiting.Dispose();
        nodes = tracks.Count > 0 ? new NodeTracks(r.Vehicle, r.Frame, tracks, windows) : null;
        return r.Burns.Length > 0
            ? Finish(system, r, trajectory, isFlightPlan: true, Math.Max(firstIgnition, 0),
                     notes.Count > 0 ? string.Join("; ", notes) : null, sw)
            : null;
    }

    // Without burns, from the plan's last coast sample before the node's window through it: the plan up to `shared`
    // (the next ignition), then the path without that burn. The plan is only read, by any number of these at once.
    private static NodeTrack? NodeTrackOf(PrincipiaSystem system, Request r, NodeBurn node, int segment,
                                          Trajectory trajectory, int segmentStart, double shared)
    {
        Ephemeris ephemeris = system.Ephemeris;
        double start, t0;
        QP s0;
        // Starting partway along the plan, in the plan's sphere there: an encounter is drawn about its body, and the
        // track's must be too, or the node would be drawn elsewhere (about the Sun at Saturn's place, not at Saturn).
        (int Body, bool Encounter)? sphere = null;
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
            sphere = SphereAt(system, r.Frame, trajectory.Times, trajectory.States, k);
        }
        // To the end of the window, as far as the planets are ready.
        double wanted = node.Time + node.Window;
        double hi = PlanetsReadyUntil(ephemeris, wanted);
        if (hi < wanted)
        {
            WantPlanets(hi, wanted);
        }
        if (!(hi > t0))
        {
            return null;
        }
        // At the plan's tolerance.
        Ephemeris.PredictionResult p = ephemeris.Predict(s0, t0, hi, Settings.MaxSteps, r.PlanTolerance,
                                                         double3.Zero, 0, 0, 0, ReadOnlySpan<int>.Empty);
        int count = p.Times.Length;
        if (count < 2)
        {
            return null;
        }
        (Line line, _, _) = Draw(system, r.Frame, p.Times, p.States, count, start: sphere);
        return new NodeTrack(start, line.Times, line.Coordinates, line.Stretches, segment, shared, p.Times,
                             p.States);
    }

    // Its Δv in the axes of Principia's state at the burn time, about the burn's body; KSA's for a targeted burn.
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

    // Where a sequence of burns is in the engine phases; a value, copied to look ahead (Duration).
    private struct Engines(EnginePhase[] phases, double mass)
    {
        public int Phase;
        public double Mass = mass;
        public double Used;  // Seconds burned in the current phase.

        public readonly bool Done => Phase >= phases.Length;

        // Toward `deltaV`, up to the end of the current phase.
        public readonly (EnginePhase Engine, double Seconds, bool PhaseEnds) Next(double deltaV)
        {
            EnginePhase e = phases[Phase];
            double left = e.MaxSeconds - Used;
            double seconds = BurnSeconds(e.Thrust, e.MassFlow, Mass, deltaV);
            return seconds >= left ? (e, left, true) : (e, seconds, false);
        }

        // Returns the Δv gained, staging where the phase ends.
        public double Burn(EnginePhase e, double seconds, bool phaseEnds)
        {
            double gained = DeltaV(e.Thrust, e.MassFlow, Mass, seconds);
            Mass -= e.MassFlow * seconds;
            Used += seconds;
            if (phaseEnds)
            {
                Phase++;
                Used = 0;
                if (!Done)
                {
                    Mass = phases[Phase].StartMass;  // Staging.
                }
            }
            return gained;
        }

        public readonly double Duration(double deltaV)
        {
            Engines ahead = this;
            double total = 0;
            while (deltaV > 1e-6 && !ahead.Done)
            {
                var (e, seconds, ends) = ahead.Next(deltaV);
                total += seconds;
                if (!ends)
                {
                    break;
                }
                deltaV -= ahead.Burn(e, seconds, ends);
            }
            return total;
        }
    }

    // The rocket equation.
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

    // ── From samples to lines: frames, spheres of influence, markers ────────────

    private static Prediction Finish(PrincipiaSystem system, Request r, Trajectory trajectory, bool isFlightPlan,
                                     int firstIgnition, string? note, Stopwatch sw)
    {
        Ephemeris ephemeris = system.Ephemeris;
        FrameSpec frame = r.Frame;
        double radius = frame.Centre.MeanRadius;
        bool impact;
        using (PredictionTiming.Stage("impact search"))
        {
            impact = Impact.Truncate(ephemeris, frame, trajectory);
        }
        int n = trajectory.Times.Count;
        var relative = new QP[n];
        using (PredictionTiming.Stage("states about the centre body"))
        {
            for (int i = 0; i < n; i++)
            {
                QP centre = ephemeris.State(frame.CentreIndex, trajectory.Times[i]);
                relative[i] = new QP(trajectory.States[i].Q.ToDouble3() - centre.Q.ToDouble3(),
                                     trajectory.States[i].P.ToDouble3() - centre.P.ToDouble3());
            }
        }
        Line line;
        int[] denseIndex;
        List<Crossing> crossings;
        using (PredictionTiming.Stage("line: frames and spheres of influence"))
        {
            (line, denseIndex, crossings) =
                Draw(system, frame, trajectory.Times, trajectory.States, n, ghosts: true);
        }
        var markersTiming = PredictionTiming.Stage("markers");
        var markers = new List<Prediction.Marker>();
        double endTime = n > 0 ? trajectory.Times[n - 1] : r.Time;
        // In the frame of stretch s, by default the one at time t.
        Prediction.Marker Marker(Prediction.MarkerKind kind, double t, double3 q, string label, string name,
                                 double? altitude = null, string? detail = null, int s = -1, bool listed = true)
        {
            s = s < 0 ? line.StretchAt(t) : s;
            FrameSpec f = line.Stretches[s].Frame;
            QP centre = ephemeris.State(f.CentreIndex, t);
            double3 position = f.Coordinates(ephemeris, t, new QP(q, double3.Zero), centre).Position;
            return new(kind, t, s, position, label, name, altitude, detail, listed);
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
        // As KSA marks its patches: where the line leaves a sphere and enters the next, and where a body met will be.
        Spheres spheres = system.Spheres;
        // Visits to a sphere are numbered, an encounter starting the next; leaving a moon's back into its planet's is no
        // new visit, and where the line starts, that sphere and those around it are visit 1.
        var visits = new Dictionary<int, int>();
        if (line.Stretches.Length > 0 && system.IndexOf(line.Stretches[0].Body) is int first)
        {
            for (int b = first; b >= 0; b = spheres.ParentOf(b))
            {
                visits[b] = 1;
            }
        }
        var numbered = new List<(Crossing Crossing, bool Encounter, int Body, int Visit)>();
        foreach (Crossing c in crossings)
        {
            bool encounter = spheres.IsChild(c.To, c.From);
            int body = encounter ? c.To : c.From;
            int visit = visits.GetValueOrDefault(body) + (encounter ? 1 : 0);
            visits[body] = Math.Max(visit, 1);
            numbered.Add((c, encounter, body, Math.Max(visit, 1)));
        }
        foreach (var (c, encounter, body, visit) in numbered)
        {
            int into = line.StretchAt(c.Time);
            if (into == 0)
            {
                continue;
            }
            Astronomical from = system.Bodies[c.From], to = system.Bodies[c.To];
            // As KSA names them ("Luna Encounter", "Luna Escape"), numbered when a sphere is visited twice or more.
            string number = visits[body] > 1 ? $" {visit}" : "";
            string name = encounter ? $"{to.Id} Encounter{number}" : $"{from.Id} Escape{number}";
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
                                   $"{to.Id} at the encounter{number}", to.Id,
                                   detail: $"at the encounter{number}", s: into - 1, listed: false));
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
                                   a.Time, a.State.Q.ToDouble3(), $"{name} {Format.Altitude(a.Distance - radius)}", name,
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
                                       $"{body.Id} {Format.Altitude(a.Distance - body.MeanRadius)}", body.Id,
                                       a.Distance - body.MeanRadius));
                }
            }
        }
        if (r.Target is { } plane)
        {
            // As KSA's: "aN"/"dN", with the inclination to the target's orbit; only the first of each listed.
            bool listedAn = false, listedDn = false;
            foreach (var (time, q, ascending, inclination) in PlaneCrossings(ephemeris, line, trajectory, plane))
            {
                string name = ascending ? "aN" : "dN";
                string detail = $"{inclination:F2}° to {plane.Name}'s orbit";
                ref bool listed = ref ascending ? ref listedAn : ref listedDn;
                markers.Add(Marker(ascending ? Prediction.MarkerKind.AscendingNode
                                             : Prediction.MarkerKind.DescendingNode,
                                   time, q, $"{name}: {detail}", name, detail: detail, listed: !listed));
                listed = true;
            }
        }
        markers.Sort((x, y) => x.Time.CompareTo(y.Time));
        markersTiming.Dispose();
        (double, double3)[][] danger;
        using (PredictionTiming.Stage("danger ground track"))
        {
            danger = Danger(frame, trajectory.Times, relative);
        }

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
            AwaitingPlanets = trajectory.AwaitingPlanets,
            Steps = trajectory.Steps,
            Impact = impact,
            Markers = markers,
            ComputeMs = sw.Elapsed.TotalMilliseconds,
            Note = note,
            ParentApsides = new ParentApsides(r.Parent, Apsides(0), Apsides(1)),
            Danger = danger,
            StateTimes = trajectory.Times.ToArray(),
            States = trajectory.States.ToArray(),
        };

        (double, double)[] Apsides(int apoapsis) => trajectory.Apsides
            .Where(a => a.Body == r.ParentIndex && a.IsApoapsis == apoapsis && a.Time <= endTime)
            .OrderBy(a => a.Time)
            .Select(a => (a.Time, a.Distance - r.Parent.MeanRadius))
            .ToArray();
    }

    // Runs inside the centre's near-surface radius, KSA's danger zone, sampled this often (s).
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
                double3 q = Hermite.Between(times, relative, i, t).Q.ToDouble3();
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

    private readonly record struct Crossing(double Time, QP State, int From, int To);

    // Split into stretches at sphere crossings, each in its frame (see Stretch), densified, with each sample's index among
    // the points. A path continuing another partway (a node's track) gives `start`, the other's sphere there (SphereAt).
    private static (Line Line, int[] Index, List<Crossing> Crossings) Draw(
        PrincipiaSystem system, FrameSpec plot, IReadOnlyList<double> times, IReadOnlyList<QP> states, int n,
        bool ghosts = false, (int Body, bool Encounter)? start = null)
    {
        Ephemeris ephemeris = system.Ephemeris;
        Spheres spheres = system.Spheres;
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
        // The stretch being built, and which samples are the trajectory's (-1: a crossing).
        var pieceTimes = new List<double>();
        var pieceStates = new List<QP>();
        var pieceSamples = new List<int>();
        int pieceBody = n > 0 ? body[0] : plot.CentreIndex;
        // Began with an encounter (into a moon's sphere); for the first, as the path it continues.
        bool pieceEncounter = start is { } s && s.Body == pieceBody && s.Encounter;

        void Add(double t, QP state, int sample)
        {
            pieceTimes.Add(t);
            pieceStates.Add(state);
            pieceSamples.Add(sample);
        }
        void Close(bool closed)
        {
            // About its body only for an encounter, as KSA draws it; anything else in the plotting frame.
            FrameSpec frame = pieceEncounter && pieceBody != plot.CentreIndex
                ? new FrameSpec(PlotFrame.BodyCentred, system.Bodies[pieceBody], pieceBody, null, -1)
                : plot;
            int m = pieceTimes.Count;
            var coordinates = new double3[m];
            var velocities = new double3[m];
            for (int k = 0; k < m; k++)
            {
                QP centre = ephemeris.State(frame.CentreIndex, pieceTimes[k]);
                (coordinates[k], velocities[k]) = frame.Coordinates(ephemeris, pieceTimes[k], pieceStates[k], centre);
            }
            int budget = Math.Max(1000, (int)((long)Budget * m / Math.Max(n, 1)));
            (double[] dt, double3[] dp, int[] di) = Densify(pieceTimes, coordinates, velocities, m, budget);
            double[]? ghostTimes = null;
            double3[]? ghost = null;
            if (ghosts && frame != plot && m > 1)
            {
                for (int k = 0; k < m; k++)
                {
                    QP centre = ephemeris.State(plot.CentreIndex, pieceTimes[k]);
                    (coordinates[k], velocities[k]) =
                        plot.Coordinates(ephemeris, pieceTimes[k], pieceStates[k], centre);
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
                        QP q = Hermite.Between(times, states, i, mid);
                        if (spheres.Of(ephemeris, body[i - 1], q.Q.ToDouble3(), mid) == body[i - 1])
                        {
                            lo = mid;
                        }
                        else
                        {
                            hi = mid;
                        }
                    }
                    at = Hermite.Between(times, states, i, hi);
                }
                crossings.Add(new Crossing(hi, at, body[i - 1], body[i]));
                Add(hi, at, -1);
                Close(closed: true);
                pieceBody = body[i];
                pieceEncounter = spheres.IsChild(body[i], body[i - 1]);
                Add(hi, at, -1);
            }
            Add(times[i], states[i], i);
        }
        Close(closed: false);
        return (new Line(pointTimes.ToArray(), points.ToArray(), stretches.ToArray()), index, crossings);
    }

    // As Draw finds them along the whole path.
    private static (int Body, bool Encounter) SphereAt(PrincipiaSystem system, FrameSpec plot,
                                                       IReadOnlyList<double> times, IReadOnlyList<QP> states, int k)
    {
        Ephemeris ephemeris = system.Ephemeris;
        Spheres spheres = system.Spheres;
        int body = plot.CentreIndex;
        bool encounter = false;
        for (int i = 0; i <= k && i < times.Count; i++)
        {
            int b = spheres.Of(ephemeris, body, states[i].Q.ToDouble3(), times[i]);
            if (i > 0 && b != body)
            {
                encounter = spheres.IsChild(b, body);
            }
            body = b;
        }
        return (body, encounter);
    }

    // Coarse samples would draw polygonal: intervals are split along the Hermite curve until the path turns at most MaxTurn
    // per piece, within Budget points.
    private const double MaxTurn = 3 * Math.PI / 180;
    private const int Budget = 60000;

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
                double norms = velocities[i].Length() * velocities[i + 1].Length();
                double a = norms > 0
                    ? Math.Acos(Math.Clamp(double3.Dot(velocities[i], velocities[i + 1]) / norms, -1, 1))
                    : double.NaN;
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
                double u = (double)j / pieces[i];
                denseTimes[k] = t0 + u * dt;
                densePositions[k++] = Hermite.Position(positions[i], velocities[i], positions[i + 1],
                                                       velocities[i + 1], dt, u);
            }
            index[i + 1] = k;
            denseTimes[k] = times[i + 1];
            densePositions[k++] = positions[i + 1];
        }
        return (denseTimes, densePositions, index);
    }

    // ── Where a line meets the ground ───────────────────────────────────────────

    // Principia integrates through bodies with samples minutes apart, so each interval dipping below the highest terrain is
    // walked finely along its Hermite curve (within 10 m of the path) to the surface KSA draws.
    private static class Impact
    {
        private const int Points = 4000;       // Per stretch between two samples.
        private const int MaxStretches = 64;   // An orbit skimming the terrain dips every revolution.

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
                         : Hermite.Between(trajectory.Times, trajectory.States, i, t);
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

    // As KSA draws it: the mean radius plus the height map, never below sea level where there is an ocean.
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
