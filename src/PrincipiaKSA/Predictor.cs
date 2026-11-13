using System.Diagnostics;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// Predicts the controlled vessel's n-body trajectory in the background: the
// prediction (coasting) on one thread, the flight plan (through KSA's planned
// burns) on another, each maneuver node's approach track as a thread-pool
// task once the plan is there. The main thread posts a snapshot every frame;
// each thread always works on the latest. Principia's ephemeris is made to be
// flowed from several threads at once.
internal static class Predictor
{
    // ── The results the threads publish, and starting and stopping them ─────────

    public static volatile Prediction? Latest;      // The prediction.
    public static volatile Prediction? LatestPlan;  // The flight plan.
    public static volatile NodeTracks? LatestNodes; // The maneuver nodes' approaches.

    // The lines of `vehicle` as shown: null while hidden, not computed yet, or
    // for another vessel. What everything else reads.
    public static Prediction? PredictionFor(Vehicle? vehicle) =>
        Settings.ShowPrediction && Latest is { } p && p.Vehicle == vehicle ? p : null;

    public static Prediction? PlanFor(Vehicle? vehicle) =>
        Settings.ShowFlightPlan && LatestPlan is { } p && p.Vehicle == vehicle ? p : null;

    public static NodeTracks? NodesFor(Vehicle? vehicle) => LatestNodes is { } n && n.Vehicle == vehicle ? n : null;

    // The last failure of each part, until it next succeeds.
    private static string? _snapshotError;
    public static string? LastError => _snapshotError ?? CoastWorker.Error ?? PlanWorker.Error;

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
                                  int ParentIndex, Astronomical Parent)
    {
        // When the vessel's state was taken (Stopwatch ticks).
        public long Requested { get; } = Stopwatch.GetTimestamp();
    }

    // A planned burn as the flight computer will fly it: from Ignition, along
    // a fixed inertial direction, until DeltaV is reached. A burn not yet
    // targeted also carries its Δv in KSA's prograde/normal/radial frame about
    // ParentIndex, to be aimed from Principia's state (BurnDirection).
    private sealed record PlannedBurn(double Impulse, double? Ignition, double DeltaV, double3 Direction,
                                      double3? DeltaVVlf = null, int ParentIndex = -1);

    // Engine phases (staging), as KSA's finite-burn planning has them.
    private sealed record EnginePhase(double Thrust, double MassFlow, double StartMass, double MaxSeconds);

    // A maneuver node needing an approach track, reaching Window either side.
    private sealed record NodeBurn(Burn Burn, double Time, double Window);

    private static readonly long MinInterval = Stopwatch.Frequency / 120;

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
        _lastRequest = now;
        StateVectors sv = orbit.StateVectors;
        double t = GameTime.ToSecondsSinceJ2000(sv.StateTime);
        QP p = system.Ephemeris.State(parentIndex, t);
        doubleQuat cci2Cce = orbit.Parent.GetCci2Cce();
        var fromKsa = new QP(p.Q.ToDouble3() + sv.PositionCci.Transform(cci2Cce),
                             p.P.ToDouble3() + sv.VelocityCci.Transform(cci2Cce));
        // The exact n-body state when coasting: KSA's conic is only within the
        // rebuild tolerances of it, enough to move a circular orbit's apsides.
        QP state = fromKsa;
        if (VesselDriver.TryGetTrack(vehicle, sv.StateTime.Nanoseconds, out QP track, out UniverseTime trackTime))
        {
            state = track;
            t = GameTime.ToSecondsSinceJ2000(trackTime);
        }
        FlightComputer fc = vehicle.FlightComputer;
        Astronomical[] candidates = Candidates(system, frame.Centre);
        // About the frame's centre for the markers, the vessel's parent for
        // KSA's Ap and Pe gauges, the others for closest approaches.
        int[] apsisBodies = new[] { frame.CentreIndex, parentIndex }
            .Concat(candidates.Select(c => system.IndexOf(c) ?? -1).Where(i => i >= 0)).Distinct().ToArray();
        Volatile.Write(ref _request, new Request(
            vehicle, state, t, frame, apsisBodies, candidates,
            Settings.ShowPrediction ? Settings.PredictionSteps : 0, Settings.PredictionTolerance,
            Burns(vehicle, t), Phases(fc), fc.TotalMassPropsBody.Mass,
            Settings.PlanSteps, Settings.PlanTolerance, Nodes(vehicle, t), Settings.ShowFlightPlan,
            parentIndex, parent));
        CoastWorker.Wake();
        PlanWorker.Wake();
    }

    // KSA's burns still ahead or being flown, with the direction the flight
    // computer will hold: its loaded target's, else from the burn's patch,
    // which has the body and state of Principia's path (BurnAnchor).
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
                // Without it, rather than no lines at all; each kind of
                // failure logged once.
                if (BurnFailures.Add($"{e.GetType().Name}: {e.Message}"))
                {
                    Log.Warn($"[predict] burn {Describe(burn, now)} left out of the plan: {e}");
                }
            }
        }
        burns.Sort((a, b) => a.Impulse.CompareTo(b.Impulse));
        return burns.ToArray();
    }

    private static readonly HashSet<string> BurnFailures = new();  // Main thread.

    // A burn for the log: its time, Δv and patch.
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

    // Not optimized, so that a failure's stack trace names the KSA call that
    // failed instead of folding it into this method (a rare "divide by zero",
    // seen with a hyperbolic patch and a Δv near zero).
    [System.Runtime.CompilerServices.MethodImpl(System.Runtime.CompilerServices.MethodImplOptions.NoInlining |
                                                System.Runtime.CompilerServices.MethodImplOptions.NoOptimization)]
    private static void AddBurn(List<PlannedBurn> burns, Vehicle vehicle, Burn burn, BurnTarget? target, double now)
    {
        if (burn.ParentDepartureBurn)
        {
            // As KSA previews it, until it is made a vessel burn: on the
            // vessel's own path at ParentDepartureTime.
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
            double ignition = GameTime.ToSecondsSinceJ2000(target.IgnitionTime);
            burns.Add(new PlannedBurn(impulse, double.IsFinite(ignition) && ignition < impulse ? ignition : null,
                                      dv.Length(), new double3(dv.X, dv.Y, dv.Z).Transform(cci2Cce).Normalized()));
            return;
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

    // How far a node's approach track reaches either side of it, and so how
    // far one drag step may move it (0.9 of it): half its orbit, so that the
    // node can go anywhere on the orbit at once; unbound, half the time to it.
    // From 10 min to 60 days: the track is integrated on every update (and
    // only as far as the planets are, NodeTrackOf), and a drag also stops at
    // the node's sphere of influence. The track follows the node as it moves.
    public static double NodeWindow(Orbit orbit, double ahead)
    {
        double period = orbit.Period;
        double half = orbit.IsBound() && double.IsFinite(period) && period > 0 ? period / 2 : ahead / 2;
        return double.IsFinite(half) ? Math.Clamp(half, 600, 60 * 86400.0) : 3600;
    }

    // Every node still ahead, with or without Δv.
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

    // Bodies whose closest approach is worth showing: the centre body's moons,
    // and if it is a moon, its planet and sibling moons.
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

    // The prediction's and the flight plan's threads: permanent, as they run
    // all the time (a thread costs ~50 µs to start, a pool task ~0.5 µs).
    private static readonly Worker CoastWorker = new("Principia prediction", "prediction", FrameProfiler.Prediction,
                                                     ComputeAndPublishCoast, () => Latest = null);
    private static readonly Worker PlanWorker = new("Principia flight plan", "flight plan", FrameProfiler.FlightPlan,
                                                    ComputeAndPublishPlan, () => (LatestPlan, LatestNodes) = (null, null));
    private static volatile bool _stop;
    private static Request? _request;
    private static long _lastRequest;

    // A background thread computing from the latest request whenever there is
    // a new one: `compute` publishes its result and returns it for the log.
    private sealed class Worker(string threadName, string what, FrameProfiler.Section profiled,
                                Func<PrincipiaSystem, Request, Prediction?> compute, Action clear)
    {
        private readonly AutoResetEvent _wake = new(false);
        private Thread? _thread;

        // The last failure, until it next succeeds.
        public string? Error;

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

    private static Prediction? ComputeAndPublishCoast(PrincipiaSystem system, Request r) =>
        Latest = r.CoastSteps > 0 ? ComputeCoast(system, r) : null;

    // The flight plan, and the nodes' tracks with it.
    private static Prediction? ComputeAndPublishPlan(PrincipiaSystem system, Request r)
    {
        NodeTracks? nodes = null;
        Prediction? plan = r.Burns.Length > 0 || r.Nodes.Length > 0 ? ComputePlan(system, r, out nodes) : null;
        LatestPlan = r.ShowPlan ? plan : null;
        LatestNodes = nodes;
        return plan;
    }

    // ── Integrating: the prediction, the flight plan and the nodes' tracks ──────

    // Burns are integrated as Principia's flight plan integrates them
    // (DefaultBurnParameters), whatever the steps and tolerance chosen for
    // the coasts: at most this many steps per engine phase, at 1 m.
    private const long BurnSteps = 1000;
    private const double BurnTolerance = 1;

    // How far ahead any line may go, steps left or not: the ephemeris it
    // needs is prolonged that far, which costs memory (twenty years).
    public const double MaxSpanYears = 20;
    private const double MaxSpan = MaxSpanYears * 365.25 * 86400;

    // How far past the integrated planets a line may go in one computation,
    // integrating them itself (~6 ms per day for KSA's 46 bodies), and how far
    // ahead of where it stopped it asks the background thread to integrate.
    private const double AheadOfPlanets = 86400;
    private const double PlanetRequestSeconds = 30 * 86400;

    // How far toward `wanted` a line may be integrated now: no further than
    // the planets are integrated and a day (integrating them takes ~6 ms per
    // day for KSA's 46 bodies, far longer than the vessel).
    private static double PlanetsReadyUntil(Ephemeris ephemeris, double wanted) =>
        Math.Min(wanted, ephemeris.TimeRange.TMax + AheadOfPlanets);

    // A line stopped at `reached` for the planets, short of `wanted`: the
    // background thread is asked to go on, so it grows over the next
    // computations instead of waiting for the planets now. Only then: a line
    // that ends on its own (its steps run out) asks for nothing, or every
    // line wanting up to MaxSpan would integrate the planets twenty years.
    private static void WantPlanets(double reached, double wanted) =>
        PrincipiaSystem.Current?.Prolonger.Request(Math.Min(wanted, reached + PlanetRequestSeconds));

    // Accumulates segments into one trajectory.
    private sealed class Trajectory
    {
        public readonly List<double> Times = new();
        public readonly List<QP> States = new();
        public readonly List<(int, int)> BurnArcs = new();
        public readonly List<Apsis> Apsides = new();
        public readonly List<(double Time, QP State, string Label)> Burns = new();
        public PredictionEnd End = PredictionEnd.ReachedEnd;
        // Stopped, steps left, where the planets are integrated so far: the
        // next computation goes further (AheadOfPlanets).
        public bool AwaitingPlanets;
        public long Steps;  // Integrator steps taken, in all.
        public QP Last;
        public double LastTime;
        public bool CollectApsides = true;

        public Trajectory(QP state, double t) => (Last, LastTime) = (state, t);

        // Integrates on, as Principia predicts: until `maxSteps` steps at
        // `tolerance` (m, m/s), time `t` (+∞: no time limit) or a body. End
        // says why it stopped; returns the steps taken.
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

    // The prediction: the vessel coasting until its steps run out.
    private static Prediction ComputeCoast(PrincipiaSystem system, Request r)
    {
        var sw = Stopwatch.StartNew();
        var trajectory = new Trajectory(r.State, r.Time);
        trajectory.Extend(system.Ephemeris, r.Time + MaxSpan, r.CoastSteps, r.CoastTolerance, r.ApsisBodies);
        return Finish(system, r, trajectory, isFlightPlan: false, firstIgnition: 0, note: null, sw);
    }

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
        // Each node's track at once, a thread-pool task each: the plan they
        // start from is done.
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
        // Where the track starts partway along the plan, the sphere the plan
        // is in there: an encounter stretch is drawn about its body, and the
        // track's must be too, or the node would be drawn elsewhere (about the
        // Sun at Saturn's place then, not at Saturn).
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

    // Where a sequence of burns is in the engine phases: the phase, the mass,
    // and the seconds already burned in that phase. A value: copy it to look
    // ahead (Duration).
    private struct Engines(EnginePhase[] phases, double mass)
    {
        public int Phase;
        public double Mass = mass;
        public double Used;  // Seconds burned in the current phase.

        public readonly bool Done => Phase >= phases.Length;

        // The next stretch of thrust toward `deltaV`: up to the end of the
        // current phase, or less where the Δv is reached first.
        public readonly (EnginePhase Engine, double Seconds, bool PhaseEnds) Next(double deltaV)
        {
            EnginePhase e = phases[Phase];
            double left = e.MaxSeconds - Used;
            double seconds = BurnSeconds(e.Thrust, e.MassFlow, Mass, deltaV);
            return seconds >= left ? (e, left, true) : (e, seconds, false);
        }

        // Burns `seconds` of `e`, staging to the next phase where it ends;
        // returns the Δv gained.
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

        // How long reaching `deltaV` takes from here, rocket equation, phase by phase.
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

    // The rocket equation: the seconds to gain `deltaV`, and the Δv gained in `seconds`.
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

    // Frame coordinates, impact, apsides and closest approaches.
    private static Prediction Finish(PrincipiaSystem system, Request r, Trajectory trajectory, bool isFlightPlan,
                                     int firstIgnition, string? note, Stopwatch sw)
    {
        Ephemeris ephemeris = system.Ephemeris;
        FrameSpec frame = r.Frame;
        double radius = frame.Centre.MeanRadius;
        // End the line where it meets the centre body's surface.
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
        // A point in the frame of stretch s (by default the one at time t).
        (int Stretch, double3 Position) Place(double t, double3 q, int s = -1)
        {
            s = s < 0 ? line.StretchAt(t) : s;
            FrameSpec f = line.Stretches[s].Frame;
            QP centre = ephemeris.State(f.CentreIndex, t);
            return (s, f.Coordinates(ephemeris, t, new QP(q, double3.Zero), centre).Position);
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
        Spheres spheres = system.Spheres;
        // Each visit to a sphere is numbered: an encounter starts the next,
        // the escape ending it shares its number. Spheres nest, so leaving a
        // moon's back into its planet's is no new visit of the planet; where
        // the line starts, that body's sphere and those around it are visit 1.
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
            // As KSA names them: into a moon's sphere, "Luna Encounter"; out
            // of it, "Luna Escape" (and "Earth Escape" out to the Sun); with
            // the visit's number where the line visits that sphere twice or more.
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

    // Where a path goes from one sphere of influence into another.
    private readonly record struct Crossing(double Time, QP State, int From, int To);

    // A path's points to draw: split where it changes sphere of influence, at
    // the crossing found between the samples, each stretch in its frame
    // (Stretch: about the body met for an encounter, as KSA draws its
    // encounter patches, else the plotting frame), densified; with `ghosts`,
    // an encounter also dimmer in the plotting frame. Also each sample's
    // index among the points. A path that continues another partway (a node's
    // track) gives `start`, the sphere the other is in there and whether it
    // entered it by an encounter (SphereAt), so that its first stretch is
    // drawn in the same frame as there.
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
        // The stretch being built: its samples, and which of them are the
        // trajectory's (-1: a crossing).
        var pieceTimes = new List<double>();
        var pieceStates = new List<QP>();
        var pieceSamples = new List<int>();
        int pieceBody = n > 0 ? body[0] : plot.CentreIndex;
        // Whether the stretch being built began with an encounter: from a
        // body's sphere into one of its moons' (or planets'). For the first,
        // as the path it continues has it.
        bool pieceEncounter = start is { } s && s.Body == pieceBody && s.Encounter;

        void Add(double t, QP state, int sample)
        {
            pieceTimes.Add(t);
            pieceStates.Add(state);
            pieceSamples.Add(sample);
        }
        void Close(bool closed)
        {
            // About its body only for an encounter, as KSA draws the
            // encounter patch about the body met; anything else (where the
            // vessel is now, back out of an encounter, an escape) in the
            // plotting frame chosen.
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
            // The same stretch in the plotting frame, to draw dimmer.
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

    // The sphere a path is in at sample `k`, and whether it entered it by an
    // encounter, as Draw finds them drawing the whole path.
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

    // Long predictions are sampled coarsely, which would draw polygonal, so
    // each interval is split along the Hermite curve of its end points until
    // the path turns at most MaxTurn per piece (within Budget points). Returns
    // the dense times and points, and each sample's index among them.
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

    private static double Angle(double3 a, double3 b)
    {
        double norms = a.Length() * b.Length();
        return norms > 0 ? Math.Acos(Math.Clamp(double3.Dot(a, b) / norms, -1, 1)) : double.NaN;
    }

    // ── Where a line meets the ground ───────────────────────────────────────────

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
