using System.Diagnostics;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// Predicts the controlled vessel's n-body trajectory on a background thread:
// the prediction (coasting), the flight plan (through KSA's planned burns) and
// the maneuver nodes' approach tracks. The main thread posts a snapshot every
// frame; the thread always works on the latest.
internal static partial class Predictor
{
    // A planned burn as the flight computer will fly it: from Ignition, along
    // a fixed inertial direction, until DeltaV is reached. A burn not yet
    // targeted also carries its Δv in KSA's prograde/normal/radial frame about
    // ParentIndex, to be aimed from Principia's state (BurnDirection).
    private sealed record PlannedBurn(double Impulse, double? Ignition, double DeltaV, double3 Direction,
                                      double3? DeltaVVlf = null, int ParentIndex = -1);

    // Engine phases (staging), as KSA's finite-burn planning has them.
    private sealed record EnginePhase(double Thrust, double MassFlow, double StartMass, double MaxSeconds);

    private sealed record Request(Vehicle Vehicle,
                                  QP State, double Time, FrameSpec Frame,
                                  int[] ApsisBodies, Astronomical[] Candidates, double CoastHorizon,
                                  PlannedBurn[] Burns, EnginePhase[] Phases, double Mass,
                                  double PlanHorizon, NodeBurn[] Nodes, bool ShowPlan,
                                  int ParentIndex, Astronomical Parent);

    // A maneuver node needing an approach track, reaching Window either side.
    private sealed record NodeBurn(Burn Burn, double Time, double Window);

    // Points and apsides per integrated segment.
    private const int SegmentCapacity = 10000;
    private const int ApsisCapacity = 400;
    private static readonly long MinInterval = Stopwatch.Frequency / 120;

    private static Thread? _thread;
    private static readonly AutoResetEvent Wake = new(false);
    private static volatile bool _stop;
    private static Request? _request;
    private static long _lastRequest;

    public static volatile Prediction? Latest;      // The prediction.
    public static volatile Prediction? LatestPlan;  // The flight plan.
    public static volatile NodeTracks? LatestNodes; // The maneuver nodes' approaches.
    public static volatile string? LastError;

    public static void Start()
    {
        if (_thread != null)
        {
            return;
        }
        _stop = false;
        _thread = new Thread(Run) { IsBackground = true, Name = "Principia prediction" };
        _thread.Start();
    }

    public static void Stop()
    {
        _stop = true;
        Wake.Set();
        _thread?.Join();
        _thread = null;
        Latest = LatestPlan = null;
        LatestNodes = null;
    }

    // Shown in the window; logged when it changes, in full the first time.
    private static void Failed(string where, Exception e)
    {
        if (e.Message != LastError)
        {
            Log.Warn($"[predict] prediction {where} failed: {e.Message}");
            Log.WarnOnce($"[predict] prediction {where}: {e.Message}", e);
        }
        LastError = e.Message;
    }

    private static void Clear()
    {
        Latest = LatestPlan = null;
        LatestNodes = null;
    }

    // Main thread, once per frame. Never throws.
    public static void Tick()
    {
        try
        {
            Snapshot();
        }
        catch (Exception e)
        {
            Failed("snapshot", e);
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
            ResolveFrame(system, parent) is not { } frame)
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
            Settings.ShowPrediction ? Settings.PredictionDays * 86400.0 : 0,
            Burns(vehicle, t), Phases(fc), fc.TotalMassPropsBody.Mass,
            Settings.FlightPlanDays * 86400.0, Nodes(vehicle, t), Settings.ShowFlightPlan,
            parentIndex, parent));
        Wake.Set();
    }

    // The plotting frame from the settings; unset bodies follow the vessel.
    private static FrameSpec? ResolveFrame(PrincipiaSystem system, Astronomical vesselParent)
    {
        Astronomical centre = Find(system, Settings.CentreBody) ?? vesselParent;
        if (system.IndexOf(centre) is not int centreIndex)
        {
            return null;
        }
        if (Settings.Frame != PlotFrame.BarycentricRotating)
        {
            return new FrameSpec(Settings.Frame, centre, centreIndex, null, -1);
        }
        // Secondary: as chosen, else the centre's heaviest moon, else its parent.
        Astronomical? secondary = Find(system, Settings.SecondaryBody);
        if (secondary == null && centre is IParentBody centreBody)
        {
            secondary = centreBody.Children.OfType<Astronomical>()
                                  .Where(c => system.IndexOf(c) != null)
                                  .MaxBy(c => ((IParentBody)c).Mass);
        }
        if (secondary == null && centre is IOrbiter { Parent: Astronomical up })
        {
            secondary = up;
        }
        if (secondary == null || secondary == centre || system.IndexOf(secondary) is not int secondaryIndex)
        {
            return new FrameSpec(PlotFrame.BodyCentred, centre, centreIndex, null, -1);
        }
        return new FrameSpec(PlotFrame.BarycentricRotating, centre, centreIndex, secondary, secondaryIndex);
    }

    public static Astronomical? Find(PrincipiaSystem system, string? id) =>
        id == null ? null : system.Bodies.FirstOrDefault(b => b.Id == id);

    // When the vessel makes a burn. A departure planned on the parent's orbit
    // (the transfer planner's, Burn.ParentDepartureBurn) is a vessel burn at
    // ParentDepartureTime, whose Δv KSA computes (CalculateTransferBurn); its
    // Time and Patch are the parent's.
    public static UniverseTime VesselTime(Burn burn) => burn.ParentDepartureBurn ? burn.ParentDepartureTime : burn.Time;

    // KSA's burns still ahead or being flown, with the direction the flight
    // computer will hold: its loaded target's, else from the burn's patch,
    // which has the body and state of Principia's path (BurnAnchor).
    private static PlannedBurn[] Burns(Vehicle vehicle, double now)
    {
        var burns = new List<PlannedBurn>();
        FlightComputer fc = vehicle.FlightComputer;
        BurnTarget? target = fc.Burn;
        BurnPlan plan = vehicle.BurnPlan;
        for (int i = 0; i < plan.BurnCount; i++)
        {
            if (!plan.TryGetBurn(i, out Burn? burn) || burn == null || !burn.HasDeltaV || burn.IsInterstellar)
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

    private static void AddBurn(List<PlannedBurn> burns, Vehicle vehicle, Burn burn, BurnTarget? target, double now)
    {
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
    }

    // How far a node's approach track reaches either side of it, and so a drag
    // step: a twentieth of its orbit (10 min to 12 h), an hour if unbound. The
    // track follows the node as it moves.
    public static double NodeWindow(Orbit orbit)
    {
        double period = orbit.Period;
        return orbit.IsBound() && double.IsFinite(period) && period > 0
            ? Math.Clamp(period / 20, 600, 12 * 3600.0)
            : 3600;
    }

    // Every node still ahead, with or without Δv.
    private static NodeBurn[] Nodes(Vehicle vehicle, double now)
    {
        var nodes = new List<NodeBurn>();
        BurnPlan plan = vehicle.BurnPlan;
        for (int i = 0; i < plan.BurnCount; i++)
        {
            if (!plan.TryGetBurn(i, out Burn? burn) || burn == null || burn.IsInterstellar ||
                burn.ParentDepartureBurn)
            {
                continue;  // A departure's node is on its parent's orbit: KSA's.
            }
            double time = GameTime.ToSecondsSinceJ2000(burn.Time);
            if (!(time > now))
            {
                continue;
            }
            nodes.Add(new NodeBurn(burn, time, NodeWindow(burn.Patch.Orbit)));
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

    private static void Run()
    {
        var buffers = new Buffers();
        Request? done = null;
        while (!_stop)
        {
            Wake.WaitOne(500);
            Request? request = Volatile.Read(ref _request);
            if (request == null || ReferenceEquals(request, done))
            {
                continue;
            }
            done = request;
            PrincipiaSystem? system = PrincipiaSystem.Current;
            if (system == null || !system.Enter())
            {
                continue;
            }
            try
            {
                Latest = request.CoastHorizon > 0 ? ComputeCoast(system, request, buffers) : null;
                NodeTracks? nodes = null;
                Prediction? plan = request.Burns.Length > 0 || request.Nodes.Length > 0
                    ? ComputePlan(system, request, buffers, out nodes)
                    : null;
                LatestPlan = request.ShowPlan ? plan : null;
                LatestNodes = nodes;
                LastError = null;
            }
            catch (Exception e)
            {
                Failed("computation", e);
                Latest = LatestPlan = null;
                LatestNodes = null;
            }
            finally
            {
                system.Exit();
            }
        }
    }

    private sealed class Buffers
    {
        public readonly double[] Times = new double[SegmentCapacity];
        public readonly QP[] States = new QP[SegmentCapacity];
        public readonly Apsis[] Apsides = new Apsis[ApsisCapacity];
    }

    // A power of two seconds fitting `span` in a segment, so that successive
    // predictions share their points and lines don't shift.
    private static double SampleInterval(double span) =>
        Math.Pow(2, Math.Ceiling(Math.Log2(Math.Max(span / (SegmentCapacity - 2), 1))));

    // Accumulates segments into one trajectory.
    private sealed class Trajectory
    {
        public readonly List<double> Times = new();
        public readonly List<QP> States = new();
        public readonly List<(int, int)> BurnArcs = new();
        public readonly List<Apsis> Apsides = new();
        public readonly List<(double Time, QP State, string Label)> Burns = new();
        public PredictionEnd End = PredictionEnd.ReachedEnd;
        public QP Last;
        public double LastTime;
        public bool CollectApsides = true;

        public Trajectory(QP state, double t) => (Last, LastTime) = (state, t);

        // Integrates on to `t`; false if it stopped early.
        public bool Extend(Ephemeris ephemeris, double t, int maxSteps, double sampleInterval, int[] apsisBodies,
                           Buffers b, double3 direction = default, double thrust = 0, double mass = 0,
                           double massFlow = 0)
        {
            if (!(t > LastTime))
            {
                return true;
            }
            (int count, End, int apsides) = ephemeris.Predict(
                Last, LastTime, t, maxSteps, direction, thrust, mass, massFlow, sampleInterval, b.Times, b.States,
                CollectApsides ? apsisBodies : [], b.Apsides);
            for (int i = Times.Count == 0 ? 0 : 1; i < count; i++)
            {
                Times.Add(b.Times[i]);
                States.Add(b.States[i]);
            }
            for (int i = 0; i < apsides; i++)
            {
                Apsides.Add(b.Apsides[i]);
            }
            Last = b.States[count - 1];
            LastTime = b.Times[count - 1];
            return End == PredictionEnd.ReachedEnd;
        }
    }

    private static Prediction ComputeCoast(PrincipiaSystem system, Request r, Buffers b)
    {
        var sw = Stopwatch.StartNew();
        var trajectory = new Trajectory(r.State, r.Time);
        trajectory.Extend(system.Ephemeris, r.Time + r.CoastHorizon, Settings.MaxSteps,
                          SampleInterval(r.CoastHorizon), r.ApsisBodies, b);
        return Finish(system, r, trajectory, isFlightPlan: false, firstIgnition: 0, note: null, sw, b);
    }
}
