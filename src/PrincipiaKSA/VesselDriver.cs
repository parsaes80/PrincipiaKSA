using System.Diagnostics;
using System.Linq.Expressions;
using System.Reflection;
using System.Runtime.CompilerServices;
using Brutal.Numerics;
using CommunityToolkit.HighPerformance.Buffers;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// A coasting vessel's true state (its Track) is integrated by Principia each step, and its conic rebuilt where it is
// off, so KSA's rails code runs unchanged while effects too small for one step still accumulate. Under full physics
// the rest of the n-body field is added to KSA's point-mass gravity (Correction).
internal static class VesselDriver
{
    private sealed class Track
    {
        public QP State;        // Absolute, barycentric, at Time.
        public Int128 TimeNs = Int128.MinValue;
        public long PointsDrawn;  // Stopwatch ticks.
    }

    private static readonly ConditionalWeakTable<Vehicle, Track> Tracks = new();

    public static long Flows, FullPhysicsFlows, Rebuilds, Resyncs, Corrections;

    // Worker threads, one per vehicle at a time.
    public static void BeforeFreefall(VehicleUpdateState vehicleState, UniverseTime nextTime)
    {
        PrincipiaSystem? system = PrincipiaSystem.Current;
        if (system == null || !system.Driving || !system.Enter())
        {
            return;
        }
        Vehicle vehicle = vehicleState.ReadOnlyVehicle;
        try
        {
            Orbit orbit = vehicleState.CurrentOrbit;
            IParentBody parent = orbit.Parent;
            if (parent is not Astronomical parentAstronomical || system.IndexOf(parentAstronomical) is not int p)
            {
                return;  // Orbiting something Principia doesn't integrate.
            }
            StateVectors ksa = vehicleState.CurrentStateVectors;
            UniverseTime t0 = ksa.StateTime;
            if (nextTime <= t0)
            {
                return;
            }
            double s0 = GameTime.ToSecondsSinceJ2000(t0), s1 = GameTime.ToSecondsSinceJ2000(nextTime);
            Ephemeris ephemeris = system.Ephemeris;
            doubleQuat cci2Cce = parent.GetCci2Cce();

            // Where KSA has the vessel now, as an absolute state.
            QP parent0 = ephemeris.State(p, s0);
            var fromKsa = new QP(parent0.Q.ToDouble3() + ksa.PositionCci.Transform(cci2Cce),
                                 parent0.P.ToDouble3() + ksa.VelocityCci.Transform(cci2Cce));
            Track track = Tracks.GetOrCreateValue(vehicle);
            // Small differences, as when KSA re-expresses the state about a new parent, aren't a move.
            if (track.TimeNs != t0.Nanoseconds || (track.State.Q.ToDouble3() - fromKsa.Q.ToDouble3()).Length() >
                                                  Math.Max(1000, 1e-4 * ksa.PositionCci.Length()))
            {
                // New, back from physics, loaded, or moved by something else.
                track.State = fromKsa;
                Interlocked.Increment(ref Resyncs);
            }

            // 1 mm per minute of step, up to 1 m: a long step (high warp) costs several times less, ~50 m a day off in
            // low orbit.
            QP next = ephemeris.FlowMassless(in track.State, s0, s1, Math.Clamp((s1 - s0) / 60 * 1e-3, 1e-3, 1));
            Interlocked.Increment(ref Flows);
            track.State = next;
            track.TimeNs = nextTime.Nanoseconds;

            // The same state relative to the parent, in its Cci.
            QP parent1 = ephemeris.State(p, s1);
            doubleQuat cce2Cci = cci2Cce.Inverse();
            double3 q = (next.Q.ToDouble3() - parent1.Q.ToDouble3()).Transform(cce2Cci);
            double3 v = (next.P.ToDouble3() - parent1.P.ToDouble3()).Transform(cce2Cci);
            // Keep the conic within a metre and a centimetre per second.
            StateVectors conic = orbit.GetStateVectorsAt(nextTime);
            double drift = (conic.PositionCci - q).Length();
            double velocityDrift = (conic.VelocityCci - v).Length();
            if (drift > Math.Max(1.0, 1e-7 * q.Length()) || velocityDrift > 0.01)
            {
                Orbit fresh = Orbit.CreateFromStateCci(parent, nextTime, q, v, vehicle.OrbitColor);
                SetOrbit(vehicleState, fresh, track);
                Interlocked.Increment(ref Rebuilds);
            }
        }
        catch (Exception e)
        {
            // This step follows KSA's conic; the next restarts from its state.
            Tracks.Remove(vehicle);
            Log.WarnOnce($"[vessels] n-body step of {vehicle.Id}", e);
        }
        finally
        {
            system.Exit();
        }
    }

    private static readonly long PointsInterval = Stopwatch.Frequency / 2;

    // SetCurrentOrbit makes KSA plan the vessel anew, as after a burn: its whole trajectory, encounters, impacts and
    // line points, at every correction. A plan of one final patch instead gets the orbit in a copy that keeps its
    // verification (KSA re-verifies it at its expiry as usual) and its line points, redrawn at most twice a second.
    private static void SetOrbit(VehicleUpdateState state, Orbit fresh, Track track)
    {
        FlightPlan plan = state.CurrentFlightPlan;
        if (state.IsControlled || !plan.IsComplete || plan.ImpactSearchUnresolved || !plan.HasCachedPoints ||
            plan.Patches is not [{ EndTransition: PatchTransition.Final } patch])
        {
            state.SetCurrentOrbit(fresh, state.ReadOnlyVehicle.Hash);
            return;
        }
        bool staged = state.UpdateData.NewFlightPlan != null;  // Then the plan and its orbits are this step's own.
        FlightPlan copy = staged ? plan : new FlightPlan(plan);
        Orbit old = patch.Orbit;
        PatchedConic first = copy.Patches[0];
        fresh.OrbitLineColor = old.OrbitLineColor;
        first.Orbit = fresh;
        long now = Stopwatch.GetTimestamp();
        if (old.CachedPoints.Length == 0 || now - track.PointsDrawn > PointsInterval)
        {
            fresh.UpdateCachedPoints(UpdateTaskUtils.GenerateSpacedPoints(first));
            track.PointsDrawn = now;
        }
        else
        {
            var points = MemoryOwner<OrbitPointCce>.Allocate(old.CachedPoints.Length);
            old.CachedPoints.CopyTo(points.Span);
            fresh.UpdateCachedPoints(points);
        }
        if (staged)
        {
            old.ReleaseCachedPoints();
        }
        state.UpdateData.NewStateVectors = null;
        state.UpdateData.NewFlightPlan = copy;
    }

    // Between jobs the track is a step ahead of KSA's state, so it is accepted up to an hour after `notBeforeNs`.
    public static bool TryGetTrack(Vehicle vehicle, Int128 notBeforeNs, out QP state, out UniverseTime time)
    {
        if (Tracks.TryGetValue(vehicle, out Track? track) && track.TimeNs >= notBeforeNs &&
            track.TimeNs - notBeforeNs <= (Int128)3600 * UniverseTime.NANOSECONDS_PER_SECOND)
        {
            state = track.State;
            time = UniverseTime.FromNanoseconds(track.TimeNs);
            return true;
        }
        state = default;
        time = default;
        return false;
    }

    // The n-body acceleration less the point-mass gravity KSA already applies, at `positionBub` (bubble axes).
    public static double3? Correction(IParentBody parent, bool rotatingFrame, double3 positionBub, UniverseTime time)
    {
        PrincipiaSystem? system = PrincipiaSystem.Current;
        if (system == null || !system.Driving || parent is not Astronomical a || system.IndexOf(a) is not int index ||
            !system.Enter())
        {
            return null;
        }
        try
        {
            double r2 = positionBub.LengthSquared();
            if (!(r2 > 1))
            {
                return null;
            }
            doubleQuat bub2Ecl = rotatingFrame ? parent.GetCcf2Cce(time) : parent.GetCci2Cce();
            XYZ relative = system.Ephemeris.RelativeAcceleration(
                index, new XYZ(positionBub.Transform(bub2Ecl)), GameTime.ToSecondsSinceJ2000(time));
            double3 nBody = relative.ToDouble3().Transform(bub2Ecl.Inverse());
            double3 pointMass = -parent.Mu / (r2 * Math.Sqrt(r2)) * positionBub;
            Interlocked.Increment(ref Corrections);
            return nBody - pointMass;
        }
        catch (PrincipiaException e) when (e.Status == Status.OutOfRange)
        {
            return null;  // A substep just outside the integrated range.
        }
        catch (Exception e)
        {
            Log.WarnOnce($"[vessels] gravity correction near {a.Id}", e);
            return null;
        }
        finally
        {
            system.Exit();
        }
    }
}

// A coasting vessel alone in its bubble, about to move along its conic.
[HarmonyPatch(typeof(PhysicsBubble), "ApplySingleFreefallMotion")]
internal static class ApplySingleFreefallMotionPatch
{
    [HarmonyPrefix]
    private static void Prefix(UniverseTime nextTime, VehicleUpdateState vehicleState)
    {
        using var profiled = FrameProfiler.VesselStep.Measure();
        try
        {
            VesselDriver.BeforeFreefall(vehicleState, nextTime);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[vessels] n-body coasting step", e);
        }
    }
}

// KSA also moves coasting vessels along their conic inside full-physics steps (past their verified conics, an
// atmosphere or an impact, often at high warp, or sharing a bubble), without the hook above.
[HarmonyPatch(typeof(PhysicsBubble), "RunUnconstrainedVehicleStep")]
internal static class RunUnconstrainedVehicleStepPatch
{
    private static readonly AccessTools.FieldRef<PhysicsBubble, List<VehicleUpdateState>> VehicleStates =
        AccessTools.FieldRefAccess<PhysicsBubble, List<VehicleUpdateState>>("_vehicleStates");

    private static readonly Func<VehicleUpdateState, bool> IsLadderParked =
        AccessTools.MethodDelegate<Func<VehicleUpdateState, bool>>(
            AccessTools.Method(typeof(PhysicsBubble), "IsLadderParked"));

    // The job is a private struct, readable only boxed (__args).
    private static readonly Type JobType = AccessTools.Inner(typeof(PhysicsBubble), "UnconstrainedVehicleStepJob");
    private static readonly Func<object, SimStep> SubStep = JobField<SimStep>("SubStep");
    private static readonly Func<object, int> VehicleIdx = JobField<int>("VehicleIdx");

    private static Func<object, T> JobField<T>(string name)
    {
        ParameterExpression job = Expression.Parameter(typeof(object));
        return Expression.Lambda<Func<object, T>>(Expression.Field(Expression.Unbox(job, JobType), name), job)
                         .Compile();
    }

    [HarmonyPrefix]
    private static void Prefix(PhysicsBubble __instance, object[] __args)
    {
        using var profiled = FrameProfiler.VesselStep.Measure();
        try
        {
            SimStep subStep = SubStep(__args[0]);
            int index = VehicleIdx(__args[0]);
            VehicleUpdateState vehicleState = VehicleStates(__instance)[index];
            if (vehicleState.GetNewStates().Props.Situation != Situation.Freefall || IsLadderParked(vehicleState))
            {
                return;
            }
            Interlocked.Increment(ref VesselDriver.FullPhysicsFlows);
            VesselDriver.BeforeFreefall(vehicleState, subStep.NextTime);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[vessels] n-body coasting step in full physics", e);
        }
    }
}

// Not on ground or water: KSA's terrain is a sphere, so J2's sideways pull would make parked vessels creep.
[HarmonyPatch]
internal static class ComputeDerivativesPatch
{
    private static MethodBase TargetMethod() =>
        AccessTools.GetDeclaredMethods(typeof(PhysicsStates))
                   .Single(m => m.Name == nameof(PhysicsStates.ComputeDerivatives) && m.IsStatic);

    [HarmonyPostfix]
    private static void Postfix(in BubbleOrigin origin,
                                in KinematicStates kinematic,
                                in VehicleProperties props,
                                in PhysicsEnvironment environment,
                                double3 positionOriginBub,
                                ref Disturbances __result)
    {
        using var profiled = FrameProfiler.ThrustGravity.Measure();
        try
        {
            if (props.Situation.HasAnyContact() || environment.ClosestParent != origin.Parent)
            {
                return;
            }
            if (VesselDriver.Correction(origin.Parent, origin.BubFrame == BubbleFrame.Ccf,
                                        positionOriginBub + kinematic.PositionPhys, origin.Time) is { } a)
            {
                __result.AddAccelPhys(a);
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[vessels] gravity correction", e);
        }
    }
}
