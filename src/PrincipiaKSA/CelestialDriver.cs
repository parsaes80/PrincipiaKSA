using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// KSA's planets and moons follow Principia. Each step, a body's state comes
// from Principia (StateFor); between frames, its conic is rebuilt as the
// osculating orbit where it has drifted (RefreshOrbits), as KSA also uses it
// for orbit lines and encounters. A body's equator is defined relative to its
// orbit's plane, so the rebuild keeps the equator where it was.
internal static class CelestialDriver
{
    // Rebuild a conic off by more than this fraction of its distance.
    private const double RefreshTolerance = 1e-6;

    private static readonly AccessTools.FieldRef<Celestial, doubleQuat> Orb2Cci =
        AccessTools.FieldRefAccess<Celestial, doubleQuat>("_orb2Cci");

    public static long StatesServed;
    public static long OrbitsRefreshed;
    private static bool _selfTestLogged;

    // Principia's state for `body` at `time`, or null if it isn't driven.
    // Thread-safe.
    public static StateVectors? StateFor(IOrbiter body, UniverseTime time)
    {
        PrincipiaSystem? system = PrincipiaSystem.Current;
        if (system == null || !system.Driving || body is not Astronomical a || !system.TryGetLink(a, out var link) ||
            !system.Enter())
        {
            return null;
        }
        try
        {
            (double3 q, double3 p) = RelativeEcl(system, link, GameTime.ToSecondsSinceJ2000(time));
            doubleQuat cce2Cci = link.Parent.GetCci2Cce().Inverse();
            double3 qCci = q.Transform(cce2Cci);
            double3 pCci = p.Transform(cce2Cci);
            double3 qOrb = qCci.Transform(body.Orbit.Orb2ParentCci.Inverse());
            var trueAnomaly = new TrueAnomaly(MathEx.ToOrbitAngle(Math.Atan2(qOrb.Y, qOrb.X)));
            Interlocked.Increment(ref StatesServed);
            return new StateVectors(time, qCci, pCci, trueAnomaly);
        }
        catch (Exception e)
        {
            system.Fail($"state of {a.Id} at t = {time.Seconds():F3} s: {e.Message}");
            return null;
        }
        finally
        {
            system.Exit();
        }
    }

    // Body minus parent, in Ecl axes, from Principia at `t` (s since J2000).
    public static (double3 Q, double3 P) RelativeEcl(PrincipiaSystem system, PrincipiaSystem.Link link, double t)
    {
        QP s = system.Ephemeris.State(link.Index, t);
        QP parent = system.Ephemeris.State(link.ParentIndex, t);
        return (s.Q.ToDouble3() - parent.Q.ToDouble3(), s.P.ToDouble3() - parent.P.ToDouble3());
    }

    // Main thread, between frames: rebuilds drifted conics, or all with `force`.
    public static void RefreshOrbits(bool force = false)
    {
        PrincipiaSystem? system = PrincipiaSystem.Current;
        if (system == null || !system.Driving)
        {
            return;
        }
        try
        {
            foreach (var (body, _) in system.Links)
            {
                if (body is not Celestial celestial)
                {
                    continue;  // OrbitingStar: no Orb2Cci to keep; leave its conic alone.
                }
                StateVectors state = celestial.Orbit.StateVectors;
                double distance = state.PositionCci.Length();
                double drift = (celestial.Orbit.GetStateVectorsAt(state.StateTime).PositionCci -
                                state.PositionCci).Length();
                if (force || drift > RefreshTolerance * distance)
                {
                    Replace(celestial, state);
                }
            }
        }
        catch (Exception e)
        {
            system.Fail($"refreshing orbits: {e}");
        }
    }

    // Moves every driven body to Principia's state at once, after the game
    // time jumps.
    public static void JumpTo(UniverseTime time)
    {
        PrincipiaSystem? system = PrincipiaSystem.Current;
        if (system == null || !system.Driving)
        {
            return;
        }
        system.Prolonger.EnsureProlonged(GameTime.ToSecondsSinceJ2000(time));
        foreach (var (body, _) in system.Links)
        {
            if (StateFor((IOrbiter)body, time) is { } state)
            {
                ((IOrbiter)body).Orbit.UpdatePosition(state);
            }
        }
        RefreshOrbits(force: true);
    }

    // Replaces the body's conic with the osculating orbit of `state` for the
    // two-body problem with μ = G(M_parent + M_body), keeping its equator.
    private static void Replace(Celestial body, StateVectors state)
    {
        Orbit old = body.Orbit;
        IParentBody parent = old.Parent;
        double totalMass = parent.Mass + body.Mass;
        Orbit fresh = Osculating(parent, state, totalMass, old.OrbitLineColor);

        // UpdatePerFrameData computes Cci→Cce as (ParentCce2Orb ∘ Orb2Cci)⁻¹.
        // Keep the composition, so the equator doesn't move.
        doubleQuat composition = doubleQuat.Concatenate(old.GetParentCce2Orb(), Orb2Cci(body));
        body.SetOrbit(fresh);
        Orb2Cci(body) = doubleQuat.Concatenate(fresh.GetParentCce2Orb().Inverse(), composition);
        Interlocked.Increment(ref OrbitsRefreshed);

        if (!_selfTestLogged)
        {
            _selfTestLogged = true;
            double3 rebuilt = fresh.GetStateVectorsAt(state.StateTime).PositionCci;
            doubleQuat after = doubleQuat.Concatenate(fresh.GetParentCce2Orb(), Orb2Cci(body));
            double3 x0 = double3.UnitX.Transform(composition), x1 = double3.UnitX.Transform(after);
            double3 z0 = double3.UnitZ.Transform(composition), z1 = double3.UnitZ.Transform(after);
            Log.Info($"[orbits] first rebuild ({body.Id}): conic reproduces the state to " +
                     $"{(rebuilt - state.PositionCci).Length():E3} m; equator moved by " +
                     $"{Math.Max((x1 - x0).Length(), (z1 - z0).Length()):E3} rad");
        }
    }

    // The osculating orbit of `state` about `centralMass`. KSA builds it with
    // the parent's μ; the elements depend on the velocity only through v²/μ
    // and (r·v)·v/μ, so scaling the velocity by k = √(μ_parent/μ_central)
    // gives them for μ_central, with the time since periapsis scaled by k.
    public static Orbit Osculating(IParentBody parent, StateVectors state, double centralMass, byte4 color)
    {
        double k = Math.Sqrt(parent.Mass / centralMass);
        Orbit scaled = Orbit.CreateFromStateCci(parent, state.StateTime, state.PositionCci,
                                                state.VelocityCci * k, color);
        OrbitData d = scaled.GetData();
        double sincePeriapsis = (state.StateTime - d.TimeAtPeriapsis).Seconds() * k;
        double period = d.IsBound() ? Orbit.Bound.GetPeriod(centralMass, d.SemiMajorAxis) : double.NaN;
        var data = new OrbitData(state.StateTime - sincePeriapsis, d.Eccentricity, d.SemiMajorAxis,
                                 d.SemiMinorAxis, d.Inclination, d.LongitudeOfAscendingNode,
                                 d.ArgumentOfPeriapsis, period, d.Periapsis, d.Apoapsis, d.Orb2ParentCci);
        return Orbit.Create(parent, data, color, state, centralMass);
    }
}
