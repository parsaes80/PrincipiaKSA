using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// Startup check of KSA's frames, units, handedness and time against
// real-world values, which the mapping to Principia relies on.
//
// KSA's frames:
//   Ecl  global, ecliptic-aligned axes
//   Cce  centred on a body, ecliptic-aligned axes (parallel to Ecl)
//   Cci  centred on a body, inertial, Z = the body's spin axis
//   Ccf  centred on a body, rotating with it (Cci rotated about Z by ωt + θ₀)
// StateVectors.PositionCci is relative to the PARENT, in the parent's Cci.
internal static class FrameProbe
{
    private const double Deg = Math.PI / 180;
    private const double J2000Obliquity = 23.4392911 * Deg;

    public static void Run(CelestialSystem system)
    {
        UniverseTime t = Universe.GetElapsedTime();
        Log.Info("=== Frame probe ===");
        LogTime(t);

        Celestial? earth = Find(system, "Earth");
        Celestial? luna = Find(system, "Luna");
        if (earth == null || luna == null)
        {
            Log.Warn("Frame probe needs Earth and Luna (Sol system); skipped");
            return;
        }

        // Expected values are copied from Content/Core/Astronomicals.xml (ecliptic elements).
        CheckOrbit(earth, t, xmlInclinationDeg: 4.495777668608611E-03, xmlNodeDeg: 2.001005464352027E+02);
        CheckOrbit(luna, t, xmlInclinationDeg: 5.027765739305488E+00, xmlNodeDeg: 3.439586928342719E+02);
        foreach (string id in new[] { "Mars", "Jupiter" })
        {
            if (Find(system, id) is { } c) CheckOrbit(c, t, null, null);
        }

        CheckCciVsCce(luna, t);
        CheckEarthSpin(earth, t);
        CheckTimeRoundTrip();
        Log.Info("=== End frame probe ===");
    }

    private static void LogTime(UniverseTime t)
    {
        Log.Info($"[time] UniverseTime = {t.Nanoseconds} ns = {t.Seconds():F3} s");
        Log.Info($"[time] → {GameTime.ToSecondsSinceJ2000(t):F3} s since J2000 = JD {GameTime.ToJulianDay(t):F6} " +
                 $"≈ {GameTime.ToDateTime(t):yyyy-MM-dd HH:mm:ss} (expect 2025-11-30 at t=0)");
    }

    // Units and orientation of the ecliptic frame: the orbital angular momentum
    // h = r × v in Cce must reproduce the XML inclination and node. A mirrored
    // (left-handed) frame would flip h or give 360° − Ω.
    private static void CheckOrbit(Celestial body, UniverseTime t, double? xmlInclinationDeg, double? xmlNodeDeg)
    {
        double3 r = body.GetPositionCce(t);
        double3 v = body.GetVelocityCce(t);
        double3 h = double3.Cross(r, v);
        double inclination = Math.Acos(h.Z / Norm(h)) / Deg;
        double3 node = double3.Cross(double3.UnitZ, h);  // ascending node direction
        double nodeLongitude = Normalize360(Math.Atan2(node.Y, node.X) / Deg);
        double mu = body.Parent.Mu;
        double a = 1 / (2 / Norm(r) - Dot(v, v) / mu);  // vis-viva

        Log.Info($"[orbit] {body.Id} around {(body.Parent as IObjectId)?.Id}: |r|={Norm(r):E6} m, |v|={Norm(v):F3} m/s, " +
                 $"a(vis-viva)={a:E6} m, period={body.Orbit.Period / 86400:F3} d");
        string expected = xmlInclinationDeg is { } xi && xmlNodeDeg is { } xn
            ? $" | XML i={xi:F4}°, Ω={xn:F4}° → {(Math.Abs(inclination - xi) < 0.01 && Math.Abs(Normalize180(nodeLongitude - xn)) < 0.01 ? "MATCH" : "MISMATCH")}"
            : "";
        Log.Info($"[orbit] {body.Id}: h·Z>0={h.Z > 0} (prograde about +Z), i={inclination:F4}°, Ω={nodeLongitude:F4}°{expected}");
    }

    // StateVectors use the parent's Cci (equatorial for Earth); the rotation
    // Cci → Cce must map one onto the other.
    private static void CheckCciVsCce(Celestial moon, UniverseTime t)
    {
        double3 cci = moon.GetPositionCci(t);
        double3 cce = moon.GetPositionCce(t);
        double3 cciRotated = cci.Transform(moon.Parent.GetCci2Cce());
        double angle = Math.Acos(Math.Clamp(Dot(cci, cce) / (Norm(cci) * Norm(cce)), -1, 1)) / Deg;
        Log.Info($"[cci] {moon.Id}: Cci=({Fmt(cci)}) Cce=({Fmt(cce)}) angle between={angle:F3}°");
        Log.Info($"[cci] {moon.Id}: |Cci·Cci2Cce − Cce| = {Norm(cciRotated - cce):E3} m " +
                 $"({(Norm(cciRotated - cce) < 1 ? "OK: Cci is the parent's equatorial frame" : "MISMATCH")})");
        Log.Info($"[cci] {moon.Id}: Orbit.StateVectors.PositionCci = ({Fmt(moon.Orbit.StateVectors.PositionCci)}) " +
                 $"at t={moon.Orbit.StateVectors.StateTime.Seconds():F3} s");
    }

    // Earth's spin axis must be close to the real J2000 north pole in ecliptic
    // coordinates, (0, +sin ε, cos ε) (ecliptic longitude 90°: at the June
    // solstice the pole leans toward the Sun), and the Ccf frame must turn
    // counter-clockwise about it once per sidereal day. KSA's pole sits at
    // longitude 89.636° (XML Azimuth −0.3636°), i.e. precessed to 2025: ~0.15°.
    private static void CheckEarthSpin(Celestial earth, UniverseTime t)
    {
        double3 axis = earth.GetRotationAxisCce();
        double3 truePole = new(0, Math.Sin(J2000Obliquity), Math.Cos(J2000Obliquity));
        double poleError = Math.Acos(Math.Clamp(Dot(axis, truePole) / Norm(axis), -1, 1)) / Deg;
        double omega = earth.GetAngularVelocity();
        Log.Info($"[spin] Earth axis (Cce) = ({axis.X:F6}, {axis.Y:F6}, {axis.Z:F6}); " +
                 $"real J2000 pole = ({truePole.X:F6}, {truePole.Y:F6}, {truePole.Z:F6}); off by {poleError:F3}°");
        Log.Info($"[spin] Earth ω = {omega:E9} rad/s → sidereal day {2 * Math.PI / omega / 3600:F6} h (expect 23.934470)");

        // Track a point on Earth's surface (Ccf X axis) through one hour in Cci.
        UniverseTime t1 = UniverseTime.FromNanoseconds(t.Nanoseconds + (Int128)3600 * UniverseTime.NANOSECONDS_PER_SECOND);
        double3 p0 = double3.UnitX.Transform(earth.GetCcf2Cci(t));
        double3 p1 = double3.UnitX.Transform(earth.GetCcf2Cci(t1));
        double turned = Math.Atan2(double3.Cross(p0, p1).Z, Dot(p0, p1)) / Deg;
        Log.Info($"[spin] Ccf X axis turns {turned:F4}° about +Z(Cci) in 1 h (expect +15.0411° for counter-clockwise)");
    }

    private static void CheckTimeRoundTrip()
    {
        foreach (double years in new[] { 0.0, 1.0, 100.0, 1000.0 })
        {
            UniverseTime t = UniverseTime.FromNanoseconds((Int128)(years * 365.25 * 86400) * UniverseTime.NANOSECONDS_PER_SECOND + 123_456_789);
            double s = GameTime.ToSecondsSinceJ2000(t);
            Int128 error = GameTime.FromSecondsSinceJ2000(s).Nanoseconds - t.Nanoseconds;
            Log.Info($"[time] round trip at +{years} y: {s:F9} s since J2000, error {error} ns");
        }
    }

    private static Celestial? Find(CelestialSystem system, string id)
    {
        var e = system.All.OfType<Celestial>().GetEnumerator();
        while (e.MoveNext())
        {
            if (e.Current.Id == id) return e.Current;
        }
        return null;
    }

    private static double Dot(double3 a, double3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
    private static double Norm(double3 a) => Math.Sqrt(Dot(a, a));
    private static double Normalize360(double deg) => ((deg % 360) + 360) % 360;
    private static double Normalize180(double deg) => Normalize360(deg + 180) - 180;
    private static string Fmt(double3 a) => $"{a.X:E6}, {a.Y:E6}, {a.Z:E6}";
}
