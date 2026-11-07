using System.Runtime.InteropServices;
using Brutal.Numerics;

namespace PrincipiaKSA;

// A massive body as Principia should model it. With MeanRadius ≤ 0 it is a
// point mass; otherwise it rotates as given, with the geopotential of
// GravityModelName if the loaded gravity model has one.
internal readonly record struct BodySpec(
    string Name,
    double Mu,
    double MeanRadius = 0,
    double3 PolarAxis = default,
    double3 PrimeMeridian = default,
    double ReferenceTime = 0,
    double AngularFrequency = 0,
    string? GravityModelName = null);

// A Principia ephemeris in the native library. Times are seconds since J2000;
// states are in KSA's ecliptic axes, metres and m/s, about the barycentre.
// Methods throw PrincipiaException on native errors; reading states is
// thread-safe within the integrated range.
internal sealed class Ephemeris : IDisposable
{
    // Principia's KSP plugin settings.
    public const double DefaultStep = 35 * 60;          // s
    public const double DefaultFittingTolerance = 1e-3; // m

    private IntPtr _handle;

    private Ephemeris(IntPtr handle) => _handle = handle;

    public static unsafe Ephemeris Create(IReadOnlyList<BodySpec> bodies, ReadOnlySpan<QP> states, double t0,
                                          double step = DefaultStep,
                                          double fittingTolerance = DefaultFittingTolerance)
    {
        if (bodies.Count != states.Length)
        {
            throw new ArgumentException($"{bodies.Count} bodies but {states.Length} states");
        }
        var strings = new List<IntPtr>();
        byte* Utf8(string? s)
        {
            if (s == null) return null;
            IntPtr p = Marshal.StringToCoTaskMemUTF8(s);
            strings.Add(p);
            return (byte*)p;
        }
        var native = new NativeBody[bodies.Count];
        try
        {
            for (int i = 0; i < bodies.Count; i++)
            {
                BodySpec b = bodies[i];
                native[i] = new NativeBody
                {
                    Name = Utf8(b.Name),
                    GravitationalParameter = b.Mu,
                    MeanRadius = b.MeanRadius,
                    PolarAxis = new XYZ(b.PolarAxis),
                    PrimeMeridian = new XYZ(b.PrimeMeridian),
                    ReferenceTime = b.ReferenceTime,
                    AngularFrequency = b.AngularFrequency,
                    GravityModelName = Utf8(b.GravityModelName),
                };
            }
            fixed (NativeBody* b = native)
            fixed (QP* s = states)
            {
                Native.Check(Native.EphemerisCreate(b, s, bodies.Count, t0, step, fittingTolerance,
                                                    out IntPtr handle));
                return new Ephemeris(handle);
            }
        }
        finally
        {
            foreach (IntPtr p in strings)
            {
                Marshal.FreeCoTaskMem(p);  // Principia copied them.
            }
        }
    }

    public void Prolong(double t) => Native.Check(Native.EphemerisProlong(Handle, t));

    public (double TMin, double TMax) TimeRange
    {
        get
        {
            Native.Check(Native.EphemerisTimeRange(Handle, out double tMin, out double tMax));
            return (tMin, tMax);
        }
    }

    public QP State(int index, double t)
    {
        Native.Check(Native.EphemerisBodyState(Handle, index, t, out QP state));
        return state;
    }

    // Acceleration at `relativePosition` from body `index`, relative to that
    // body: its own field, geopotential included, plus the other bodies' tides.
    public XYZ RelativeAcceleration(int index, XYZ relativePosition, double t)
    {
        Native.Check(Native.EphemerisRelativeAcceleration(Handle, index, relativePosition, t, out XYZ a));
        return a;
    }

    // Integrates a massless body (a vessel) from t0 to t1.
    public QP FlowMassless(in QP initial, double t0, double t1)
    {
        Native.Check(Native.FlowMassless(Handle, in initial, t0, t1, out QP final));
        return final;
    }

    // Predicts a massless body from t0 towards tEnd, optionally under thrust
    // along a fixed direction: points on the absolute grid of `sampleInterval`,
    // and the exact apsides relative to `apsisBodies`.
    public unsafe (int Count, PredictionEnd End, int ApsisCount) Predict(
        in QP initial, double t0, double tEnd, int maxSteps, double3 direction, double thrust,
        double initialMass, double massFlow, double sampleInterval, Span<double> times, Span<QP> states,
        ReadOnlySpan<int> apsisBodies, Span<Apsis> apsides)
    {
        fixed (double* t = times)
        fixed (QP* s = states)
        fixed (int* b = apsisBodies)
        fixed (Apsis* a = apsides)
        {
            Native.Check(Native.PredictMasslessDetailed(Handle, in initial, t0, tEnd, maxSteps, new XYZ(direction),
                                                        thrust, initialMass, massFlow, sampleInterval, t, s,
                                                        Math.Min(times.Length, states.Length), out int count,
                                                        b, apsisBodies.Length, a, apsides.Length,
                                                        out int apsisCount, out PredictionEnd end));
            return (count, end, apsisCount);
        }
    }

    private IntPtr Handle =>
        _handle != IntPtr.Zero ? _handle : throw new ObjectDisposedException(nameof(Ephemeris));

    public void Dispose()
    {
        if (_handle != IntPtr.Zero)
        {
            Native.EphemerisDestroy(_handle);
            _handle = IntPtr.Zero;
        }
    }
}
