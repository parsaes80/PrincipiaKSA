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

    // How closely a prediction's points follow its path: every integrator
    // step, thinned to those a cubic Hermite spline needs to stay within this
    // (m), as Principia keeps vessel histories.
    public const double DownsamplingTolerance = 10;

    // At most this many apsides per prediction, in all.
    private const int ApsisCapacity = 100_000;

    // A prediction: its points, the exact apsides relative to the bodies
    // asked for, the integrator steps it took, and why it stopped.
    public readonly record struct PredictionResult(double[] Times, QP[] States, Apsis[] Apsides, long Steps,
                                             PredictionEnd End);

    // Predicts a massless body from t0 as Principia predicts a vessel, with
    // its adaptive integrator at `tolerance` (m, and m/s), until it has taken
    // `maxSteps` steps, reached tEnd (+∞: no time limit) or hit a body;
    // optionally under thrust along a fixed direction (tEnd finite then).
    public unsafe PredictionResult Predict(in QP initial, double t0, double tEnd, long maxSteps, double tolerance,
                                     double3 direction, double thrust, double initialMass, double massFlow,
                                     ReadOnlySpan<int> apsisBodies)
    {
        fixed (int* bodies = apsisBodies)
        {
            Status status = Native.Predict(Handle, in initial, t0, tEnd, maxSteps, tolerance, tolerance,
                                           new XYZ(direction), thrust, initialMass, massFlow,
                                           DownsamplingTolerance, bodies, apsisBodies.Length, ApsisCapacity,
                                           out double* times, out QP* states, out int count,
                                           out Apsis* apsides, out int apsisCount, out long steps,
                                           out PredictionEnd end);
            try
            {
                Native.Check(status);
                return new PredictionResult(new ReadOnlySpan<double>(times, count).ToArray(),
                                      new ReadOnlySpan<QP>(states, count).ToArray(),
                                      new ReadOnlySpan<Apsis>(apsides, apsisCount).ToArray(), steps, end);
            }
            finally
            {
                Native.Free(times);
                Native.Free(states);
                Native.Free(apsides);
            }
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
