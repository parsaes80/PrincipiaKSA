using System.Runtime.InteropServices;
using Brutal.Numerics;

namespace PrincipiaKSA;

// MeanRadius ≤ 0: a point mass; otherwise it rotates as given, with GravityModelName's geopotential if there is one.
internal readonly record struct BodySpec(
    string Name,
    double Mu,
    double MeanRadius = 0,
    double3 PolarAxis = default,
    double3 PrimeMeridian = default,
    double ReferenceTime = 0,
    double AngularFrequency = 0,
    string? GravityModelName = null);

// A native Principia ephemeris: seconds since J2000, KSA's ecliptic axes about the barycentre, SI units. Throws
// PrincipiaException on native errors; reading states is thread-safe within the integrated range.
internal sealed class Ephemeris : IDisposable
{
    // Principia's KSP plugin settings.
    private const double Step = 35 * 60;           // s
    private const double FittingTolerance = 1e-3;  // m

    private IntPtr _handle;

    private Ephemeris(IntPtr handle) => _handle = handle;

    public static unsafe Ephemeris Create(IReadOnlyList<BodySpec> bodies, ReadOnlySpan<QP> states, double t0)
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
                Native.Check(Native.EphemerisCreate(b, s, bodies.Count, t0, Step, FittingTolerance, out IntPtr handle));
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

    public void Prolong(double t)
    {
        using var profiled = FrameProfiler.NativeProlong.Measure();
        Native.Check(Native.EphemerisProlong(Handle, t));
    }

    public (double TMin, double TMax) TimeRange
    {
        get
        {
            using var profiled = FrameProfiler.NativeTimeRange.Measure();
            Native.Check(Native.EphemerisTimeRange(Handle, out double tMin, out double tMax));
            return (tMin, tMax);
        }
    }

    public QP State(int index, double t)
    {
        using var profiled = FrameProfiler.NativeState.Measure();
        Native.Check(Native.EphemerisBodyState(Handle, index, t, out QP state));
        return state;
    }

    // At `relativePosition` from body `index`, relative to it: its own field with geopotential, plus the others' tides.
    public XYZ RelativeAcceleration(int index, XYZ relativePosition, double t)
    {
        using var profiled = FrameProfiler.NativeAcceleration.Measure();
        Native.Check(Native.EphemerisRelativeAcceleration(Handle, index, relativePosition, t, out XYZ a));
        return a;
    }

    public QP FlowMassless(in QP initial, double t0, double t1)
    {
        using var profiled = FrameProfiler.NativeFlow.Measure();
        Native.Check(Native.FlowMassless(Handle, in initial, t0, t1, out QP final));
        return final;
    }

    // A prediction keeps the steps a cubic Hermite spline needs to stay within this of its path (m), as Principia does.
    public const double DownsamplingTolerance = 10;

    private const int ApsisCapacity = 100_000;

    public readonly record struct PredictionResult(double[] Times, QP[] States, Apsis[] Apsides, long Steps,
                                             PredictionEnd End);

    // As Principia predicts a vessel, until `maxSteps`, tEnd (+∞: none) or a collision; thrust needs a finite tEnd.
    public unsafe PredictionResult Predict(in QP initial, double t0, double tEnd, long maxSteps, double tolerance,
                                     double3 direction, double thrust, double initialMass, double massFlow,
                                     ReadOnlySpan<int> apsisBodies)
    {
        using var profiled = FrameProfiler.NativePredict.Measure();
        fixed (int* bodies = apsisBodies)
        {
            Status status = Native.Predict(Handle, in initial, t0, tEnd, maxSteps, tolerance, tolerance,
                                           new XYZ(direction), thrust, initialMass, massFlow,
                                           DownsamplingTolerance, bodies, apsisBodies.Length, ApsisCapacity,
                                           out double* times, out QP* states, out int count,
                                           out Apsis* apsides, out int apsisCount, out long steps,
                                           out PredictionEnd end);
            Native.LastPredictTimings(out PredictTimings timings);
            PredictionTiming.AddNative(timings, apsisBodies.Length);
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
