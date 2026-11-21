using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Brutal.Numerics;

namespace PrincipiaKSA;

// Mirrors of the structs in native/principia_ksa.h. Same field order and
// types, so they cross the boundary without copying or conversion.
[StructLayout(LayoutKind.Sequential)]
internal struct XYZ
{
    public double X, Y, Z;

    public XYZ(double x, double y, double z) => (X, Y, Z) = (x, y, z);
    public XYZ(double3 v) => (X, Y, Z) = (v.X, v.Y, v.Z);

    public double3 ToDouble3() => new(X, Y, Z);
    public override string ToString() => $"({X}, {Y}, {Z})";
}

// A state: position Q and velocity P.
[StructLayout(LayoutKind.Sequential)]
internal struct QP
{
    public XYZ Q, P;

    public QP(double3 q, double3 p) => (Q, P) = (new XYZ(q), new XYZ(p));
}

[StructLayout(LayoutKind.Sequential)]
internal unsafe struct NativeBody
{
    public byte* Name;                    // UTF-8, null-terminated.
    public double GravitationalParameter; // m³/s²
    public double MeanRadius;             // m; ≤ 0: point mass
    public XYZ PolarAxis;
    public XYZ PrimeMeridian;
    public double ReferenceTime;          // s since J2000
    public double AngularFrequency;       // rad/s
    public byte* GravityModelName;        // UTF-8 or null
}

internal enum Status
{
    Ok = 0,
    InvalidArgument = 1,
    OutOfRange = 2,
    PrincipiaError = 3,
    Internal = 4,
}

// An exact apsis of a prediction relative to a body.
[StructLayout(LayoutKind.Sequential)]
internal struct Apsis
{
    public int Body;          // Ephemeris index.
    public int IsApoapsis;    // 0: periapsis (closest approach), 1: apoapsis.
    public double Time;       // s since J2000.
    public double Distance;   // m from the body's centre.
    public QP State;          // Absolute.
}

// Where a native Predict spent its time (ms); see principia_ksa.h.
[StructLayout(LayoutKind.Sequential)]
internal struct PredictTimings
{
    public double Total;
    public double Prolong;
    public double GiveWay;
    public long ProlongPieces;
    public double Flow;
    public double Apsides;
    public double Thinning;
    public long Chunks;
}

internal enum PredictionEnd
{
    ReachedEnd = 0,
    MaxSteps = 1,
    Collision = 2,
}

internal sealed class PrincipiaException(Status status, string message)
    : Exception($"{status}: {message}")
{
    public Status Status { get; } = status;
}

// Bindings to the native library (native/principia_ksa.h); see Ephemeris.
internal static unsafe partial class Native
{
    private const string LibraryName = "principia_ksa";

    // StarMap loads mods in their own load context: find the library next to
    // our DLL rather than by default probing.
    public static void RegisterResolver()
    {
        NativeLibrary.SetDllImportResolver(typeof(Native).Assembly, Resolve);
    }

    private static IntPtr _handle;

    // Built for AVX and FMA (or Apple Silicon) only; never loaded on a CPU
    // without them, where it would crash the game.
    public static readonly string LibraryFile =
        OperatingSystem.IsWindows() ? $"{LibraryName}_AVX.dll" :
        OperatingSystem.IsMacOS() ? $"lib{LibraryName}_arm64.dylib" : $"lib{LibraryName}_AVX.so";

    public static bool CpuSupported => OperatingSystem.IsMacOS()
        ? RuntimeInformation.ProcessArchitecture == Architecture.Arm64
        : System.Runtime.Intrinsics.X86.Avx.IsSupported && System.Runtime.Intrinsics.X86.Fma.IsSupported;

    private static IntPtr Resolve(string name, Assembly assembly, DllImportSearchPath? searchPath)
    {
        if (name != LibraryName)
        {
            return IntPtr.Zero;
        }
        if (_handle == IntPtr.Zero)
        {
            if (!CpuSupported)
            {
                throw new DllNotFoundException($"{LibraryFile} needs a CPU with AVX and FMA, or Apple Silicon");
            }
            string path = Path.Combine(Path.GetDirectoryName(assembly.Location) ?? ".", LibraryFile);
            Log.Info($"Loading native library {path}");
            _handle = NativeLibrary.Load(path);
        }
        return _handle;
    }

    public static void Check(Status status)
    {
        if (status != Status.Ok)
        {
            throw new PrincipiaException(status, LastError());
        }
    }

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__Version")]
    private static partial IntPtr VersionPtr();
    public static string Version() => Marshal.PtrToStringUTF8(VersionPtr()) ?? "";

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__LastError")]
    private static partial IntPtr LastErrorPtr();
    public static string LastError() => Marshal.PtrToStringUTF8(LastErrorPtr()) ?? "";

    // Principia's log goes to ours, from any native thread.
    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__SetLogCallback")]
    private static partial void SetLogCallback(IntPtr callback);

    public static void EnableLogging() =>
        SetLogCallback((IntPtr)(delegate* unmanaged[Cdecl]<int, byte*, void>)&OnLog);
    public static void DisableLogging() => SetLogCallback(IntPtr.Zero);

    [UnmanagedCallersOnly(CallConvs = [typeof(CallConvCdecl)])]
    private static void OnLog(int severity, byte* message)
    {
        try
        {
            string text = "[principia] " + (Marshal.PtrToStringUTF8((IntPtr)message) ?? "");
            switch (severity)
            {
                case 0: Log.Info(text); break;
                case 1: Log.Warn(text); break;
                default: Log.Error(text); break;  // error or fatal
            }
        }
        catch
        {
            // Nothing may unwind into C++.
        }
    }

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__LoadGravityModel",
                   StringMarshalling = StringMarshalling.Utf8)]
    public static partial Status LoadGravityModel(string path);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__GravityModelBody",
                   StringMarshalling = StringMarshalling.Utf8)]
    public static partial Status GravityModelBody(string name, out double gravitationalParameter,
                                                  out double meanRadius, out double rightAscension,
                                                  out double declination);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__EphemerisCreate")]
    public static partial Status EphemerisCreate(NativeBody* bodies, QP* states, int count,
                                                 double t0, double step, double fittingTolerance,
                                                 out IntPtr ephemeris);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__EphemerisDestroy")]
    public static partial void EphemerisDestroy(IntPtr ephemeris);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__EphemerisProlong")]
    public static partial Status EphemerisProlong(IntPtr ephemeris, double t);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__EphemerisTimeRange")]
    public static partial Status EphemerisTimeRange(IntPtr ephemeris, out double tMin, out double tMax);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__EphemerisBodyState")]
    public static partial Status EphemerisBodyState(IntPtr ephemeris, int index, double t, out QP state);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__EphemerisRelativeAcceleration")]
    public static partial Status EphemerisRelativeAcceleration(IntPtr ephemeris, int index, XYZ relativePosition,
                                                               double t, out XYZ acceleration);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__FlowMassless")]
    public static partial Status FlowMassless(IntPtr ephemeris, in QP initial, double t0, double t1,
                                              out QP finalState);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__Predict")]
    public static partial Status Predict(IntPtr ephemeris, in QP initial, double t0, double tEnd, long maxSteps,
                                         double lengthTolerance, double speedTolerance, XYZ thrustDirection,
                                         double thrust, double initialMass, double massFlow,
                                         double downsamplingTolerance, int* apsisBodies, int apsisBodyCount,
                                         int apsisCapacity, out double* times, out QP* states, out int count,
                                         out Apsis* apsides, out int apsisCount, out long steps,
                                         out PredictionEnd end);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__Free")]
    public static partial void Free(void* array);

    [LibraryImport(LibraryName, EntryPoint = "principia_ksa__LastPredictTimings")]
    public static partial void LastPredictTimings(out PredictTimings timings);
}
