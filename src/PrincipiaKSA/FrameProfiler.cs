using System.Diagnostics;
using System.Text;
using KSA;

namespace PrincipiaKSA;

// Finds what makes a frame stutter. Sections of code (KSA's per-frame methods,
// the mod's hooks, the native calls) are timed into three buckets by thread:
// KSA's main thread, KSA's job threads, and the mod's background threads.
// At the start of each frame the last frame's totals are taken; a frame much
// slower than usual is logged with its slowest sections ("[stutter]").
//
// Measure a section with `using var _ = FrameProfiler.Something.Measure();`.
internal static class FrameProfiler
{
    // A frame is a stutter from this long, and this many times the usual.
    private const double StutterMs = 40;
    private const double StutterFactor = 3;
    // At most one report this often; the slow frames between are counted.
    private static readonly long ReportInterval = Stopwatch.Frequency;
    // Sections shorter than this in a stutter aren't listed.
    private const double ListedMs = 0.5;
    private const int ListedPerBucket = 12;

    private const int Main = 0, Jobs = 1, Background = 2, Buckets = 3;
    private static readonly string[] BucketNames =
        ["main thread", "KSA's job threads, summed over threads", "the mod's background threads, finished during the frame"];

    // Every section, in the order defined. Defined first: the fields below use it.
    private static readonly List<Section> All = new();

    public static Section Define(string name)
    {
        var section = new Section(name);
        lock (All)
        {
            All.Add(section);
        }
        return section;
    }

    // The mod's work on KSA's threads.
    public static readonly Section ProlongToNextStep = Define("mod: prolong to the next step");
    public static readonly Section BurnTargets = Define("mod: burn target refresher");
    public static readonly Section PlanetConics = Define("mod: rebuild planet conics");
    public static readonly Section Snapshot = Define("mod: prediction snapshot");
    public static readonly Section StatusAndSettings = Define("mod: status log and settings");
    public static readonly Section MapOverlay = Define("mod: map markers and window");
    public static readonly Section Window = Define("mod: PrincipiaKSA window");
    public static readonly Section OrbitLines = Define("mod: orbit lines");
    public static readonly Section ImpactOverride = Define("mod: impact override");
    public static readonly Section LineHover = Define("mod: line hover");
    public static readonly Section NodeHover = Define("mod: node hover");
    public static readonly Section BurnMenuSnap = Define("mod: burn menu snap");
    public static readonly Section NodeTime = Define("mod: exact node time");
    public static readonly Section BurnAnchor = Define("mod: burn anchor");
    public static readonly Section WarpApsides = Define("mod: Ap/Pe warp targets");
    public static readonly Section NodeDrag = Define("mod: node drag");
    public static readonly Section PlanetState = Define("mod: planet state");
    public static readonly Section VesselStep = Define("mod: vessel n-body step");
    public static readonly Section ThrustGravity = Define("mod: thrust gravity correction");

    // The mod's own threads.
    public static readonly Section Prediction = Define("mod: prediction");
    public static readonly Section FlightPlan = Define("mod: flight plan");
    public static readonly Section NodeTrack = Define("mod: node track");
    public static readonly Section ProlongAhead = Define("mod: prolong ahead");

    // Principia, through the C API, from any thread.
    public static readonly Section NativeProlong = Define("native: Prolong");
    public static readonly Section NativeState = Define("native: body state");
    public static readonly Section NativeTimeRange = Define("native: time range");
    public static readonly Section NativeAcceleration = Define("native: relative acceleration");
    public static readonly Section NativeFlow = Define("native: flow vessel");
    public static readonly Section NativePredict = Define("native: Predict");

    internal sealed class Section(string name)
    {
        public readonly string Name = name;
        internal readonly long[] Ticks = new long[Buckets];
        internal readonly long[] Calls = new long[Buckets];
        internal readonly long[] Longest = new long[Buckets];

        public Measurement Measure() => new(this);

        internal void Add(int bucket, long ticks)
        {
            Interlocked.Add(ref Ticks[bucket], ticks);
            Interlocked.Increment(ref Calls[bucket]);
            long seen = Volatile.Read(ref Longest[bucket]);
            while (ticks > seen)
            {
                long previous = Interlocked.CompareExchange(ref Longest[bucket], ticks, seen);
                if (previous == seen)
                {
                    break;
                }
                seen = previous;
            }
        }
    }

    // One timed run of a section, ended by Dispose.
    public readonly struct Measurement : IDisposable
    {
        private readonly Section? _section;
        private readonly long _start;
        private readonly bool _outermost;

        public Measurement(Section section)
        {
            _section = section;
            _outermost = _depth++ == 0;
            _start = Stopwatch.GetTimestamp();
        }

        public void Dispose()
        {
            if (_section == null)
            {
                return;
            }
            long ticks = Stopwatch.GetTimestamp() - _start;
            _depth--;
            int bucket = CurrentBucket;
            _section.Add(bucket, ticks);
            if (_outermost && bucket == Main)
            {
                Interlocked.Add(ref _mainOutermostTicks, ticks);
            }
        }
    }

    // Timed sections running on this thread, to tell outermost ones from
    // those nested in them.
    [ThreadStatic] private static int _depth;
    [ThreadStatic] private static bool _background;

    // Marks the calling thread as one of the mod's own, for good.
    public static void MarkBackgroundThread() => _background = true;

    // Marks a thread-pool thread as the mod's while a task runs on it.
    public readonly struct BackgroundScope : IDisposable
    {
        private readonly bool _was;
        public BackgroundScope() { _was = _background; _background = true; }
        public void Dispose() => _background = _was;
    }

    private static int _mainThreadId = -1;
    private static int CurrentBucket =>
        _background ? Background : Environment.CurrentManagedThreadId == _mainThreadId ? Main : Jobs;

    private static long _mainOutermostTicks;
    private static long _frameStart, _nextReport, _frames;
    private static double _usualMs;
    private static int _slowSinceReport;
    private static TimeSpan _lastGcPause;
    private static readonly int[] LastCollections = new int[3];
    private static long _lastCorrections;

    private static double Ms(long ticks) => ticks * 1000.0 / Stopwatch.Frequency;

    // Main thread, at the start of each of KSA's frames (Program.OnFrame).
    public static void FrameStart()
    {
        long now = Stopwatch.GetTimestamp();
        _mainThreadId = Environment.CurrentManagedThreadId;
        long start = _frameStart;
        _frameStart = now;

        // The last frame's totals, reset for this one.
        var taken = new List<(Section Section, long[] Ticks, long[] Calls, long[] Longest)>();
        lock (All)
        {
            foreach (Section s in All)
            {
                long[] ticks = new long[Buckets], calls = new long[Buckets], longest = new long[Buckets];
                bool any = false;
                for (int b = 0; b < Buckets; b++)
                {
                    ticks[b] = Interlocked.Exchange(ref s.Ticks[b], 0);
                    calls[b] = Interlocked.Exchange(ref s.Calls[b], 0);
                    longest[b] = Interlocked.Exchange(ref s.Longest[b], 0);
                    any |= calls[b] > 0;
                }
                if (any)
                {
                    taken.Add((s, ticks, calls, longest));
                }
            }
        }
        long outermost = Interlocked.Exchange(ref _mainOutermostTicks, 0);
        TimeSpan gcPause = GC.GetTotalPauseDuration();
        int[] collections = [GC.CollectionCount(0), GC.CollectionCount(1), GC.CollectionCount(2)];
        long corrections = Interlocked.Read(ref VesselDriver.Corrections);
        TimeSpan framePause = gcPause - _lastGcPause;
        int[] frameCollections = [collections[0] - LastCollections[0], collections[1] - LastCollections[1],
                                  collections[2] - LastCollections[2]];
        long frameCorrections = corrections - _lastCorrections;
        _lastGcPause = gcPause;
        collections.CopyTo(LastCollections, 0);
        _lastCorrections = corrections;

        if (start == 0 || PrincipiaSystem.Current == null)
        {
            return;  // No frame before, or nothing of ours running.
        }
        double ms = Ms(now - start);
        // The first frames after a load aren't the usual.
        bool settled = ++_frames > 120;
        bool slow = settled && ms >= StutterMs && ms >= StutterFactor * _usualMs;
        if (!slow)
        {
            _usualMs = _usualMs == 0 ? ms : _usualMs + 0.05 * (ms - _usualMs);
            return;
        }
        if (now < _nextReport)
        {
            _slowSinceReport++;
            return;
        }
        _nextReport = now + ReportInterval;
        try
        {
            Report(ms, outermost, taken, framePause, frameCollections, frameCorrections);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[stutter] report", e);
        }
        _slowSinceReport = 0;
    }

    private static void Report(double ms, long outermost,
                               List<(Section Section, long[] Ticks, long[] Calls, long[] Longest)> taken,
                               TimeSpan gcPause, int[] collections, long corrections)
    {
        var text = new StringBuilder();
        text.Append($"[stutter] {ms:F0} ms frame, usually {_usualMs:F1} ms");
        if (_slowSinceReport > 0)
        {
            text.Append($" ({_slowSinceReport} more slow frames since the last report)");
        }
        text.Append($"; {Context(corrections)}");

        double timedMs = Ms(outermost);
        text.Append($"\n[stutter]   main thread: {timedMs:F1} ms in timed code, {Math.Max(0, ms - timedMs):F1} ms " +
                    "elsewhere (rendering and KSA's untimed work); a KSA method's time includes the mod's " +
                    "hooks and native calls inside it");
        for (int b = 0; b < Buckets; b++)
        {
            var rows = taken.Where(t => t.Calls[b] > 0)
                            .OrderByDescending(t => t.Ticks[b])
                            .ToList();
            if (rows.Count == 0)
            {
                continue;
            }
            text.Append($"\n[stutter]   {BucketNames[b]}:");
            int listed = 0;
            foreach (var row in rows)
            {
                double total = Ms(row.Ticks[b]);
                if (listed == ListedPerBucket || (total < ListedMs && listed > 0))
                {
                    break;
                }
                long calls = row.Calls[b];
                text.Append($"\n[stutter]     {total,8:F1} ms  {row.Section.Name}  ({calls:N0} call{(calls == 1 ? "" : "s")}");
                if (calls > 1)
                {
                    text.Append($", longest {Ms(row.Longest[b]):F1} ms");
                }
                text.Append(')');
                listed++;
            }
            int rest = rows.Count - listed;
            if (rest > 0)
            {
                text.Append($"\n[stutter]     ... {rest} shorter");
            }
        }
        if (collections.Any(c => c > 0) || gcPause.TotalMilliseconds >= ListedMs)
        {
            text.Append($"\n[stutter]   garbage collection: {gcPause.TotalMilliseconds:F1} ms paused " +
                        $"(collections: gen 0 ×{collections[0]}, gen 1 ×{collections[1]}, gen 2 ×{collections[2]})");
        }
        Log.Warn(text.ToString());
    }

    // What the game was doing: warp, the vessel, the lines.
    private static string Context(long corrections)
    {
        var parts = new List<string> { $"warp ×{Universe.GetSimulationSpeed():G4}" };
        if (Program.ControlledVehicle is { } vehicle)
        {
            parts.Add($"{vehicle.Id} {vehicle.Situation} around {(vehicle.Orbit.Parent as IObjectId)?.Id ?? "?"}");
            if (corrections > 0)
            {
                parts.Add($"under full physics ({corrections:N0} n-body gravity corrections)");
            }
        }
        static string Line(Prediction? p, string what) =>
            p is not { Times.Length: > 0 } ? $"no {what}"
            : $"{what} {(p.Times[^1] - p.Times[0]) / 86400:F1} d, {p.Times.Length:N0} points, {p.Steps:N0} steps, " +
              $"{p.ComputeMs:F0} ms to compute{(p.Impact ? ", ends in an impact" : "")}";
        parts.Add(Line(Predictor.Latest, "prediction"));
        parts.Add(Line(Predictor.LatestPlan, "flight plan"));
        return string.Join("; ", parts);
    }
}
