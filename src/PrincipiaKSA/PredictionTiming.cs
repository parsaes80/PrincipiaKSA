using System.Diagnostics;
using System.Text;

namespace PrincipiaKSA;

// Finds slow predictions. Each computation (the prediction, the flight plan)
// times its stages on its own thread: the native Predict calls, split as the
// library reports them (PredictTimings), and the C# stages around them. A
// computation slower than SlowMs is logged with the breakdown ("[predict]").
internal static class PredictionTiming
{
    private const double SlowMs = 150;
    private static readonly long ReportInterval = Stopwatch.Frequency;

    // This thread's computation.
    [ThreadStatic] private static Dictionary<string, double>? _stages;
    [ThreadStatic] private static PredictTimings _native;
    [ThreadStatic] private static int _nativeCalls, _apsisBodies;

    // When each kind was last logged, and how many slow ones were not.
    private static readonly Dictionary<string, (long Next, int Skipped)> Reported = new();

    public static void Begin()
    {
        (_stages ??= new()).Clear();
        _native = default;
        _nativeCalls = _apsisBodies = 0;
    }

    // `using (PredictionTiming.Stage("..."))` around a C# stage.
    public static Measurement Stage(string name) => new(name);

    public readonly struct Measurement(string name) : IDisposable
    {
        private readonly long _start = Stopwatch.GetTimestamp();

        public void Dispose()
        {
            if (_stages is { } stages)
            {
                stages[name] = stages.GetValueOrDefault(name) + Ms(Stopwatch.GetTimestamp() - _start);
            }
        }
    }

    // After each native Predict on this thread.
    public static void AddNative(in PredictTimings t, int apsisBodies)
    {
        _nativeCalls++;
        _apsisBodies = Math.Max(_apsisBodies, apsisBodies);
        _native.Total += t.Total;
        _native.Prolong += t.Prolong;
        _native.GiveWay += t.GiveWay;
        _native.ProlongPieces += t.ProlongPieces;
        _native.Flow += t.Flow;
        _native.Apsides += t.Apsides;
        _native.Thinning += t.Thinning;
        _native.Chunks += t.Chunks;
    }

    private static double Ms(long ticks) => ticks * 1000.0 / Stopwatch.Frequency;

    // Logs the computation if it was slow. `waitedMs`: from its request
    // (the snapshot of the vessel) to its start.
    public static void End(string what, double totalMs, double waitedMs, Prediction? p)
    {
        if (totalMs < SlowMs)
        {
            return;
        }
        long now = Stopwatch.GetTimestamp();
        lock (Reported)
        {
            var (next, skipped) = Reported.GetValueOrDefault(what);
            if (now < next)
            {
                Reported[what] = (next, skipped + 1);
                return;
            }
            Reported[what] = (now + ReportInterval, 0);
            var text = new StringBuilder();
            text.Append($"[predict] slow {what}: {totalMs:F0} ms, started {waitedMs:F0} ms after its request");
            if (skipped > 0)
            {
                text.Append($" ({skipped} more slow since the last report)");
            }
            PredictTimings n = _native;
            text.Append($"\n[predict]   native Predict ×{_nativeCalls}: {n.Total:F1} ms");
            text.Append($"\n[predict]     planets integrated ahead: {n.Prolong:F1} ms in {n.ProlongPieces:N0} pieces " +
                        $"of 6 h, of which giving way to the game's calls {n.GiveWay:F1} ms");
            text.Append($"\n[predict]     vessel flow: {n.Flow:F1} ms in {n.Chunks:N0} chunks");
            text.Append($"\n[predict]     apsides: {n.Apsides:F1} ms, about up to {_apsisBodies} bodies");
            text.Append($"\n[predict]     thinning: {n.Thinning:F1} ms");
            double other = n.Total - n.Prolong - n.Flow - n.Apsides - n.Thinning;
            if (other >= 0.5)
            {
                text.Append($"\n[predict]     other native: {other:F1} ms");
            }
            if (_stages is { Count: > 0 } stages)
            {
                text.Append("\n[predict]   C#:");
                foreach (var (name, ms) in stages.OrderByDescending(s => s.Value))
                {
                    text.Append($"\n[predict]     {name}: {ms:F1} ms");
                }
            }
            if (p is { Times.Length: > 0 })
            {
                text.Append($"\n[predict]   result: {(p.StateTimes[^1] - p.StateTimes[0]) / 86400:F1} d, " +
                            $"{p.Steps:N0} steps, {p.StateTimes.Length:N0} points kept, {p.Times.Length:N0} drawn, " +
                            $"{p.Stretches.Length} stretches, {p.Markers.Count} markers" +
                            (p.Impact ? ", ends in an impact" : ""));
            }
            Log.Info(text.ToString());
        }
    }
}
