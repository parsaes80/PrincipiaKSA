namespace PrincipiaKSA;

// Integrates the ephemeris ahead of game time on a background thread, so
// that time warp doesn't stall frames.
internal sealed class Prolonger : IDisposable
{
    // Work in bounded chunks so that Dispose doesn't wait for a huge prolong.
    private const double ChunkSeconds = 30 * 86400;

    // How far ahead of game time the planets are kept integrated, so that
    // lines rarely wait for them: for KSA's 46 bodies a year takes ~2 s of
    // this thread's time and ~30 MB (0.6 MB per body-year). Lines reaching
    // further ask for more as they go (Predictor.AheadOfPlanets).
    public const double Horizon = 365.25 * 86400;

    private readonly PrincipiaSystem _system;
    private readonly Thread _thread;
    private readonly AutoResetEvent _wake = new(false);
    private double _target;  // s since J2000; read and written with Volatile/Interlocked.
    private volatile bool _stop;

    public Prolonger(PrincipiaSystem system)
    {
        _system = system;
        // Below the game's threads: what the game needs next it integrates
        // itself if this thread is behind (EnsureProlonged).
        _thread = new Thread(Run)
        {
            IsBackground = true, Name = "Principia prolong", Priority = ThreadPriority.BelowNormal,
        };
        _thread.Start();
    }

    // Asks the background thread to integrate up to `t`. Never blocks.
    public void Request(double t)
    {
        double current = Volatile.Read(ref _target);
        while (t > current)
        {
            double seen = Interlocked.CompareExchange(ref _target, t, current);
            if (seen == current)
            {
                _wake.Set();
                return;
            }
            current = seen;
        }
    }

    // Integrates up to `t` on this thread if the background one hasn't.
    public void EnsureProlonged(double t)
    {
        if (_system.Ephemeris.TimeRange.TMax < t)
        {
            _system.Ephemeris.Prolong(t);
        }
    }

    private void Run()
    {
        FrameProfiler.MarkBackgroundThread();
        while (!_stop)
        {
            _wake.WaitOne(250);
            try
            {
                double target = Volatile.Read(ref _target);
                double tMax = _system.Ephemeris.TimeRange.TMax;
                while (!_stop && tMax < target)
                {
                    using (FrameProfiler.ProlongAhead.Measure())
                    {
                        _system.Ephemeris.Prolong(Math.Min(target, tMax + ChunkSeconds));
                    }
                    tMax = _system.Ephemeris.TimeRange.TMax;
                }
            }
            catch (Exception e)
            {
                if (!_stop)
                {
                    _system.Fail($"background prolongation failed: {e.Message}");
                }
                return;
            }
        }
    }

    public void Dispose()
    {
        _stop = true;
        _wake.Set();
        _thread.Join();
        _wake.Dispose();
    }
}
