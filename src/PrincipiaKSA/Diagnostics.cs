using System.Diagnostics;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// A status line every 10 seconds: time and warp, how far ahead Principia is,
// the drivers' work, and how far a few bodies are from KSA's Kepler orbits.
internal static class Diagnostics
{
    private static readonly long Interval = Stopwatch.Frequency * 10;
    private static readonly string[] Watched = ["Luna", "Earth", "Io", "Charon"];
    private static long _next;
    private static long _lastStates, _lastRefreshes;
    private static long _lastFlows, _lastFullPhysicsFlows, _lastRebuilds, _lastResyncs, _lastCorrections, _lastFlowTicks;

    // Main-thread time in our per-frame work since the last line.
    private static long _prolongTicks, _prolongMaxTicks, _refreshTicks, _refreshMaxTicks, _frames;

    public static void RecordFrame(long prolongTicks, long refreshTicks)
    {
        _frames++;
        _prolongTicks += prolongTicks;
        _refreshTicks += refreshTicks;
        _prolongMaxTicks = Math.Max(_prolongMaxTicks, prolongTicks);
        _refreshMaxTicks = Math.Max(_refreshMaxTicks, refreshTicks);
    }

    private static double Ms(long ticks) => ticks * 1000.0 / Stopwatch.Frequency;

    public static void Tick()
    {
        long now = Stopwatch.GetTimestamp();
        if (now < _next)
        {
            return;
        }
        _next = now + Interval;
        PrincipiaSystem? system = PrincipiaSystem.Current;
        if (system == null)
        {
            return;
        }
        if (system.Failed)
        {
            Log.Warn($"[status] stopped: {system.FailureReason}");
            return;
        }
        try
        {
            UniverseTime time = Universe.GetElapsedTime();
            double t = GameTime.ToSecondsSinceJ2000(time);
            var (_, tMax) = system.Ephemeris.TimeRange;
            long states = Interlocked.Read(ref CelestialDriver.StatesServed);
            long refreshes = Interlocked.Read(ref CelestialDriver.OrbitsRefreshed);
            Log.Info($"[status] {GameTime.ToDateTime(time):yyyy-MM-dd HH:mm} (day {time.Seconds() / 86400:F2}), " +
                     $"warp ×{Universe.GetSimulationSpeed():G4}, Principia ahead by {(tMax - t) / 86400:F2} d, " +
                     $"{states - _lastStates} states served, {refreshes - _lastRefreshes} orbits rebuilt");
            _lastStates = states;
            _lastRefreshes = refreshes;
            if (_frames > 0)
            {
                Log.Info($"[status] main thread per frame ({_frames} frames): prolong avg " +
                         $"{Ms(_prolongTicks) / _frames:F3} ms max {Ms(_prolongMaxTicks):F2} ms, orbit rebuilds avg " +
                         $"{Ms(_refreshTicks) / _frames:F3} ms max {Ms(_refreshMaxTicks):F2} ms");
            }
            _frames = _prolongTicks = _prolongMaxTicks = _refreshTicks = _refreshMaxTicks = 0;

            long flows = Interlocked.Read(ref VesselDriver.Flows);
            long fullPhysicsFlows = Interlocked.Read(ref VesselDriver.FullPhysicsFlows);
            long rebuilds = Interlocked.Read(ref VesselDriver.Rebuilds);
            long resyncs = Interlocked.Read(ref VesselDriver.Resyncs);
            long corrections = Interlocked.Read(ref VesselDriver.Corrections);
            long flowTicks = Interlocked.Read(ref VesselDriver.FlowTicks);
            long dFlows = flows - _lastFlows;
            Log.Info($"[status] vessels: {dFlows} n-body coasting steps " +
                     $"(avg {(dFlows > 0 ? Ms(flowTicks - _lastFlowTicks) / dFlows : 0):F3} ms on workers; " +
                     $"{fullPhysicsFlows - _lastFullPhysicsFlows} within KSA's full-physics steps), " +
                     $"{rebuilds - _lastRebuilds} conics rebuilt, {resyncs - _lastResyncs} resyncs from KSA, " +
                     $"{corrections - _lastCorrections} physics gravity corrections");
            (_lastFlows, _lastFullPhysicsFlows, _lastRebuilds, _lastResyncs, _lastCorrections, _lastFlowTicks) =
                (flows, fullPhysicsFlows, rebuilds, resyncs, corrections, flowTicks);

            var drifts = new List<string>();
            foreach (string id in Watched)
            {
                Astronomical? body = system.Bodies.FirstOrDefault(b => b.Id == id);
                if (body == null || !system.TryGetLink(body, out var link) ||
                    !system.OriginalOrbits.TryGetValue(body, out Orbit? original))
                {
                    continue;
                }
                (double3 q, _) = CelestialDriver.RelativeEcl(system, link, t);
                double3 kepler = original.GetStateVectorsAt(time).PositionCci.Transform(link.Parent.GetCci2Cce());
                drifts.Add($"{id} {(q - kepler).Length() / 1000:N0} km");
            }
            Log.Info($"[status] n-body vs KSA's original Kepler orbits: {string.Join(", ", drifts)}");

            // The vessel's osculating elements (equatorial).
            if (Program.ControlledVehicle is { } vehicle)
            {
                Orbit o = vehicle.Orbit;
                const double deg = 180 / Math.PI;
                Log.Info($"[status] {vehicle.Id} around {(o.Parent as IObjectId)?.Id}: " +
                         $"{vehicle.Situation}, pe {o.PeriapsisAltitude / 1000:F1} km, " +
                         $"ap {o.ApoapsisAltitude / 1000:F1} km, i {o.Inclination * deg:F3}°, " +
                         $"Ω {o.LongitudeOfAscendingNode * deg:F3}°, ω {o.ArgumentOfPeriapsis * deg:F3}°");
            }
        }
        catch (Exception e)
        {
            Log.Warn($"[status] {e.Message}");
        }
    }
}
