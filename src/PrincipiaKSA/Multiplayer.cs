using KSA;

namespace PrincipiaKSA;

// While online, n-body driving pauses (the other players follow KSA's
// conics); back offline, the bodies move to Principia's states again.
internal static class Multiplayer
{
    private static bool? _wasOnline;

    // Main thread, once per frame.
    public static void Tick()
    {
        bool online = PrincipiaSystem.Online;
        if (_wasOnline == online)
        {
            return;
        }
        if (_wasOnline != null || online)
        {
            Log.Info(online ? "Multiplayer session: n-body driving paused" : "Offline: n-body driving resumed");
        }
        bool resumed = _wasOnline == true;
        _wasOnline = online;
        if (resumed)
        {
            CelestialDriver.JumpTo(Universe.GetElapsedTime());
        }
    }
}
