using KSA;

namespace PrincipiaKSA;

// KSA's UniverseTime (Int128 ns from JD 2461009.5, the epoch of its Horizons
// elements) to Principia's time (double seconds from J2000, TT; TDB and TT
// differ by < 2 ms).
internal static class GameTime
{
    public const double J2000JulianDay = 2451545.0;
    public const double SolEpochJulianDay = 2461009.5;
    public const double SecondsPerDay = 86400.0;

    // 9464.5 days = 817,732,800 s.
    public const double EpochSecondsSinceJ2000 = (SolEpochJulianDay - J2000JulianDay) * SecondsPerDay;

    // KSA uses MaxValue and MinValue for "never": they map to ±∞.
    public static double ToSecondsSinceJ2000(UniverseTime t) =>
        t == UniverseTime.MaxValue ? double.PositiveInfinity
        : t == UniverseTime.MinValue ? double.NegativeInfinity
        : EpochSecondsSinceJ2000 + t.Seconds();

    public static double Seconds(UniverseTime from, UniverseTime to) =>
        ToSecondsSinceJ2000(to) - ToSecondsSinceJ2000(from);

    public static UniverseTime FromSecondsSinceJ2000(double seconds) =>
        UniverseTime.FromNanoseconds((Int128)Math.Round((seconds - EpochSecondsSinceJ2000) * 1e9));

    public static double ToJulianDay(UniverseTime t) =>
        J2000JulianDay + ToSecondsSinceJ2000(t) / SecondsPerDay;

    // Calendar date for logging, ignoring the ~69 s TT-UTC offset.
    public static DateTime ToDateTime(UniverseTime t) =>
        new DateTime(2000, 1, 1, 12, 0, 0, DateTimeKind.Utc).AddSeconds(ToSecondsSinceJ2000(t));
}
