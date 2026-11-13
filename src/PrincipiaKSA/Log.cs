namespace PrincipiaKSA;

// Logs to PrincipiaKSA.log next to the mod's DLL, and to the terminal too
// with log2terminal in settings.json (Settings.Log2Terminal).
internal static class Log
{
    private static readonly string LogPath =
        Path.Combine(Path.GetDirectoryName(typeof(Log).Assembly.Location) ?? ".",
                     "PrincipiaKSA.log");

    private static readonly object Lock = new();

    static Log()
    {
        try { File.WriteAllText(LogPath, ""); } catch { }
    }

    private static readonly HashSet<string> Warned = new();

    public static void Info(string message) => Write("INFO", message);
    public static void Warn(string message) => Write("WARN", message);
    public static void Error(string message) => Write("ERROR", message);

    // A warning logged only the first time for its `what`: for code that runs
    // every frame or step. Any thread.
    public static void WarnOnce(string what, Exception e)
    {
        lock (Warned)
        {
            if (!Warned.Add(what))
            {
                return;
            }
        }
        Warn($"{what} failed (further failures not logged): {e}");
    }

    private static void Write(string level, string message)
    {
        string line = $"[{DateTime.Now:HH:mm:ss.fff}] [PrincipiaKSA] [{level}] {message}";
        if (Settings.Log2Terminal)
        {
            Console.WriteLine(line);
        }
        lock (Lock)
        {
            try { File.AppendAllText(LogPath, line + Environment.NewLine); } catch { }
        }
    }
}
