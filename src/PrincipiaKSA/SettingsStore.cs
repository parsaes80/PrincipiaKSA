using System.Diagnostics;
using System.Text.Json;

namespace PrincipiaKSA;

// The window's settings, kept in settings.json next to the mod's DLL: saved
// a couple of seconds after a change, and at exit. The lines' switches and
// lengths are not kept: every launch starts from Settings' values.
internal static class SettingsStore
{
    private sealed class Data
    {
        public string Frame { get; set; } = Settings.Frame.ToString();
        public string? CentreBody { get; set; } = Settings.CentreBody;
        public string? SecondaryBody { get; set; } = Settings.SecondaryBody;
        public bool StartupChecks { get; set; } = Settings.StartupChecks;
    }

    private static readonly JsonSerializerOptions Options = new() { WriteIndented = true };
    private static readonly long Delay = Stopwatch.Frequency * 2;
    private static string? _saved;
    private static long _dueAt;

    private static string FilePath =>
        Path.Combine(Path.GetDirectoryName(typeof(SettingsStore).Assembly.Location) ?? ".", "settings.json");

    public static void Load()
    {
        try
        {
            if (!File.Exists(FilePath))
            {
                _saved = JsonSerializer.Serialize(new Data(), Options);
                return;
            }
            string json = File.ReadAllText(FilePath);
            Data d = JsonSerializer.Deserialize<Data>(json) ?? new Data();
            Settings.Frame = Enum.TryParse(d.Frame, out PlotFrame frame) ? frame : PlotFrame.BodyCentred;
            Settings.CentreBody = d.CentreBody;
            Settings.SecondaryBody = d.SecondaryBody;
            Settings.StartupChecks = d.StartupChecks;
            _saved = json;  // A file with keys no longer kept is rewritten without them.
            Log.Info($"Settings loaded from {FilePath}");
        }
        catch (Exception e)
        {
            Log.Warn($"Settings not loaded ({e.Message}); using defaults");
        }
    }

    // Main thread, every frame: write the file a little after a change.
    public static void Tick()
    {
        string current = JsonSerializer.Serialize(new Data(), Options);
        if (current == _saved)
        {
            _dueAt = 0;
            return;
        }
        long now = Stopwatch.GetTimestamp();
        if (_dueAt == 0)
        {
            _dueAt = now + Delay;
        }
        else if (now >= _dueAt)
        {
            Write(current);
        }
    }

    public static void Save() => Write(JsonSerializer.Serialize(new Data(), Options));

    private static void Write(string json)
    {
        try
        {
            File.WriteAllText(FilePath, json);
            _saved = json;
            _dueAt = 0;
        }
        catch (Exception e)
        {
            Log.Warn($"Settings not saved: {e.Message}");
            _saved = json;  // Don't retry every frame.
        }
    }
}
