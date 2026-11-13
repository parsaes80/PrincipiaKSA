using System.Diagnostics;
using System.Text.Json.Serialization;
using System.Text.Json;

namespace PrincipiaKSA;

// The mod's settings (Settings) and their file, settings.json (SettingsStore).

// The PrincipiaKSA window's settings. The lines' switches, steps and
// tolerances start from these values at every launch; the rest are kept in
// settings.json.
internal static class Settings
{
    // The nodes' approach tracks, which reach a given time: a guard against
    // a runaway only.
    public const long MaxSteps = 1L << 24;

    // The prediction (the vessel coasting) and the flight plan (through KSA's
    // burns) are integrated as Principia's: no time, but a tolerance and a
    // number of steps, from Principia's own choices (ksp_plugin_adapter,
    // main_window.cs and flight_planner.cs). The prediction runs until its
    // steps run out; the flight plan's coasts each have up to its steps, the
    // last (after the last burn) until they run out.
    public static readonly double[] PredictionTolerances = [1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4];
    public static readonly string[] PredictionToleranceNames =
        ["1 mm", "1 cm", "10 cm", "1 m", "10 m", "100 m", "1 km", "10 km"];
    public static readonly long[] PredictionStepChoices =
        [1L << 2, 1L << 4, 1L << 6, 1L << 8, 1L << 10, 1L << 12, 1L << 14, 1L << 16, 1L << 18, 1L << 20,
         1L << 22, 1L << 24];
    public static readonly double[] PlanTolerances =
        [1e-6, 1e-5, 1e-4, 1e-3, 1e-2, 1e-1, 1e0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6];
    public static readonly string[] PlanToleranceNames =
        ["1 µm", "10 µm", "100 µm", "1 mm", "1 cm", "10 cm", "1 m", "10 m", "100 m", "1 km", "10 km", "100 km",
         "1000 km"];
    public static readonly long[] PlanStepChoices =
        [1L << 6, 1L << 8, 1L << 10, 1L << 12, 1L << 14, 1L << 16, 1L << 18, 1L << 20];

    // The prediction: the vessel coasting. 1 m, 1,024 steps.
    public static bool ShowPrediction = true;
    public static int PredictionToleranceIndex = 3;
    public static int PredictionStepsIndex = 4;
    public static double PredictionTolerance => PredictionTolerances[PredictionToleranceIndex];
    public static long PredictionSteps => PredictionStepChoices[PredictionStepsIndex];

    // The flight plan: through KSA's burns. 1 m, 1,024 steps per coast.
    public static bool ShowFlightPlan = true;
    public static int PlanToleranceIndex = 6;
    public static int PlanStepsIndex = 2;
    public static double PlanTolerance => PlanTolerances[PlanToleranceIndex];
    public static long PlanSteps => PlanStepChoices[PlanStepsIndex];

    // KSA's own patched-conic lines and markers for the controlled vessel.
    public static bool ShowKsaConics = false;

    public static PlotFrame Frame = PlotFrame.BodyCentred;

    // At system load: list the bodies and check KSA's frames.
    public static bool StartupChecks = false;
    public static string? CentreBody;     // null: the vessel's parent.
    public static string? SecondaryBody;  // For BarycentricRotating; null: automatic.

    // The window, as KSA's panels (right click on it): shown (else listed
    // unchecked in KSA's HUD menu), always on top (when detached), locked
    // (no moving or resizing), detached (its own OS window).
    public static bool WindowEnabled = true;
    public static bool WindowAlwaysOnTop = true;
    public static bool WindowLocked = false;
    public static bool WindowDetached = false;

    // The mod's log (Principia's included) also to the terminal; it always
    // goes to PrincipiaKSA.log. Only in settings.json ("log2terminal").
    public static bool Log2Terminal = false;
}

// The window's settings, and log2terminal, kept in settings.json next to the
// mod's DLL: saved a couple of seconds after a change, and at exit (so a file
// without a key gets it, at its default). The lines' switches, steps and
// tolerances are not kept: every launch starts from Settings' values.
internal static class SettingsStore
{
    private sealed class Data
    {
        public string Frame { get; set; } = Settings.Frame.ToString();
        public string? CentreBody { get; set; } = Settings.CentreBody;
        public string? SecondaryBody { get; set; } = Settings.SecondaryBody;
        public bool StartupChecks { get; set; } = Settings.StartupChecks;
        public bool WindowEnabled { get; set; } = Settings.WindowEnabled;
        public bool WindowAlwaysOnTop { get; set; } = Settings.WindowAlwaysOnTop;
        public bool WindowLocked { get; set; } = Settings.WindowLocked;
        public bool WindowDetached { get; set; } = Settings.WindowDetached;
        [JsonPropertyName("log2terminal")]
        public bool Log2Terminal { get; set; } = Settings.Log2Terminal;
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
            Settings.WindowEnabled = d.WindowEnabled;
            Settings.WindowAlwaysOnTop = d.WindowAlwaysOnTop;
            Settings.WindowLocked = d.WindowLocked;
            Settings.WindowDetached = d.WindowDetached;
            Settings.Log2Terminal = d.Log2Terminal;
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
