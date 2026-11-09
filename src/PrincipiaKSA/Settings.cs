namespace PrincipiaKSA;

// How predictions are drawn, as in Principia's plotting frames.
internal enum PlotFrame
{
    BodyCentred,         // Non-rotating, centred on a body (like KSA's own orbit lines).
    BodySurface,         // Rotating with a body: ground tracks, landing sites.
    BarycentricRotating, // Rotating with two bodies: L-points stand still.
}

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
}
