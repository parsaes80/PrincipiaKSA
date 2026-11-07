namespace PrincipiaKSA;

// How predictions are drawn, as in Principia's plotting frames.
internal enum PlotFrame
{
    BodyCentred,         // Non-rotating, centred on a body (like KSA's own orbit lines).
    BodySurface,         // Rotating with a body: ground tracks, landing sites.
    BarycentricRotating, // Rotating with two bodies: L-points stand still.
}

// The PrincipiaKSA window's settings. The lines' switches and lengths start
// from these values at every launch; the rest are kept in settings.json.
internal static class Settings
{
    public const float MaxDays = 3652.5f;  // Ten years.

    // Guards against a runaway only: 10 years of low Earth orbit, the most
    // step-hungry there is, takes ~7.5 million steps.
    public const int MaxSteps = 10_000_000;

    // The prediction: the vessel coasting.
    public static bool ShowPrediction = true;
    public static float PredictionDays = 1f;

    // The flight plan: through KSA's burns; its length counts from the last.
    public static bool ShowFlightPlan = true;
    public static float FlightPlanDays = 1f;

    // KSA's own patched-conic lines and markers for the controlled vessel.
    public static bool ShowKsaConics = false;

    public static PlotFrame Frame = PlotFrame.BodyCentred;

    // At system load: list the bodies and check KSA's frames.
    public static bool StartupChecks = false;
    public static string? CentreBody;     // null: the vessel's parent.
    public static string? SecondaryBody;  // For BarycentricRotating; null: automatic.
}
