using Brutal.ImGuiApi;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// The PrincipiaKSA window: status, prediction, plotting frame, flight plan
// and upcoming events.
internal static class SettingsWindow
{
    private static bool _logged;

    public static void Draw()
    {
        if (!_logged)
        {
            _logged = true;
            Log.Info("PrincipiaKSA window is being drawn");
        }
        ImGui.SetNextWindowSize(new float2(400, 0), ImGuiCond.FirstUseEver);
        if (ImGui.Begin("PrincipiaKSA"))
        {
            PrincipiaSystem? system = PrincipiaSystem.Current;
            DrawStatus(system);
            if (system != null && !system.Failed)
            {
                DrawPrediction();
                DrawFrame(system);
                DrawFlightPlan(system);
                DrawEvents();
                DrawDiagnostics();
            }
        }
        ImGui.End();
    }

    private static void DrawStatus(PrincipiaSystem? system)
    {
        if (system == null)
        {
            ImGui.Text("Not running (no star system loaded).");
        }
        else if (system.Failed)
        {
            ImGui.Text("Stopped: " + system.FailureReason);
        }
        else
        {
            double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
            double ahead = (system.Ephemeris.TimeRange.TMax - now) / 86400;
            ImGui.Text($"n-body gravity: {system.Links.Count()} bodies, integrated {ahead:F1} days ahead");
        }
    }

    private static void DrawPrediction()
    {
        if (!ImGui.CollapsingHeader("Prediction", ImGuiTreeNodeFlags.DefaultOpen))
        {
            return;
        }
        ImGui.Checkbox("Show prediction (coasting)", ref Settings.ShowPrediction);
        ImGui.SliderFloat("Length (days)##prediction", ref Settings.PredictionDays, 0.05f, Settings.MaxDays, "%.2f",
                          ImGuiSliderFlags.Logarithmic);
        if (Settings.ShowPrediction)
        {
            Summary(Predictor.Latest);
        }
        if (Predictor.LastError is { } error)
        {
            ImGui.Text("Prediction failed: " + error);
        }
    }

    private static void Summary(Prediction? p)
    {
        if (p == null || p.Times.Length < 2)
        {
            return;
        }
        double covered = (p.Times[^1] - p.Times[0]) / 86400;
        string end = p.Impact ? "ends in an impact" : p.End switch
        {
            PredictionEnd.ReachedEnd => "full length",
            PredictionEnd.MaxSteps => $"stopped early: needs over {Settings.MaxSteps:N0} steps, shorten it",
            _ => "stopped",
        };
        ImGui.Text($"{covered:F2} days computed ({end}), {p.ComputeMs:F0} ms");
        if (p.Note != null)
        {
            ImGui.Text(p.Note);
        }
    }

    private static void DrawFrame(PrincipiaSystem system)
    {
        if (!ImGui.CollapsingHeader("Plotting frame", ImGuiTreeNodeFlags.DefaultOpen))
        {
            return;
        }
        int kind = (int)Settings.Frame;
        ImGui.RadioButton("Body-centred", ref kind, (int)PlotFrame.BodyCentred);
        ImGui.SameLine();
        ImGui.RadioButton("Body surface", ref kind, (int)PlotFrame.BodySurface);
        ImGui.SameLine();
        ImGui.RadioButton("Two-body rotating", ref kind, (int)PlotFrame.BarycentricRotating);
        Settings.Frame = (PlotFrame)kind;

        BodyCombo(system, "Centre", ref Settings.CentreBody, "Automatic (vessel's parent)");
        if (Settings.Frame == PlotFrame.BarycentricRotating)
        {
            BodyCombo(system, "With", ref Settings.SecondaryBody, "Automatic (largest moon or parent)");
        }
        if (Predictor.Latest is { } p)
        {
            ImGui.Text($"Drawn {p.Frame.Describe()}");
        }
    }

    private static void BodyCombo(PrincipiaSystem system, string label, ref string? selection, string automatic)
    {
        if (ImGui.BeginCombo(label, selection ?? automatic))
        {
            if (ImGui.Selectable(automatic, selection == null))
            {
                selection = null;
            }
            foreach (Astronomical body in system.Bodies)
            {
                if (ImGui.Selectable(body.Id, selection == body.Id))
                {
                    selection = body.Id;
                }
            }
            ImGui.EndCombo();
        }
    }

    // KSA's planned burns, read-only: KSA's burn tool is the planner.
    private static void DrawFlightPlan(PrincipiaSystem system)
    {
        if (!ImGui.CollapsingHeader("Flight plan", ImGuiTreeNodeFlags.DefaultOpen))
        {
            return;
        }
        ImGui.Checkbox("Show flight plan", ref Settings.ShowFlightPlan);
        ImGui.SliderFloat("Length after last burn (days)", ref Settings.FlightPlanDays, 0.05f, Settings.MaxDays,
                          "%.2f", ImGuiSliderFlags.Logarithmic);
        if (Program.ControlledVehicle is not { } vehicle)
        {
            ImGui.Text("No vessel.");
            return;
        }
        BurnPlan plan = vehicle.BurnPlan;
        BurnTarget? target = vehicle.FlightComputer.Burn;
        double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
        int shown = 0;
        for (int i = 0; i < plan.BurnCount; i++)
        {
            if (!plan.TryGetBurn(i, out Burn? burn) || burn == null || !burn.HasDeltaV)
            {
                continue;
            }
            double3 dv = burn.DeltaVVlf;
            string components = $"prograde {dv.X:F1}, normal {dv.Y:F1}, outward {dv.Z:F1}";
            if (burn.ParentDepartureBurn)
            {
                // The vessel's burn KSA computed for the departure planned on
                // its parent's orbit, not yet one of its own.
                double departure = GameTime.ToSecondsSinceJ2000(burn.ParentDepartureTime);
                ImGui.Text($"Departure {++shown}: Δv {dv.Length():F1} m/s ({components}), " +
                           $"in {MapOverlay.Duration(departure - now)}");
                ImGui.TextDisabled("  Flown once made a vessel burn (\"Make vehicle transfer flightplan\")");
                continue;
            }
            double t = GameTime.ToSecondsSinceJ2000(burn.ImpulseTime);
            string state = target != null && target.ImpulsiveInstant == burn.ImpulseTime && target.StartedAt.HasValue
                ? $"burning, {target.DeltaVToGoCci.Length():F1} m/s to go"
                : $"in {MapOverlay.Duration(t - now)}";
            ImGui.Text($"Burn {++shown}: Δv {dv.Length():F1} m/s ({components}), {state}");
        }
        if (shown == 0)
        {
            ImGui.Text("No planned burns.");
        }
        else if (Settings.ShowFlightPlan)
        {
            Summary(Predictor.LatestPlan);
        }
        if (BurnTargetRefresher.Refreshes > 0)
        {
            ImGui.Text($"Burn target kept on the n-body trajectory ({BurnTargetRefresher.LastChangeDegrees:F3}° " +
                       "last correction)");
        }
    }

    private static void DrawDiagnostics()
    {
        if (!ImGui.CollapsingHeader("Diagnostics"))
        {
            return;
        }
        ImGui.Checkbox("Show KSA's patched conics", ref Settings.ShowKsaConics);
        ImGui.Checkbox("Check KSA's frames at the next launch", ref Settings.StartupChecks);
        ImGui.Text("Logs to mods/PrincipiaKSA/PrincipiaKSA.log");
    }

    private static void DrawEvents()
    {
        if (!ImGui.CollapsingHeader("Upcoming", ImGuiTreeNodeFlags.DefaultOpen))
        {
            return;
        }
        double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
        var events = new List<(double Time, string Text)>();
        void Add(Prediction? p, string prefix)
        {
            if (p == null)
            {
                return;
            }
            foreach (var m in p.Markers)
            {
                if (m.Time >= now && m.Listed)
                {
                    events.Add((m.Time, prefix + m.Label));
                }
            }
            if (p.Impact && p.Times.Length > 0)
            {
                events.Add((p.Times[^1], prefix + "Impact"));
            }
        }
        if (Settings.ShowPrediction) Add(Predictor.Latest, "");
        if (Settings.ShowFlightPlan) Add(Predictor.LatestPlan, "Plan: ");
        foreach (var (time, text) in events.OrderBy(e => e.Time).Take(12))
        {
            ImGui.Text($"{text}  in {MapOverlay.Duration(time - now)}");
        }
    }
}
