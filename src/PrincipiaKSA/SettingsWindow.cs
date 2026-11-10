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
            // How far ahead the lines reach now, as they change (not the
            // planets' ephemeris, which only grows).
            double now = GameTime.ToSecondsSinceJ2000(Universe.GetElapsedTime());
            var reach = new List<string>();
            void Reach(Prediction? p, bool shown, string what)
            {
                if (shown && p is { Times.Length: > 0 } && p.Vehicle == Program.ControlledVehicle)
                {
                    reach.Add($"{Math.Max(0, p.Times[^1] - now) / 86400:F1} d ({what})");
                }
            }
            Reach(Predictor.Latest, Settings.ShowPrediction, "prediction");
            Reach(Predictor.LatestPlan, Settings.ShowFlightPlan, "flight plan");
            ImGui.Text($"n-body gravity: {system.Links.Count()} bodies" +
                       (reach.Count > 0 ? $", lines reach {string.Join(" / ", reach)}" : ""));
        }
    }

    private static void DrawPrediction()
    {
        if (!ImGui.CollapsingHeader("Prediction", ImGuiTreeNodeFlags.DefaultOpen))
        {
            return;
        }
        ImGui.Checkbox("Show prediction (coasting)", ref Settings.ShowPrediction);
        Selector("Tolerance", "prediction tolerance", ref Settings.PredictionToleranceIndex,
                 Settings.PredictionTolerances.Length,
                 Settings.PredictionToleranceNames[Settings.PredictionToleranceIndex]);
        ImGui.SameLine(0, 24);
        Selector("Steps", "prediction steps", ref Settings.PredictionStepsIndex,
                 Settings.PredictionStepChoices.Length, $"{Settings.PredictionSteps:N0}");
        if (Settings.ShowPrediction)
        {
            Summary(Predictor.Latest);
        }
        if (Predictor.LastError is { } error)
        {
            ImGui.Text("Prediction failed: " + error);
        }
    }

    // A choice from a list, as Principia's selectors: − and + step through it.
    private static void Selector(string label, string id, ref int index, int count, string value)
    {
        ImGui.Text(label);
        ImGui.SameLine();
        if (ImGui.Button($"-##{id}", (float2?)null) && index > 0)
        {
            index--;
        }
        ImGui.SameLine();
        if (ImGui.Button($"+##{id}", (float2?)null) && index < count - 1)
        {
            index++;
        }
        ImGui.SameLine();
        ImGui.Text(value);
    }

    private static void Summary(Prediction? p)
    {
        if (p == null || p.Times.Length < 2)
        {
            return;
        }
        double covered = (p.Times[^1] - p.Times[0]) / 86400;
        // Steps run out: the usual end.
        string end = p.Impact ? ", ends in an impact"
                   : p.End == PredictionEnd.ReachedEnd ? ", stops at ten years ahead"
                   : "";
        ImGui.Text($"{covered:F2} days in {p.Steps:N0} steps{end}, {p.ComputeMs:F0} ms");
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

    // A body to plot about, as KSA's system holds them: each star, then its
    // planets by distance, then "Asteroids & Dwarf Planets" by distance. A
    // body with moons is a menu item of its own: clicked, it is picked;
    // hovered, it opens its moons by distance.
    private static void BodyCombo(PrincipiaSystem system, string label, ref string? selection, string automatic)
    {
        if (!ImGui.BeginCombo(label, selection ?? automatic))
        {
            return;
        }
        if (ImGui.Selectable(automatic, selection == null))
        {
            selection = null;
        }
        IReadOnlyList<Astronomical> bodies = system.Bodies;
        string? current = selection, picked = null;
        bool close = false;
        void Pick(Astronomical body)
        {
            if (ImGui.Selectable(body.Id, current == body.Id))
            {
                picked = body.Id;
            }
        }
        // A body, opening its moons on hover if it has any.
        void Item(Astronomical body)
        {
            List<Astronomical> moons = Moons(bodies, body);
            if (moons.Count == 0)
            {
                Pick(body);
                return;
            }
            bool open = ImGui.BeginMenu(body.Id);
            if (ImGui.IsItemClicked())
            {
                picked = body.Id;
                close = true;  // A menu item doesn't close the drop-down by itself.
            }
            if (open)
            {
                foreach (Astronomical moon in moons)
                {
                    Pick(moon);
                }
                ImGui.EndMenu();
            }
        }
        foreach (Astronomical root in bodies.Where(b => ParentIn(bodies, b) == null))
        {
            ImGui.Separator();
            Pick(root);
            List<Astronomical> orbiting = Orbiting(bodies, root).ToList();
            foreach (Astronomical planet in orbiting.Where(b => !IsSmall(b)))
            {
                Item(planet);
            }
            List<Astronomical> small = orbiting.Where(IsSmall).ToList();
            if (small.Count > 0)
            {
                ImGui.Separator();
                if (ImGui.BeginMenu("Asteroids & Dwarf Planets"))
                {
                    foreach (Astronomical body in small)
                    {
                        Item(body);
                    }
                    ImGui.EndMenu();
                }
            }
        }
        if (picked != null)
        {
            selection = picked;
        }
        if (close)
        {
            ImGui.CloseCurrentPopup();
        }
        ImGui.EndCombo();
    }

    // Lighter than any planet (Mercury: 3.3e23 kg): a dwarf planet or an
    // asteroid (Pluto, Eris: ~1.5e22 kg; Ceres: 9.4e20 kg).
    private const double PlanetMass = 1e23;

    private static bool IsSmall(Astronomical body) => body is IParentBody { Mass: < PlanetMass };

    // The body `body` orbits, if Principia has it.
    private static Astronomical? ParentIn(IReadOnlyList<Astronomical> bodies, Astronomical body) =>
        body is IOrbiter { Parent: Astronomical parent } && bodies.Contains(parent) ? parent : null;

    // What orbits `parent`, nearest first.
    private static IEnumerable<Astronomical> Orbiting(IReadOnlyList<Astronomical> bodies, Astronomical parent) =>
        bodies.Where(b => ReferenceEquals(ParentIn(bodies, b), parent))
              .OrderBy(b => b is IOrbiter o ? o.Orbit.SemiMajorAxis : 0);

    // A planet's moons, nearest first, each followed by its own.
    private static List<Astronomical> Moons(IReadOnlyList<Astronomical> bodies, Astronomical planet)
    {
        var moons = new List<Astronomical>();
        foreach (Astronomical moon in Orbiting(bodies, planet))
        {
            moons.Add(moon);
            moons.AddRange(Moons(bodies, moon));
        }
        return moons;
    }

    // KSA's planned burns, read-only: KSA's burn tool is the planner.
    private static void DrawFlightPlan(PrincipiaSystem system)
    {
        if (!ImGui.CollapsingHeader("Flight plan", ImGuiTreeNodeFlags.DefaultOpen))
        {
            return;
        }
        ImGui.Checkbox("Show flight plan", ref Settings.ShowFlightPlan);
        Selector("Tolerance", "plan tolerance", ref Settings.PlanToleranceIndex, Settings.PlanTolerances.Length,
                 Settings.PlanToleranceNames[Settings.PlanToleranceIndex]);
        ImGui.SameLine(0, 24);
        // Per coast: between two burns, and after the last.
        Selector("Steps", "plan steps", ref Settings.PlanStepsIndex, Settings.PlanStepChoices.Length,
                 $"{Settings.PlanSteps:N0}");
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

    private const int VisibleEvents = 10;

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
        if (events.Count == 0)
        {
            return;
        }
        // Every event, in a box as tall as them up to VisibleEvents lines,
        // scrolling beyond.
        float line = ImGui.GetTextLineHeightWithSpacing();
        float height = Math.Min(events.Count, VisibleEvents) * line + ImGui.GetStyle().WindowPadding.Y * 2;
        if (ImGui.BeginChild("##upcoming", new float2?(new float2(0, height)), ImGuiChildFlags.Borders,
                             ImGuiWindowFlags.None))
        {
            foreach (var (time, text) in events.OrderBy(e => e.Time))
            {
                ImGui.Text($"{text}  in {MapOverlay.Duration(time - now)}");
            }
        }
        ImGui.EndChild();
    }
}
