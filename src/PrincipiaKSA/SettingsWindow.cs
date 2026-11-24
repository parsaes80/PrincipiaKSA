using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// Drawn as KSA draws its console windows (Ground Track, Target Track...), with its ConsoleStyle chrome and widgets.
internal static class SettingsWindow
{
    // At interface scale 1.
    private static readonly float2 InitialSize = new(460, 640);

    // "Reset" chosen: back to the initial place and size, next frame.
    private static bool _reset;

    public static void Draw()
    {
        if (!Settings.WindowEnabled)
        {
            return;  // Back from StarMap's Mods menu.
        }
        float scale = GameSettings.GetInterfaceScale();
        ImGuiViewportPtr viewport = ImGui.GetMainViewport();
        ImGuiCond place = _reset ? ImGuiCond.Always : ImGuiCond.FirstUseEver;
        _reset = false;
        ImGui.SetNextWindowPos(viewport.WorkPos + new float2(40, 80) * scale, place, (float2?)null);
        ImGuiWindowFlags flags = Settings.WindowLocked ? ImGuiWindowFlags.NoMove | ImGuiWindowFlags.NoResize
                                                       : ImGuiWindowFlags.None;
        // As KSA's windows: on top of the game when detached, if chosen.
        ImGuiWindow.SetNextTopMost(Settings.WindowDetached, Settings.WindowAlwaysOnTop);
        // No close button, as KSA's panels: hidden with "Enabled" instead.
        if (!ConsoleStyle.BeginWindow("PrincipiaKSA", "PrincipiaKSA", "PKSA", InitialSize * scale, flags, place,
                                      pinToMainViewport: !Settings.WindowDetached))
        {
            return;  // Collapsed or hidden: ImGui's End is already called.
        }
        ConsoleStyle.BeginBody();
        ConsoleStyle.PushWidgetStyle();
        WindowMenu("##PrincipiaKSA body menu");
        PrincipiaSystem? system = PrincipiaSystem.Current;
        DrawStatus(system);
        if (system != null && !system.Failed)
        {
            DrawPrediction();
            DrawFrame(system);
            DrawFlightPlan();
            DrawEvents();
            Section("Diagnostics");
            ImGui.Checkbox("Show KSA's patched conics", ref Settings.ShowKsaConics);
        }
        ConsoleStyle.PopWidgetStyle();
        ConsoleStyle.EndBody();
        // Right click on the title bar or footer too.
        ConsoleStyle.PushWidgetStyle();
        WindowMenu("##PrincipiaKSA window menu");
        ConsoleStyle.PopWidgetStyle();
        ConsoleStyle.BeginFooter();
        ConsoleStyle.EndFooter();
        ConsoleStyle.EndWindow();
    }

    // As KSA's panels' right-click menu (GaugeCanvas).
    private static void WindowMenu(string id)
    {
        if (!ImGui.BeginPopupContextWindow(id))
        {
            return;
        }
        if (ImGuiHelper.DrawMenuItem("Enabled"u8, Settings.WindowEnabled))
        {
            Settings.WindowEnabled = !Settings.WindowEnabled;
        }
        ImGui.Separator();
        // Only for a window of its own: within the game's, it is drawn with the rest.
        if (ImGuiHelper.DrawMenuItem("Always on top"u8, Settings.WindowAlwaysOnTop, !Settings.WindowDetached))
        {
            Settings.WindowAlwaysOnTop = !Settings.WindowAlwaysOnTop;
        }
        if (ImGuiHelper.DrawMenuItem("Reset"u8, isSelected: false))
        {
            _reset = true;
            Settings.WindowDetached = false;
        }
        if (Program.AllowMultipleWindows)
        {
            ImGuiHelper.DrawMenuItem("Detached"u8, ref Settings.WindowDetached);
        }
        ImGuiHelper.DrawMenuItem("Lock"u8, ref Settings.WindowLocked);
        ImGui.EndPopup();
    }

    private static void Section(string title)
    {
        ImGui.Spacing();
        ConsoleWidgets.RegionHeader(title);
    }

    private static void DrawStatus(PrincipiaSystem? system)
    {
        if (system == null)
        {
            ImGui.Text("Not running (no star system loaded).");
        }
        else if (system.Failed)
        {
            ImGui.TextColored(in ConsoleStyle.Danger, "Stopped: " + system.FailureReason);
        }
        else
        {
            UniverseTime now = Universe.GetElapsedTime();
            double tMax = system.Ephemeris.TimeRange.TMax;
            double ahead = (tMax - GameTime.ToSecondsSinceJ2000(now)) / 86400;
            ImGui.Text($"{system.Links.Count()} bodies, planets integrated to " +
                       $"{GameTime.ToDateTime(GameTime.FromSecondsSinceJ2000(tMax)):yyyy-MM-dd HH:mm} " +
                       $"({ahead:F1} days ahead)");
        }
    }

    private static void DrawPrediction()
    {
        Section("Prediction");
        ImGui.Checkbox("Show prediction (coasting)", ref Settings.ShowPrediction);
        Selector("Tolerance", "prediction tolerance", ref Settings.PredictionToleranceIndex,
                 Settings.PredictionTolerances.Length,
                 Settings.PredictionToleranceNames[Settings.PredictionToleranceIndex]);
        ImGui.SameLine(0, 24);
        Selector("Steps", "prediction steps", ref Settings.PredictionStepsIndex,
                 Settings.PredictionStepChoices.Length, $"{Settings.PredictionSteps:N0}");
        if (Settings.ShowPrediction)
        {
            Summary(Predictor.PredictionFor(Program.ControlledVehicle));
        }
        if (Predictor.LastError is { } error)
        {
            ImGui.TextColored(in ConsoleStyle.Danger, "Prediction failed: " + error);
        }
    }

    // As Principia's selectors: − and + step through a list.
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
                   : p.AwaitingPlanets ? ", growing as the planets are integrated ahead"
                   : p.End == PredictionEnd.ReachedEnd ? $", stops at {Predictor.MaxSpanYears:F0} years ahead"
                   : "";
        ImGui.TextDisabled($"{covered:F2} days in {p.Steps:N0} steps{end}, {p.ComputeMs:F0} ms");
        if (p.Note != null)
        {
            ImGui.TextColored(in ConsoleStyle.Pending, p.Note);
        }
    }

    private static void DrawFrame(PrincipiaSystem system)
    {
        Section("Plotting frame");
        int kind = (int)Settings.Frame;
        ImGui.RadioButton("Body Centred", ref kind, (int)PlotFrame.BodyCentred);
        ImGui.SameLine();
        ImGui.RadioButton("Body Surface", ref kind, (int)PlotFrame.BodySurface);
        ImGui.SameLine();
        ImGui.RadioButton("Two Body", ref kind, (int)PlotFrame.BarycentricRotating);
        Settings.Frame = (PlotFrame)kind;

        Prediction? p = Predictor.PredictionFor(Program.ControlledVehicle);
        BodyCombo(system, "Centre", ref Settings.CentreBody, "Automatic (vessel's parent)");
        if (Settings.Frame == PlotFrame.BarycentricRotating)
        {
            BodyCombo(system, "With", ref Settings.SecondaryBody, "Automatic (largest moon or parent)");
            // Off: about the pair's barycentre; on: about the centre body (Principia's body-direction frame).
            ImGui.Checkbox($"Centred on {p?.Frame.Centre.Id ?? "the centre body"}###centred",
                           ref Settings.TwoBodyCentred);
            ImGui.Checkbox("Show Lagrange points", ref Settings.ShowLagrangePoints);
        }
        if (p != null)
        {
            ImGui.TextDisabled($"Drawn {p.Frame.Describe()}");
        }
    }

    // Each star, its planets by distance, then "Asteroids & Dwarf Planets"; a body with moons opens them on hover.
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

    // Below Mercury (3.3e23 kg), above Pluto and Eris (~1.5e22 kg).
    private const double PlanetMass = 1e23;

    private static bool IsSmall(Astronomical body) => body is IParentBody { Mass: < PlanetMass };

    private static Astronomical? ParentIn(IReadOnlyList<Astronomical> bodies, Astronomical body) =>
        body is IOrbiter { Parent: Astronomical parent } && bodies.Contains(parent) ? parent : null;

    private static IEnumerable<Astronomical> Orbiting(IReadOnlyList<Astronomical> bodies, Astronomical parent) =>
        bodies.Where(b => ReferenceEquals(ParentIn(bodies, b), parent))
              .OrderBy(b => b is IOrbiter o ? o.Orbit.SemiMajorAxis : 0);

    // Each followed by its own.
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

    // Read-only: KSA's burn tool is the planner.
    private static void DrawFlightPlan()
    {
        Section("Flight plan");
        ImGui.Checkbox("Show flight plan", ref Settings.ShowFlightPlan);
        Selector("Tolerance", "plan tolerance", ref Settings.PlanToleranceIndex, Settings.PlanTolerances.Length,
                 Settings.PlanToleranceNames[Settings.PlanToleranceIndex]);
        ImGui.SameLine(0, 24);
        // Per coast: between two burns, and after the last.
        Selector("Steps", "plan steps", ref Settings.PlanStepsIndex, Settings.PlanStepChoices.Length,
                 $"{Settings.PlanSteps:N0}");
        if (Program.ControlledVehicle is not { } vehicle)
        {
            ImGui.TextDisabled("No vessel.");
            return;
        }
        BurnPlan plan = vehicle.BurnPlan;
        BurnTarget? target = vehicle.FlightComputer.Burn;
        double now = GameTime.Now;
        int shown = 0;
        foreach (Burn burn in plan.All())
        {
            if (!burn.HasDeltaV)
            {
                continue;
            }
            double3 dv = burn.DeltaVVlf;
            string components = $"prograde {dv.X:F1}, normal {dv.Y:F1}, outward {dv.Z:F1}";
            if (burn.ParentDepartureBurn)
            {
                // Planned on the parent's orbit, not yet the vessel's own burn.
                double departure = GameTime.ToSecondsSinceJ2000(burn.ParentDepartureTime);
                ImGui.Text($"Departure {++shown}: Δv {dv.Length():F1} m/s ({components}), " +
                           $"in {Format.Duration(departure - now)}");
                ImGui.TextDisabled("  Flown once made a vessel burn (\"Make vehicle transfer flightplan\")");
                continue;
            }
            double t = GameTime.ToSecondsSinceJ2000(burn.ImpulseTime);
            string state = target != null && target.ImpulsiveInstant == burn.ImpulseTime && target.StartedAt.HasValue
                ? $"burning, {target.DeltaVToGoCci.Length():F1} m/s to go"
                : $"in {Format.Duration(t - now)}";
            ImGui.Text($"Burn {++shown}: Δv {dv.Length():F1} m/s ({components}), {state}");
        }
        if (shown == 0)
        {
            ImGui.TextDisabled("No planned burns.");
        }
        else if (Settings.ShowFlightPlan)
        {
            Summary(Predictor.PlanFor(vehicle));
        }
        if (BurnTargetRefresher.Refreshes > 0)
        {
            ImGui.TextDisabled($"Burn target kept on the n-body trajectory " +
                               $"({BurnTargetRefresher.LastChangeDegrees:F3}° last correction)");
        }
    }

    private const int VisibleEvents = 10;

    private static void DrawEvents()
    {
        Section("Upcoming");
        double now = GameTime.Now;
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
        Add(Predictor.PredictionFor(Program.ControlledVehicle), "");
        Add(Predictor.PlanFor(Program.ControlledVehicle), "Plan: ");
        if (events.Count == 0)
        {
            ImGui.TextDisabled("Nothing ahead on the lines.");
            return;
        }
        // As tall as the events, up to VisibleEvents lines, then scrolling.
        float line = ImGui.GetTextLineHeightWithSpacing();
        float height = Math.Min(events.Count, VisibleEvents) * line + ImGui.GetStyle().WindowPadding.Y * 2;
        if (ImGui.BeginChild("##upcoming", new float2?(new float2(0, height)), ImGuiChildFlags.Borders,
                             ImGuiWindowFlags.None))
        {
            foreach (var (time, text) in events.OrderBy(e => e.Time))
            {
                ImGui.Text($"{text}  in {Format.Duration(time - now)}");
            }
        }
        ImGui.EndChild();
    }
}

// Once a frame (StarMap's [StarMapAfterGui] isn't called in this KSA).
[HarmonyPatch(typeof(Universe), nameof(Universe.OnDrawUi))]
internal static class WindowPatch
{
    [HarmonyPostfix]
    private static void Postfix(IGameViewport inViewport)
    {
        if (inViewport != Program.MainViewport)
        {
            return;
        }
        using var profiled = FrameProfiler.Window.Measure();
        try
        {
            SettingsWindow.Draw();
        }
        catch (Exception e)
        {
            Log.WarnOnce("[window] drawing the PrincipiaKSA window", e);
        }
    }
}
