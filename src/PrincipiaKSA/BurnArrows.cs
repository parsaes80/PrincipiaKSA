using Brutal.ImGuiApi;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA;

// "Burn arrows follow the plotting frame": KSA's node arrows turn to the frame's prograde/normal/outward, their
// drags are converted to KSA's axes (the burn's Δv stays KSA's), and faint lines show KSA's own directions.

internal static class BurnAxes
{
    // A burn's axes (x prograde, y normal, z outward) to Cce: KSA's and the frame's at the burn, and both as drawn now.
    internal readonly record struct Axes(doubleQuat Ksa, doubleQuat Frame, doubleQuat KsaShown, doubleQuat FrameShown);

    public static Axes? Of(Burn burn)
    {
        if (!Settings.BurnArrowsInFrame || burn.ParentDepartureBurn || burn.Vehicle is not { } vehicle ||
            !KsaConics.Hidden(vehicle) || PrincipiaSystem.Current is not { Failed: false } system ||
            (Predictor.PlanFor(vehicle) ?? Predictor.PredictionFor(vehicle))?.Line is not { Stretches.Length: > 0 } line)
        {
            return null;
        }
        Orbit orbit = burn.Patch.Orbit;
        Ephemeris ephemeris = system.Ephemeris;
        double t = GameTime.ToSecondsSinceJ2000(burn.Time);
        if (orbit.Parent is not Astronomical parent || system.IndexOf(parent) is not int parentIndex ||
            !(t <= ephemeris.TimeRange.TMax))
        {
            return null;
        }
        // The frame the line is drawn in where the burn is: an encounter's stretch is drawn about its body.
        int stretch = line.StretchAt(t);
        FrameSpec frame = line.Stretches[stretch].Frame;
        FrameMapper now = line.Mappers()[stretch];
        StateVectors sv = orbit.GetStateVectorsAt(burn.Time);
        if (sv.GetVlf2ParentCce(in orbit) is not { } ksa)
        {
            return null;
        }
        // As KSA's axes, with the velocity seen in the frame in place of the one about the parent.
        doubleQuat cci2Cce = orbit.Parent.GetCci2Cce();
        double3 r = sv.PositionCci.Transform(cci2Cce), v = sv.VelocityCci.Transform(cci2Cce);
        QP p = ephemeris.State(parentIndex, t);
        double3 seen = frame.FrameVelocity(ephemeris, t, new QP(p.Q.ToDouble3() + r, p.P.ToDouble3() + v));
        if (StateVectors.GetVlf2GivenFrame(r.Normalized(), seen.Normalized()) is not { } axes)
        {
            return null;
        }
        doubleQuat Shown(doubleQuat q)
        {
            double3 x = frame.DirectionNow(ephemeris, t, double3.UnitX.Transform(q), now);
            double3 y = frame.DirectionNow(ephemeris, t, double3.UnitY.Transform(q), now);
            double3 z = frame.DirectionNow(ephemeris, t, double3.UnitZ.Transform(q), now);
            return doubleQuat.CreateFromRotationMatrix(new double4x4(x.X, x.Y, x.Z, 0, y.X, y.Y, y.Z, 0,
                                                                      z.X, z.Y, z.Z, 0, 0, 0, 0, 1));
        }
        return new Axes(ksa, axes, Shown(ksa), Shown(axes));
    }

    // A Δv along the frame's axes, in KSA's.
    public static double3 ToKsa(in Axes axes, double3 inFrame)
    {
        double3 cce = inFrame.Transform(axes.Frame);
        return new double3(double3.Dot(cce, double3.UnitX.Transform(axes.Ksa)),
                           double3.Dot(cce, double3.UnitY.Transform(axes.Ksa)),
                           double3.Dot(cce, double3.UnitZ.Transform(axes.Ksa)));
    }
}

// KSA turns a node's sphere and arrows by its axes at the burn (GetVlf2ParentCce): the frame's instead, as drawn now.
[HarmonyPatch(typeof(Burn), nameof(Burn.UpdateGizmos))]
internal static class BurnArrowsPatch
{
    [ThreadStatic] private static Burn? _burn;

    [HarmonyPrefix]
    private static void Prefix(Burn __instance) => _burn = __instance;

    [HarmonyFinalizer]
    private static void Finalizer() => _burn = null;

    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions) =>
        Patching.ReplaceCalls(instructions, "[burn arrows] Burn.UpdateGizmos no longer reads GetVlf2ParentCce; " +
                              "node arrows stay in KSA's axes", 1,
            (AccessTools.Method(typeof(StateVectors), nameof(StateVectors.GetVlf2ParentCce)),
             AccessTools.Method(typeof(BurnArrowsPatch), nameof(Vlf2ParentCce))));

    private static doubleQuat? Vlf2ParentCce(in StateVectors sv, in Orbit orbit)
    {
        try
        {
            if (_burn is { } burn && BurnAxes.Of(burn) is { } axes)
            {
                return axes.FrameShown;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] turning a node's arrows", e);
        }
        return sv.GetVlf2ParentCce(in orbit);
    }
}

// A drag on arrow i adds along the frame's axis i, converted to KSA's axes (Burn.Update calls ApplyGizmoDeltaV).
[HarmonyPatch(typeof(Burn), nameof(Burn.Update), typeof(FlightComputer), typeof(IViewport), typeof(double))]
internal static class BurnArrowDragPatch
{
    private static readonly Action<Burn, double3> Apply =
        AccessTools.MethodDelegate<Action<Burn, double3>>(AccessTools.Method(typeof(Burn), "ApplyGizmoDeltaV"));

    [HarmonyTranspiler]
    private static IEnumerable<CodeInstruction> Transpiler(IEnumerable<CodeInstruction> instructions) =>
        Patching.ReplaceCalls(instructions, "[burn arrows] Burn.Update no longer calls ApplyGizmoDeltaV; " +
                              "arrow drags stay in KSA's axes", 3,
            (AccessTools.Method(typeof(Burn), "ApplyGizmoDeltaV"),
             AccessTools.Method(typeof(BurnArrowDragPatch), nameof(ApplyGizmoDeltaV))));

    private static void ApplyGizmoDeltaV(Burn burn, double3 offsetVlf)
    {
        try
        {
            if (BurnAxes.Of(burn) is { } axes)
            {
                offsetVlf = BurnAxes.ToKsa(axes, offsetVlf);
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] converting a drag", e);
        }
        Apply(burn, offsetVlf);
    }
}

// Faint lines along KSA's own prograde (red), normal (green) and outward (blue) at the active node.
[HarmonyPatch(typeof(Universe), nameof(Universe.OnDrawUi))]
internal static class KsaBurnDirections
{
    private static readonly byte4[] Colors = [new(255, 70, 70, 110), new(70, 255, 70, 110), new(90, 90, 255, 110)];

    private static readonly AccessTools.FieldRef<Burn, GenericGizmo> Cone =
        AccessTools.FieldRefAccess<Burn, GenericGizmo>("ConeGizmo");

    [HarmonyPostfix]
    private static void Postfix(IGameViewport inViewport)
    {
        try
        {
            Draw(inViewport);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[burn arrows] drawing KSA's directions", e);
        }
    }

    private static void Draw(IGameViewport viewport)
    {
        if (!Settings.BurnArrowsInFrame || viewport.Mode != CameraMode.Map || Program.ControlledVehicle is not { } vehicle)
        {
            return;
        }
        Camera camera = viewport.GetCamera();
        ImDrawListPtr drawList = ImGuiHelper.GetOverlayDrawList(viewport);
        float2 origin = viewport.Position;
        foreach (Burn burn in vehicle.BurnPlan.All())
        {
            if (!burn.IsGizmoActive || BurnAxes.Of(burn) is not { } axes)
            {
                continue;
            }
            double3 at = burn.Patch.Orbit.Parent.GetPositionEclFromCce(BurnNode.PositionCce(burn));
            // As far out as KSA's cones (Burn.UpdateGizmos), and a little beyond.
            double reach = 3 * Cone(burn).Mesh.BoundingSphereRadius * 0.03 * camera.EclToEgo(at).Length();
            double3[] directions =
            [
                double3.UnitX.Transform(axes.KsaShown),
                double3.UnitY.Transform(axes.KsaShown),
                double3.UnitZ.Transform(axes.KsaShown),
            ];
            for (int i = 0; i < directions.Length; i++)
            {
                float2 a = camera.EclToScreen(at - directions[i] * reach);
                float2 b = camera.EclToScreen(at + directions[i] * reach);
                if (float.IsNaN(a.X) || float.IsNaN(a.Y) || float.IsNaN(b.X) || float.IsNaN(b.Y))
                {
                    continue;
                }
                drawList.AddLine(origin + a, origin + b, Colors[i]);
            }
        }
    }
}
