using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

// KSA's patched-conic lines and markers for the controlled vessel, hidden by
// default while Principia's lines replace them.
internal static class KsaConics
{
    // Whether Principia's lines replace KSA's conics for this vessel: drawing,
    // hover and node drag then never use the conics. Not while Principia isn't
    // driving (multiplayer), when the conics are the vessel's path.
    public static bool Hidden(Vehicle? vehicle) =>
        !Settings.ShowKsaConics && vehicle != null && vehicle == Program.ControlledVehicle &&
        (Settings.ShowPrediction || Settings.ShowFlightPlan) && PrincipiaSystem.Current is { Driving: true };

    // The vessel's flight plan, or one of its burns' post-burn flight plans.
    public static bool Hidden(FlightPlan plan)
    {
        Vehicle? vehicle = Program.ControlledVehicle;
        if (!Hidden(vehicle))
        {
            return false;
        }
        if (plan == vehicle!.FlightPlan)
        {
            return true;
        }
        BurnPlan burns = vehicle.BurnPlan;
        for (int i = 0; i < burns.BurnCount; i++)
        {
            if (burns.TryGetBurn(i, out Burn? burn) && burn?.FlightPlan == plan)
            {
                return true;
            }
        }
        return false;
    }
}

// The orbit lines of the vessel's and its burns' flight plans. (Not
// Vehicle.AddLineInstances, which gets inlined.)
[HarmonyPatch(typeof(FlightPlan), nameof(FlightPlan.AddLineInstances))]
internal static class FlightPlanLinesPatch
{
    [HarmonyPrefix]
    private static bool Prefix(IOrbiter orbiter)
    {
        try
        {
            return !KsaConics.Hidden(orbiter as Vehicle);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[conics] hiding KSA's orbit lines", e);
            return true;
        }
    }
}

// The markers on those lines.
[HarmonyPatch(typeof(FlightPlan), nameof(FlightPlan.DrawUi))]
internal static class FlightPlanMarkersPatch
{
    [HarmonyPrefix]
    private static bool Prefix(FlightPlan __instance, ref bool __result)
    {
        try
        {
            if (KsaConics.Hidden(__instance))
            {
                __result = false;  // Nothing hovered.
                return false;
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[conics] hiding KSA's orbit markers", e);
        }
        return true;
    }
}
