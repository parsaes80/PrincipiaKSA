using System.Linq.Expressions;
using System.Reflection;
using Brutal.Numerics;
using HarmonyLib;
using KSA;

namespace PrincipiaKSA.Patches;

// A coasting vessel alone in its bubble, about to move along its conic.
[HarmonyPatch(typeof(PhysicsBubble), "ApplySingleFreefallMotion")]
internal static class ApplySingleFreefallMotionPatch
{
    [HarmonyPrefix]
    private static void Prefix(UniverseTime nextTime, VehicleUpdateState vehicleState)
    {
        try
        {
            VesselDriver.BeforeFreefall(vehicleState, nextTime);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[vessels] n-body coasting step", e);
        }
    }
}

// A coasting vessel inside a full-physics step, where KSA moves it along its
// conic without the hook above: whenever a step would pass its verified
// conics, an atmosphere or an impact (often, at high warp), or it shares a
// bubble.
[HarmonyPatch(typeof(PhysicsBubble), "RunUnconstrainedVehicleStep")]
internal static class RunUnconstrainedVehicleStepPatch
{
    private static readonly AccessTools.FieldRef<PhysicsBubble, List<VehicleUpdateState>> VehicleStates =
        AccessTools.FieldRefAccess<PhysicsBubble, List<VehicleUpdateState>>("_vehicleStates");

    private static readonly Func<VehicleUpdateState, bool> IsLadderParked =
        AccessTools.MethodDelegate<Func<VehicleUpdateState, bool>>(
            AccessTools.Method(typeof(PhysicsBubble), "IsLadderParked"));

    // The job is a private struct, readable only boxed (__args).
    private static readonly Type JobType = AccessTools.Inner(typeof(PhysicsBubble), "UnconstrainedVehicleStepJob");
    private static readonly Func<object, SimStep> SubStep = JobField<SimStep>("SubStep");
    private static readonly Func<object, int> VehicleIdx = JobField<int>("VehicleIdx");

    private static Func<object, T> JobField<T>(string name)
    {
        ParameterExpression job = Expression.Parameter(typeof(object));
        return Expression.Lambda<Func<object, T>>(Expression.Field(Expression.Unbox(job, JobType), name), job)
                         .Compile();
    }

    [HarmonyPrefix]
    private static void Prefix(PhysicsBubble __instance, object[] __args)
    {
        try
        {
            SimStep subStep = SubStep(__args[0]);
            int index = VehicleIdx(__args[0]);
            VehicleUpdateState vehicleState = VehicleStates(__instance)[index];
            if (vehicleState.GetNewStates().Props.Situation != Situation.Freefall || IsLadderParked(vehicleState))
            {
                return;
            }
            Interlocked.Increment(ref VesselDriver.FullPhysicsFlows);
            VesselDriver.BeforeFreefall(vehicleState, subStep.NextTime);
        }
        catch (Exception e)
        {
            Log.WarnOnce("[vessels] n-body coasting step in full physics", e);
        }
    }
}

// Forces on a vessel under full physics: the rest of the n-body field on top
// of KSA's point-mass gravity. Not on ground or water: KSA's terrain is a
// sphere, so J2's sideways pull would make parked vessels creep.
[HarmonyPatch]
internal static class ComputeDerivativesPatch
{
    private static MethodBase TargetMethod() =>
        AccessTools.GetDeclaredMethods(typeof(PhysicsStates))
                   .Single(m => m.Name == nameof(PhysicsStates.ComputeDerivatives) && m.IsStatic);

    [HarmonyPostfix]
    private static void Postfix(in BubbleOrigin origin,
                                in KinematicStates kinematic,
                                in VehicleProperties props,
                                in PhysicsEnvironment environment,
                                double3 positionOriginBub,
                                ref Disturbances __result)
    {
        try
        {
            if (props.Situation.HasAnyContact() || environment.ClosestParent != origin.Parent)
            {
                return;
            }
            if (VesselDriver.Correction(origin.Parent, origin.BubFrame == BubbleFrame.Ccf,
                                        positionOriginBub + kinematic.PositionPhys, origin.Time) is { } a)
            {
                __result.AddAccelPhys(a);
            }
        }
        catch (Exception e)
        {
            Log.WarnOnce("[vessels] gravity correction", e);
        }
    }
}
