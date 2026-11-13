using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// Plotting frames: the kinds (PlotFrame), the bodies one is attached to and
// coordinates in it (FrameSpec), and those coordinates back to Ecl positions
// now, for drawing (FrameMapper).

// How predictions are drawn, as in Principia's plotting frames.
internal enum PlotFrame
{
    BodyCentred,         // Non-rotating, centred on a body (like KSA's own orbit lines).
    BodySurface,         // Rotating with a body: ground tracks, landing sites.
    BarycentricRotating, // Rotating with two bodies: L-points stand still.
}

// The bodies a plotting frame is attached to.
internal sealed record FrameSpec(PlotFrame Kind, Astronomical Centre, int CentreIndex,
                                 Astronomical? Secondary, int SecondaryIndex)
{
    public string Describe() => Kind switch
    {
        PlotFrame.BodyCentred => $"centred on {Centre.Id}",
        PlotFrame.BodySurface => $"surface of {Centre.Id}",
        _ => $"rotating with {Centre.Id}–{Secondary?.Id}",
    };

    // The plotting frame chosen in the window; unset bodies follow the vessel.
    public static FrameSpec? FromSettings(PrincipiaSystem system, Astronomical vesselParent)
    {
        Astronomical centre = system.Find(Settings.CentreBody) ?? vesselParent;
        if (system.IndexOf(centre) is not int centreIndex)
        {
            return null;
        }
        if (Settings.Frame != PlotFrame.BarycentricRotating)
        {
            return new FrameSpec(Settings.Frame, centre, centreIndex, null, -1);
        }
        // Secondary: as chosen, else the centre's heaviest moon, else its parent.
        Astronomical? secondary = system.Find(Settings.SecondaryBody);
        if (secondary == null && centre is IParentBody centreBody)
        {
            secondary = centreBody.Children.OfType<Astronomical>()
                                  .Where(c => system.IndexOf(c) != null)
                                  .MaxBy(c => ((IParentBody)c).Mass);
        }
        if (secondary == null && centre is IOrbiter { Parent: Astronomical up })
        {
            secondary = up;
        }
        if (secondary == null || secondary == centre || system.IndexOf(secondary) is not int secondaryIndex)
        {
            return new FrameSpec(PlotFrame.BodyCentred, centre, centreIndex, null, -1);
        }
        return new FrameSpec(PlotFrame.BarycentricRotating, centre, centreIndex, secondary, secondaryIndex);
    }

    // An absolute state in this frame's coordinates (`centre`: the centre's
    // state at t), its velocity as seen in the frame (less Ω × c in a
    // rotating one): about the centre in Ecl axes, or in its rotating axes,
    // or about the pair's barycentre with x towards the secondary and z along
    // their orbital angular momentum.
    public (double3 Position, double3 Velocity) Coordinates(Ephemeris ephemeris, double t, QP state, QP centre)
    {
        FrameSpec frame = this;
        double3 q = state.Q.ToDouble3(), v = state.P.ToDouble3();
        double3 rel = q - centre.Q.ToDouble3(), relV = v - centre.P.ToDouble3();
        switch (frame.Kind)
        {
            case PlotFrame.BodySurface:
            {
                doubleQuat cce2Ccf = ((IParentBody)frame.Centre).GetCce2Ccf(GameTime.FromSecondsSinceJ2000(t));
                double3 c = rel.Transform(cce2Ccf);
                double omega = frame.Centre is Celestial celestial ? celestial.GetAngularVelocity() : 0;
                return (c, relV.Transform(cce2Ccf) - double3.Cross(new double3(0, 0, omega), c));
            }
            case PlotFrame.BarycentricRotating:
            {
                QP secondary = ephemeris.State(frame.SecondaryIndex, t);
                double mu1 = ((IParentBody)frame.Centre).Mu, mu2 = ((IParentBody)frame.Secondary!).Mu;
                double3 q1 = centre.Q.ToDouble3(), v1 = centre.P.ToDouble3();
                double3 q2 = secondary.Q.ToDouble3(), v2 = secondary.P.ToDouble3();
                (double3 origin, double3 x, double3 y, double3 z) = RotatingAxes(mu1, q1, v1, mu2, q2, v2);
                double3 vOrigin = (mu1 * v1 + mu2 * v2) / (mu1 + mu2);
                double3 d = q - origin, dv = v - vOrigin;
                var c = new double3(double3.Dot(d, x), double3.Dot(d, y), double3.Dot(d, z));
                var cv = new double3(double3.Dot(dv, x), double3.Dot(dv, y), double3.Dot(dv, z));
                // The axes turn about z at the pair's orbital rate.
                double3 r12 = q2 - q1;
                double rate = double3.Cross(r12, v2 - v1).Length() / double3.Dot(r12, r12);
                return (c, cv - double3.Cross(new double3(0, 0, rate), c));
            }
            default:
                return (rel, relV);
        }
    }

    // The two-body rotating frame's origin (the pair's barycentre) and axes.
    public static (double3 Origin, double3 X, double3 Y, double3 Z) RotatingAxes(
        double mu1, double3 q1, double3 v1, double mu2, double3 q2, double3 v2)
    {
        double3 origin = (mu1 * q1 + mu2 * q2) / (mu1 + mu2);
        double3 x = (q2 - q1).Normalized();
        double3 z = double3.Cross(q2 - q1, v2 - v1).Normalized();
        double3 y = double3.Cross(z, x);
        return (origin, x, y, z);
    }
}

// Maps a plotting frame's coordinates to Ecl positions now: about each body
// where it is now, as KSA draws its own orbit lines.
internal readonly struct FrameMapper
{
    private readonly double3 _origin, _x, _y, _z;
    private readonly doubleQuat _rotation;
    private readonly PlotFrame _kind;

    public FrameMapper(FrameSpec frame)
    {
        _kind = frame.Kind;
        _x = _y = _z = double3.Zero;
        _rotation = doubleQuat.Identity;
        switch (frame.Kind)
        {
            case PlotFrame.BodySurface:
                _origin = frame.Centre.GetPositionEcl();
                _rotation = ((IParentBody)frame.Centre).GetCcf2Cce(Universe.GetElapsedTime());
                break;
            case PlotFrame.BarycentricRotating:
                (_origin, _x, _y, _z) = FrameSpec.RotatingAxes(
                    ((IParentBody)frame.Centre).Mu, frame.Centre.GetPositionEcl(), frame.Centre.GetVelocityEcl(),
                    ((IParentBody)frame.Secondary!).Mu, frame.Secondary!.GetPositionEcl(),
                    frame.Secondary!.GetVelocityEcl());
                break;
            default:
                _origin = frame.Centre.GetPositionEcl();
                break;
        }
    }

    public double3 ToEcl(double3 c) => _kind switch
    {
        PlotFrame.BodySurface => _origin + c.Transform(_rotation),
        PlotFrame.BarycentricRotating => _origin + c.X * _x + c.Y * _y + c.Z * _z,
        _ => _origin + c,
    };
}
