using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// How predictions are drawn, as in Principia's plotting frames.
internal enum PlotFrame
{
    BodyCentred,         // Non-rotating, centred on a body (like KSA's own orbit lines).
    BodySurface,         // Rotating with a body: ground tracks, landing sites.
    BarycentricRotating, // Rotating with two bodies: L-points stand still.
    BodyDirection,       // The same, centred on the first body ("Centred on" in the window).
}

// The bodies a plotting frame is attached to.
internal sealed record FrameSpec(PlotFrame Kind, Astronomical Centre, int CentreIndex,
                                 Astronomical? Secondary, int SecondaryIndex)
{
    public string Describe() => Kind switch
    {
        PlotFrame.BodyCentred => $"centred on {Centre.Id}",
        PlotFrame.BodySurface => $"surface of {Centre.Id}",
        PlotFrame.BodyDirection => $"centred on {Centre.Id}, facing {Secondary?.Id}",
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
        PlotFrame kind = Settings.TwoBodyCentred ? PlotFrame.BodyDirection : PlotFrame.BarycentricRotating;
        return new FrameSpec(kind, centre, centreIndex, secondary, secondaryIndex);
    }

    // An absolute state in this frame (`centre`: the centre's state at t), its velocity as seen in it (less Ω × c when
    // rotating). Two body: about the barycentre, x towards the secondary, z along their orbital angular momentum.
    public (double3 Position, double3 Velocity) Coordinates(Ephemeris ephemeris, double t, QP state, QP centre)
    {
        double3 q = state.Q.ToDouble3(), v = state.P.ToDouble3();
        double3 rel = q - centre.Q.ToDouble3(), relV = v - centre.P.ToDouble3();
        switch (Kind)
        {
            case PlotFrame.BodySurface:
            {
                doubleQuat cce2Ccf = ((IParentBody)Centre).GetCce2Ccf(GameTime.FromSecondsSinceJ2000(t));
                double3 c = rel.Transform(cce2Ccf);
                double omega = Centre is Celestial celestial ? celestial.GetAngularVelocity() : 0;
                return (c, relV.Transform(cce2Ccf) - double3.Cross(new double3(0, 0, omega), c));
            }
            case PlotFrame.BarycentricRotating:
            case PlotFrame.BodyDirection:
            {
                var (origin, vOrigin, x, y, z, rate) = RotatingAt(ephemeris, t, centre);
                double3 d = q - origin, dv = v - vOrigin;
                var c = new double3(double3.Dot(d, x), double3.Dot(d, y), double3.Dot(d, z));
                var cv = new double3(double3.Dot(dv, x), double3.Dot(dv, y), double3.Dot(dv, z));
                return (c, cv - double3.Cross(new double3(0, 0, rate), c));
            }
            default:
                return (rel, relV);
        }
    }

    // The "Two body" frame at t (`centre`: the centre's state then): origin, its velocity, axes, and the rate they turn about z.
    private (double3 Origin, double3 VOrigin, double3 X, double3 Y, double3 Z, double Rate) RotatingAt(
        Ephemeris ephemeris, double t, QP centre)
    {
        QP secondary = ephemeris.State(SecondaryIndex, t);
        double mu1 = ((IParentBody)Centre).Mu, mu2 = ((IParentBody)Secondary!).Mu;
        double3 q1 = centre.Q.ToDouble3(), v1 = centre.P.ToDouble3();
        double3 q2 = secondary.Q.ToDouble3(), v2 = secondary.P.ToDouble3();
        (double3 origin, double3 x, double3 y, double3 z) = RotatingAxes(mu1, q1, v1, mu2, q2, v2);
        double3 vOrigin = (mu1 * v1 + mu2 * v2) / (mu1 + mu2);
        if (Kind == PlotFrame.BodyDirection)
        {
            (origin, vOrigin) = (q1, v1);
        }
        double3 r12 = q2 - q1;
        double rate = double3.Cross(r12, v2 - v1).Length() / double3.Dot(r12, r12);
        return (origin, vOrigin, x, y, z, rate);
    }

    // An absolute state's velocity at t as seen in this frame, in Ecl axes (Coordinates' velocity, not turned).
    public double3 FrameVelocity(Ephemeris ephemeris, double t, QP state)
    {
        QP centre = ephemeris.State(CentreIndex, t);
        double3 q = state.Q.ToDouble3(), v = state.P.ToDouble3();
        switch (Kind)
        {
            case PlotFrame.BodySurface:
            {
                double3 axis = double3.UnitZ.Transform(
                    ((IParentBody)Centre).GetCcf2Cce(GameTime.FromSecondsSinceJ2000(t)));
                double omega = Centre is Celestial celestial ? celestial.GetAngularVelocity() : 0;
                return v - centre.P.ToDouble3() - double3.Cross(omega * axis, q - centre.Q.ToDouble3());
            }
            case PlotFrame.BarycentricRotating:
            case PlotFrame.BodyDirection:
            {
                var (origin, vOrigin, _, _, z, rate) = RotatingAt(ephemeris, t, centre);
                return v - vOrigin - double3.Cross(rate * z, q - origin);
            }
            default:
                return v - centre.P.ToDouble3();
        }
    }

    // A direction in Ecl axes at t, turned as the map draws that moment now (`now`: this frame's mapper).
    public double3 DirectionNow(Ephemeris ephemeris, double t, double3 d, in FrameMapper now)
    {
        double3 c = d;
        if (Kind == PlotFrame.BodySurface)
        {
            c = d.Transform(((IParentBody)Centre).GetCce2Ccf(GameTime.FromSecondsSinceJ2000(t)));
        }
        else if (Kind is PlotFrame.BarycentricRotating or PlotFrame.BodyDirection)
        {
            var (_, _, x, y, z, _) = RotatingAt(ephemeris, t, ephemeris.State(CentreIndex, t));
            c = new double3(double3.Dot(d, x), double3.Dot(d, y), double3.Dot(d, z));
        }
        return now.ToEcl(c) - now.ToEcl(double3.Zero);
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

    // L1–L5 (circular restricted three-body, at the present distance); L2 beyond the lighter body, L4 ahead of it.
    public static double3[] LagrangePoints(double mu1, double mu2, double distance)
    {
        // In units of the distance: the centre at −m, the secondary at 1 − m.
        double m = mu2 / (mu1 + mu2);
        double light = m <= 0.5 ? 1 - m : -m, heavy = m <= 0.5 ? -m : 1 - m;
        double towardsLight = Math.Sign(light - heavy);
        double Collinear(double a, double b)
        {
            // Net acceleration on the x axis grows with x between the bodies' singularities.
            double lo = Math.Min(a, b), hi = Math.Max(a, b);
            for (int i = 0; i < 200 && hi - lo > 1e-15; i++)
            {
                double x = 0.5 * (lo + hi), d1 = x + m, d2 = x - 1 + m;
                double f = x - (1 - m) * d1 / Math.Pow(Math.Abs(d1), 3) - m * d2 / Math.Pow(Math.Abs(d2), 3);
                (lo, hi) = f < 0 ? (x, hi) : (lo, x);
            }
            return 0.5 * (lo + hi);
        }
        double l1 = Collinear(heavy, light);
        double l2 = Collinear(light, light + 2 * towardsLight);
        double l3 = Collinear(heavy - 2 * towardsLight, heavy);
        double side = towardsLight * Math.Sqrt(3) / 2;
        return
        [
            new double3(l1, 0, 0) * distance,
            new double3(l2, 0, 0) * distance,
            new double3(l3, 0, 0) * distance,
            new double3(0.5 - m, side, 0) * distance,
            new double3(0.5 - m, -side, 0) * distance,
        ];
    }
}

// A frame's coordinates to Ecl positions now, about each body where it is now, as KSA draws its own orbit lines.
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
            case PlotFrame.BodyDirection:
                (_origin, _x, _y, _z) = FrameSpec.RotatingAxes(
                    ((IParentBody)frame.Centre).Mu, frame.Centre.GetPositionEcl(), frame.Centre.GetVelocityEcl(),
                    ((IParentBody)frame.Secondary!).Mu, frame.Secondary!.GetPositionEcl(),
                    frame.Secondary!.GetVelocityEcl());
                if (frame.Kind == PlotFrame.BodyDirection)
                {
                    _origin = frame.Centre.GetPositionEcl();
                }
                break;
            default:
                _origin = frame.Centre.GetPositionEcl();
                break;
        }
    }

    public double3 ToEcl(double3 c) => _kind switch
    {
        PlotFrame.BodySurface => _origin + c.Transform(_rotation),
        PlotFrame.BarycentricRotating or PlotFrame.BodyDirection => _origin + c.X * _x + c.Y * _y + c.Z * _z,
        _ => _origin + c,
    };
}
