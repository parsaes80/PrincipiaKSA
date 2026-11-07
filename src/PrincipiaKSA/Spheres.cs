using System.Runtime.CompilerServices;
using Brutal.Numerics;
using KSA;

namespace PrincipiaKSA;

// KSA's spheres of influence over Principia's bodies, as KSA moves a vessel
// between them (FiniteBurnPredictor.CrossedAt): out of a body's sphere into
// its parent's, into a child's within it. Indices are the ephemeris'.
internal sealed class Spheres
{
    private readonly int[] _parent;      // -1: a root, or a parent Principia doesn't have.
    private readonly double[] _radius;
    private readonly int[][] _children;  // Those with a finite sphere.

    private static readonly ConditionalWeakTable<PrincipiaSystem, Spheres> Cache = new();

    public static Spheres Of(PrincipiaSystem system) => Cache.GetValue(system, s => new Spheres(s));

    private Spheres(PrincipiaSystem system)
    {
        int n = system.Bodies.Count;
        _parent = new int[n];
        _radius = new double[n];
        var children = new List<int>[n];
        for (int i = 0; i < n; i++)
        {
            children[i] = new List<int>();
        }
        for (int i = 0; i < n; i++)
        {
            Astronomical body = system.Bodies[i];
            _radius[i] = body is IParentBody p ? p.SphereOfInfluence : 0;
            _parent[i] = body is not IIndependentRoot && body is IOrbiter { Parent: Astronomical up } &&
                         system.IndexOf(up) is int u ? u : -1;
            if (_parent[i] >= 0 && _radius[i] > 0 && double.IsFinite(_radius[i]))
            {
                children[_parent[i]].Add(i);
            }
        }
        _children = children.Select(c => c.ToArray()).ToArray();
    }

    // The body a vessel at `position` (absolute, at t) is about, starting from
    // the one it was about.
    public int Of(Ephemeris ephemeris, int from, double3 position, double t)
    {
        int body = from;
        double Distance(int i) => (position - ephemeris.State(i, t).Q.ToDouble3()).Length();
        while (_parent[body] >= 0 && Distance(body) > _radius[body])
        {
            body = _parent[body];
        }
        for (bool deeper = true; deeper;)
        {
            deeper = false;
            foreach (int child in _children[body])
            {
                if (Distance(child) < _radius[child])
                {
                    body = child;
                    deeper = true;
                    break;
                }
            }
        }
        return body;
    }

    // Whether `body` is a child of `of`, whose sphere it is entered from.
    public bool IsChild(int body, int of) => _parent[body] == of;
}
