// Checks which μ convention turns KSA's orbital elements (JPL Horizons,
// Content/Core/Astronomicals.xml) back into the real solar system.
//
// Reference: Principia's own JPL initial state, integrated with Principia's
// gravity model (geopotentials included) to KSA's epoch, JD 2461009.5 TDB.
// Candidates: the XML elements turned into states with
//   A) μ = GM(parent)              (what KSA does), or
//   B) μ = GM(parent) + GM(body)   (what Horizons uses for its elements).
//
// Usage: sol_reference /path/to/Principia

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "astronomy/epoch.hpp"
#include "astronomy/frames.hpp"
#include "astronomy/time_scales.hpp"
#include "geometry/instant.hpp"
#include "integrators/methods.hpp"
#include "integrators/symplectic_runge_kutta_nyström_integrator.hpp"
#include "physics/ephemeris.hpp"
#include "physics/solar_system.hpp"
#include "quantities/si.hpp"

using namespace principia::astronomy::_epoch;
using namespace principia::astronomy::_frames;
using namespace principia::astronomy::_time_scales;
using namespace principia::geometry::_instant;
using namespace principia::integrators::_methods;
using namespace principia::integrators::_symplectic_runge_kutta_nyström_integrator;  // NOLINT
using namespace principia::physics::_ephemeris;
using namespace principia::physics::_solar_system;
using namespace principia::quantities::_named_quantities;
using namespace principia::quantities::_quantities;
using namespace principia::quantities::_si;

namespace {

struct V { double x, y, z; };
V operator-(V a, V b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
V operator+(V a, V b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
V operator*(double k, V a) { return {k * a.x, k * a.y, k * a.z}; }
double Norm(V a) { return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z); }

constexpr double deg = M_PI / 180;
// Horizons' "ecliptic and mean equinox of reference epoch": IAU 1976
// obliquity at J2000, 84381.448″.
constexpr double obliquity = 84381.448 / 3600 * deg;

// ICRS (equatorial) → J2000 ecliptic: rotation by ε about X.
V ToEcliptic(V e) {
  return {e.x,
          e.y * std::cos(obliquity) + e.z * std::sin(obliquity),
          -e.y * std::sin(obliquity) + e.z * std::cos(obliquity)};
}

struct State { V q, p; };

// Elliptic Kepler orbit from KSA's XML elements at KSA time t (s).
State FromElements(double a_km, double e, double i_deg, double node_deg,
                   double peri_deg, double tp, double t, double μ) {
  double const a = a_km * 1000;
  double const n = std::sqrt(μ / (a * a * a));
  double const M = n * (t - tp);
  double E = M;
  for (int k = 0; k < 50; ++k) {
    E -= (E - e * std::sin(E) - M) / (1 - e * std::cos(E));
  }
  double const r = a * (1 - e * std::cos(E));
  double const b = std::sqrt(1 - e * e);
  V const q{a * (std::cos(E) - e), a * b * std::sin(E), 0};
  V const p = (std::sqrt(μ * a) / r) * V{-std::sin(E), b * std::cos(E), 0};
  double const Ω = node_deg * deg, i = i_deg * deg, ω = peri_deg * deg;
  auto rotate = [&](V v) {  // Rz(Ω) Rx(i) Rz(ω)
    V const v1{v.x * std::cos(ω) - v.y * std::sin(ω),
               v.x * std::sin(ω) + v.y * std::cos(ω), v.z};
    V const v2{v1.x, v1.y * std::cos(i) - v1.z * std::sin(i),
               v1.y * std::sin(i) + v1.z * std::cos(i)};
    return V{v2.x * std::cos(Ω) - v2.y * std::sin(Ω),
             v2.x * std::sin(Ω) + v2.y * std::cos(Ω), v2.z};
  };
  return {rotate(q), rotate(p)};
}

double Dot(V a, V b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
V Cross(V a, V b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

// Δq split into radial, along-track and normal components (km), and the
// along-track part as a time offset (s).
void Line(char const* label, State const& real, State const& c) {
  V const d = c.q - real.q;
  V const r = (1 / Norm(real.q)) * real.q;
  V const n = (1 / Norm(Cross(real.q, real.p))) * Cross(real.q, real.p);
  V const t = Cross(n, r);
  std::printf("  %-22s Δq = %11.3f km (radial %10.3f, along %11.3f = %8.2f s, "
              "normal %9.3f), Δv = %.6f m/s\n",
              label, Norm(d) / 1000, Dot(d, r) / 1000, Dot(d, t) / 1000,
              Dot(d, t) / Dot(real.p, t), Dot(d, n) / 1000, Norm(c.p - real.p));
}

void Compare(char const* what, State const& real, State const& a,
             State const& b) {
  std::printf("%s: real |q| = %.3f km, |v| = %.6f m/s\n", what,
              Norm(real.q) / 1000, Norm(real.p));
  Line("A (μ parent)", real, a);
  Line("B (μ parent + body)", real, b);
}

}  // namespace

int main(int argc, char** argv) {
  std::string const dir = argc > 1 ? argv[1] : ".";
  SolarSystem<ICRS> const solar_system(
      dir + "/astronomy/sol_gravity_model.proto.txt",
      dir + "/astronomy/sol_initial_state_jd_2455200_500000000.proto.txt");
  auto const ephemeris = solar_system.MakeEphemeris(
      Ephemeris<ICRS>::AccuracyParameters(1 * Milli(Metre), 0x1.0p-24),
      Ephemeris<ICRS>::FixedStepParameters(
          SymplecticRungeKuttaNyströmIntegrator<
              BlanesMoan2002SRKN14A,
              Ephemeris<ICRS>::NewtonianMotionEquation>(),
          10 * Minute));
  Instant const ksa_epoch = "JD2461009.5"_TT;  // Horizons uses TDB; < 2 ms.
  std::printf("integrating from %s to JD 2461009.5...\n",
              solar_system.epoch_literal().c_str());
  CHECK_OK(ephemeris->Prolong(ksa_epoch));

  auto state = [&](std::string const& name) {
    auto const dof =
        solar_system.trajectory(*ephemeris, name).EvaluateDegreesOfFreedom(
            ksa_epoch);
    auto const q = (dof.position() - ICRS::origin).coordinates();
    auto const p = dof.velocity().coordinates();
    return State{ToEcliptic({q.x / Metre, q.y / Metre, q.z / Metre}),
                 ToEcliptic({p.x / (Metre / Second), p.y / (Metre / Second),
                             p.z / (Metre / Second)})};
  };
  auto μ = [&](std::string const& name) {
    return solar_system.gravitational_parameter(name) /
           Unit<GravitationalParameter>;
  };
  auto relative = [](State a, State b) { return State{a.q - b.q, a.p - b.p}; };

  State const sun = state("Sun"), earth = state("Earth"), moon = state("Moon");
  double const μ_sun = μ("Sun"), μ_earth = μ("Earth"), μ_moon = μ("Moon");
  State const emb{(1 / (μ_earth + μ_moon)) * (μ_earth * earth.q + μ_moon * moon.q),
                  (1 / (μ_earth + μ_moon)) * (μ_earth * earth.p + μ_moon * moon.p)};

  // Content/Core/Astronomicals.xml, KSA 2026.10.25.5707.
  auto earth_elements = [&](double μ) {
    return FromElements(1.495396277103892E+08, 1.657443784940060E-02,
                        4.495777668608611E-03, 2.001005464352027E+02,
                        2.647204271842132E+02, 3155415.2731954656, 0, μ);
  };
  auto moon_elements = [&](double μ) {
    return FromElements(3.808633349807188E+05, 6.734146418596218E-02,
                        5.027765739305488E+00, 3.439586928342719E+02,
                        9.095632163367938E+01, 442332.8562855744, 0, μ);
  };

  Compare("Earth − Sun", relative(earth, sun), earth_elements(μ_sun),
          earth_elements(μ_sun + μ_earth));
  Compare("(Earth–Moon barycentre) − Sun", relative(emb, sun),
          earth_elements(μ_sun), earth_elements(μ_sun + μ_earth + μ_moon));
  Compare("Moon − Earth", relative(moon, earth), moon_elements(μ_earth),
          moon_elements(μ_earth + μ_moon));

  auto elements = [](double a, double e, double i, double node, double peri,
                     double tp) {
    return [=](double μ) { return FromElements(a, e, i, node, peri, tp, 0, μ); };
  };
  auto barycentre = [&](std::vector<std::string> const& names) {
    double total = 0;
    State b{{0, 0, 0}, {0, 0, 0}};
    for (auto const& name : names) {
      double const m = μ(name);
      State const s = state(name);
      b.q = b.q + m * s.q;
      b.p = b.p + m * s.p;
      total += m;
    }
    return State{(1 / total) * b.q, (1 / total) * b.p};
  };
  auto const mars = elements(2.279399440595953E+08, 9.348560325567271E-02,
                             1.847500920441235E+00, 4.948350995893581E+01,
                             2.866279383251176E+02, 10049350.4347086144);
  auto const jupiter = elements(7.783184372433803E+08, 4.832479219312407E-02,
                                1.303590359360468E+00, 1.005180387211400E+02,
                                2.735957096173138E+02, -90113548.8271415328);
  auto const saturn = elements(1.427887258370589E+09, 5.542239872393825E-02,
                               2.486575364141227E+00, 1.135601857911130E+02,
                               3.383383032687136E+02, 219817510.331082336);
  double const μ_mars = μ("Mars"), μ_jupiter = μ("Jupiter"), μ_saturn = μ("Saturn");
  Compare("Mars − Sun", relative(state("Mars"), sun), mars(μ_sun),
          mars(μ_sun + μ_mars));
  Compare("Jupiter − Sun", relative(state("Jupiter"), sun), jupiter(μ_sun),
          jupiter(μ_sun + μ_jupiter));
  Compare("(Jupiter system barycentre) − Sun",
          relative(barycentre({"Jupiter", "Io", "Europa", "Ganymede", "Callisto"}), sun),
          jupiter(μ_sun), jupiter(μ_sun + μ_jupiter));
  Compare("Saturn − Sun", relative(state("Saturn"), sun), saturn(μ_sun),
          saturn(μ_sun + μ_saturn));
  Compare("(Saturn system barycentre) − Sun",
          relative(barycentre({"Saturn", "Titan", "Rhea", "Iapetus", "Dione", "Tethys"}), sun),
          saturn(μ_sun), saturn(μ_sun + μ_saturn));
  // Relative to the solar-system barycentre instead of the Sun.
  State const ssb = barycentre(solar_system.names());
  Compare("Jupiter − SSB", relative(state("Jupiter"), ssb), jupiter(μ_sun),
          jupiter(μ_sun + μ_jupiter));
  return 0;
}
