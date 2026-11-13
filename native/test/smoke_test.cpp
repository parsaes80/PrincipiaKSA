// Exercises libprincipia_ksa.so through its C API only, as C# does, with a
// Sun–Earth–Moon system. Run with `zig build test`.

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <limits>
#include <numbers>
#include <thread>
#include <vector>

#include "../principia_ksa.h"

namespace {

int failures = 0;

void Expect(bool const ok, char const* const what) {
  std::printf("  [%s] %s\n", ok ? "ok" : "FAIL", what);
  if (!ok) {
    ++failures;
  }
}

double Norm(principia_ksa_XYZ const& v) {
  return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
principia_ksa_XYZ Minus(principia_ksa_XYZ const& a, principia_ksa_XYZ const& b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double Energy(principia_ksa_Body const* bodies, principia_ksa_QP const* s, int n) {
  double e = 0;
  for (int i = 0; i < n; ++i) {
    double const v = Norm(s[i].p);
    e += 0.5 * bodies[i].gravitational_parameter * v * v;
    for (int j = 0; j < i; ++j) {
      e -= bodies[i].gravitational_parameter * bodies[j].gravitational_parameter /
           Norm(Minus(s[i].q, s[j].q));
    }
  }
  return e;  // G × the total energy; G cancels in relative drift.
}

// A prediction, copied out of the library's arrays.
struct Prediction {
  principia_ksa_Status status = PRINCIPIA_KSA_OK;
  std::vector<double> times;
  std::vector<principia_ksa_QP> states;
  std::vector<principia_ksa_Apsis> apsides;
  std::int64_t steps = 0;
  principia_ksa_PredictionEnd end = PRINCIPIA_KSA_PREDICTION_REACHED_END;
  int count() const { return static_cast<int>(times.size()); }
};

// `t_end` +∞: until the steps run out. Thinned to `downsampling` metres (0:
// every step), at 1 m and 1 m/s.
Prediction Predict(principia_ksa_Ephemeris const* const e,
                   principia_ksa_QP const* const initial, double const t0,
                   double const t_end, std::int64_t const max_steps,
                   double const downsampling = 10,
                   std::vector<int> const& apsis_bodies = {},
                   principia_ksa_XYZ const direction = {0, 0, 0},
                   double const thrust = 0, double const mass = 0,
                   double const mass_flow = 0) {
  Prediction p;
  double* times = nullptr;
  principia_ksa_QP* states = nullptr;
  principia_ksa_Apsis* apsides = nullptr;
  int count = 0, apsis_count = 0;
  p.status = principia_ksa__Predict(
      e, initial, t0, t_end, max_steps, 1, 1, direction, thrust, mass,
      mass_flow, downsampling, apsis_bodies.data(),
      static_cast<int>(apsis_bodies.size()), 100'000, &times, &states, &count,
      &apsides, &apsis_count, &p.steps, &p.end);
  p.times.assign(times, times + count);
  p.states.assign(states, states + count);
  p.apsides.assign(apsides, apsides + apsis_count);
  principia_ksa__Free(times);
  principia_ksa__Free(states);
  principia_ksa__Free(apsides);
  return p;
}

// Position at time t on the cubic Hermite spline through a prediction's
// points, as the C# side interpolates them.
principia_ksa_XYZ HermiteAt(Prediction const& p, double const t) {
  int i = 1;
  while (i < p.count() - 1 && p.times[i] < t) ++i;
  double const h = p.times[i] - p.times[i - 1];
  double const u = (t - p.times[i - 1]) / h, u2 = u * u, u3 = u2 * u;
  auto const& a = p.states[i - 1];
  auto const& b = p.states[i];
  double const c0 = 2 * u3 - 3 * u2 + 1, c1 = (u3 - 2 * u2 + u) * h,
               c2 = -2 * u3 + 3 * u2, c3 = (u3 - u2) * h;
  return {c0 * a.q.x + c1 * a.p.x + c2 * b.q.x + c3 * b.p.x,
          c0 * a.q.y + c1 * a.p.y + c2 * b.q.y + c3 * b.p.y,
          c0 * a.q.z + c1 * a.p.z + c2 * b.q.z + c3 * b.p.z};
}

double const infinity = std::numeric_limits<double>::infinity();

void Log(int const severity, char const* const message) {
  std::printf("  principia[%d]: %s\n", severity, message);
}

}  // namespace

int main(int argc, char** argv) {
  std::setvbuf(stdout, nullptr, _IONBF, 0);  // Nothing lost if it aborts.
  std::printf("%s\n", principia_ksa__Version());
  principia_ksa__SetLogCallback(&Log);
  if (argc < 2) {
    std::printf("usage: smoke_test path/to/sol_gravity_model.proto.txt\n");
    return 2;
  }

  double const au = 1.495978707e11;
  principia_ksa_Body const bodies[] = {
      {"Sun", 1.32712440018e20},
      {"Earth", 3.986004418e14},
      {"Moon", 4.9048695e12},
  };
  int const n = 3;
  principia_ksa_QP states[] = {
      {{0, 0, 0}, {0, 0, 0}},
      {{au, 0, 0}, {0, 29784.7, 0}},
      {{au + 3.844e8, 0, 0}, {0, 29784.7 + 1022.0, 0}},
  };
  // Move the origin to the centre of mass, at rest, as the C# side does.
  principia_ksa_QP centre{};
  double total = 0;
  for (int i = 0; i < n; ++i) {
    double const m = bodies[i].gravitational_parameter;
    total += m;
    centre.q.x += m * states[i].q.x; centre.q.y += m * states[i].q.y;
    centre.p.x += m * states[i].p.x; centre.p.y += m * states[i].p.y;
  }
  for (int i = 0; i < n; ++i) {
    states[i].q.x -= centre.q.x / total; states[i].q.y -= centre.q.y / total;
    states[i].p.x -= centre.p.x / total; states[i].p.y -= centre.p.y / total;
  }

  double const t0 = 817'732'800.0;  // KSA's UniverseTime zero.
  double const year = 365.25 * 86400;

  std::printf("create:\n");
  principia_ksa_Ephemeris* ephemeris = nullptr;
  Expect(principia_ksa__EphemerisCreate(bodies, states, n, t0, 2100, 1e-3,
                                        &ephemeris) == PRINCIPIA_KSA_OK &&
             ephemeris != nullptr,
         "EphemerisCreate");
  if (ephemeris == nullptr) {
    std::printf("error: %s\n", principia_ksa__LastError());
    return 1;
  }

  principia_ksa_QP at_t0[3];
  Expect(principia_ksa__EphemerisAllStates(ephemeris, t0, at_t0, n) ==
             PRINCIPIA_KSA_OK,
         "AllStates at t0");
  double worst = 0;
  for (int i = 0; i < n; ++i) {
    worst = std::fmax(worst, Norm(Minus(at_t0[i].q, states[i].q)));
  }
  std::printf("  largest position error at t0: %.3e m\n", worst);
  Expect(worst < 1e-3, "states at t0 match the input within 1 mm");

  std::printf("prolong one year:\n");
  auto const start = std::chrono::steady_clock::now();
  Expect(principia_ksa__EphemerisProlong(ephemeris, t0 + year) ==
             PRINCIPIA_KSA_OK,
         "Prolong");
  double const ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - start).count();
  std::printf("  took %.1f ms\n", ms);
  double t_min, t_max;
  principia_ksa__EphemerisTimeRange(ephemeris, &t_min, &t_max);
  std::printf("  range: [%.1f, %.1f] s since J2000\n", t_min, t_max);
  Expect(t_max >= t0 + year, "t_max reaches t0 + 1 year");

  principia_ksa_QP at_year[3];
  principia_ksa__EphemerisAllStates(ephemeris, t0 + year, at_year, n);
  double const sun_earth = Norm(Minus(at_year[1].q, at_year[0].q)) / au;
  double const earth_moon = Norm(Minus(at_year[2].q, at_year[1].q));
  double const drift = std::fabs(Energy(bodies, at_year, n) /
                                     Energy(bodies, at_t0, n) - 1);
  std::printf("  Sun–Earth %.6f AU, Earth–Moon %.0f km, energy drift %.2e\n",
              sun_earth, earth_moon / 1000, drift);
  Expect(sun_earth > 0.97 && sun_earth < 1.03, "Earth stays near 1 AU");
  Expect(earth_moon > 3.0e8 && earth_moon < 4.5e8, "Moon stays bound to Earth");
  Expect(drift < 1e-9, "energy is conserved to 1e-9");

  std::printf("gravity at Earth's surface:\n");
  principia_ksa_QP earth;
  principia_ksa__EphemerisBodyState(ephemeris, 1, t0, &earth);
  principia_ksa_XYZ const surface{earth.q.x, earth.q.y, earth.q.z + 6.371e6};
  principia_ksa_XYZ g;
  Expect(principia_ksa__EphemerisGravitationalAcceleration(
             ephemeris, surface, t0, &g) == PRINCIPIA_KSA_OK,
         "GravitationalAcceleration");
  std::printf("  |g| = %.4f m/s²\n", Norm(g));
  Expect(std::fabs(Norm(g) - 9.82) < 0.01, "|g| ≈ 9.82 m/s²");

  std::printf("bad inputs return errors instead of crashing:\n");
  principia_ksa_QP qp;
  Expect(principia_ksa__EphemerisBodyState(ephemeris, 1, t0 - 1e6, &qp) ==
             PRINCIPIA_KSA_OUT_OF_RANGE,
         "time before t_min → OUT_OF_RANGE");
  std::printf("    (%s)\n", principia_ksa__LastError());
  Expect(principia_ksa__EphemerisBodyState(ephemeris, 7, t0, &qp) ==
             PRINCIPIA_KSA_INVALID_ARGUMENT,
         "bad body index → INVALID_ARGUMENT");
  Expect(principia_ksa__EphemerisProlong(ephemeris, NAN) ==
             PRINCIPIA_KSA_INVALID_ARGUMENT,
         "NaN time → INVALID_ARGUMENT");
  principia_ksa_Body const massless[] = {{"Rock", 0}};
  principia_ksa_Ephemeris* bad = nullptr;
  Expect(principia_ksa__EphemerisCreate(massless, states, 1, t0, 2100, 1e-3,
                                        &bad) ==
             PRINCIPIA_KSA_INVALID_ARGUMENT && bad == nullptr,
         "μ = 0 → INVALID_ARGUMENT");
  std::printf("    (%s)\n", principia_ksa__LastError());

  principia_ksa__EphemerisDestroy(ephemeris);

  std::printf("geopotential (Earth with the real J2 and beyond):\n");
  Expect(principia_ksa__LoadGravityModel(argv[1]) == PRINCIPIA_KSA_OK,
         "LoadGravityModel");
  {
    // The Moon: μ ≈ 4902.8 km³/s², r ≈ 1737.4 km, IAU pole α ≈ 269.99°, δ ≈ 66.54°.
    double mu = 0, radius = 0, ra = 0, dec = 0;
    Expect(principia_ksa__GravityModelBody("Moon", &mu, &radius, &ra, &dec) == PRINCIPIA_KSA_OK &&
               std::fabs(mu / 4.9028e12 - 1) < 1e-4 && std::fabs(radius / 1.7374e6 - 1) < 1e-4 &&
               std::fabs(ra * 180 / std::numbers::pi - 269.99) < 0.1 &&
               std::fabs(dec * 180 / std::numbers::pi - 66.54) < 0.1,
           "GravityModelBody gives the Moon's μ, radius and IAU pole");
    Expect(principia_ksa__GravityModelBody("Nowhere", &mu, &radius, &ra, &dec) ==
               PRINCIPIA_KSA_INVALID_ARGUMENT,
           "GravityModelBody of an unknown body → INVALID_ARGUMENT");
  }
  principia_ksa_Body oblate[] = {
      {"Sun", 1.32712440018e20},
      {"Earth", 3.986004418e14, 6.371e6, {0, 0, 1}, {1, 0, 0}, t0,
       7.2921159e-5, "Earth"},
  };
  principia_ksa_Ephemeris* geo = nullptr;
  Expect(principia_ksa__EphemerisCreate(oblate, states, 2, t0, 2100, 1e-3,
                                        &geo) == PRINCIPIA_KSA_OK,
         "EphemerisCreate with an OblateBody");
  if (geo != nullptr) {
    principia_ksa_QP e;
    principia_ksa__EphemerisBodyState(geo, 1, t0, &e);
    double const r = 7.0e6;
    principia_ksa_XYZ g_pole, g_equator;
    principia_ksa__EphemerisGravitationalAcceleration(
        geo, {e.q.x, e.q.y, e.q.z + r}, t0, &g_pole);
    principia_ksa__EphemerisGravitationalAcceleration(
        geo, {e.q.x, e.q.y + r, e.q.z}, t0, &g_equator);
    // Remove the Sun's pull (nearly the same at both points) via the
    // component along the Earth direction.
    double const pole = -g_pole.z, equator = -g_equator.y;
    double const ratio = equator / pole - 1;
    std::printf("  g(equator)/g(pole) − 1 at r = 7000 km: %.5f%%"
                " (J2 alone: +%.5f%%)\n",
                100 * ratio, 100 * 4.5 * 1.0826e-3 * std::pow(6.3781e6 / r, 2));
    // J3, J4 and C22 move this by a few 1e-5 at 7000 km.
    Expect(std::fabs(ratio - 4.5 * 1.0826e-3 * std::pow(6.3781e6 / r, 2)) < 1e-4,
           "equator/pole difference matches Earth's J2 (with higher terms)");
    principia_ksa__EphemerisDestroy(geo);
  }

  // Vesta's model has only J2, so the exact J2 field applies:
  // g = μ/r² (1 + 3/2 J2 (R/r)²) on the equator, μ/r² (1 − 3 J2 (R/r)²) at
  // the pole.
  principia_ksa_Body vesta[] = {
      {"Sun", 1.32712440018e20},
      {"Vesta", 3.986004418e14, 6.371e6, {0, 0, 1}, {1, 0, 0}, t0, 7.29e-5,
       "Vesta"}};
  Expect(principia_ksa__EphemerisCreate(vesta, states, 2, t0, 2100, 1e-3,
                                        &geo) == PRINCIPIA_KSA_OK,
         "EphemerisCreate with a J2 body");
  if (geo != nullptr) {
    principia_ksa_QP v;
    principia_ksa__EphemerisBodyState(geo, 1, t0, &v);
    double const r = 1.0e6, x = std::pow(2.65e5 / r, 2), j2 = 0.071060891954441914;
    principia_ksa_XYZ g_pole, g_equator;
    principia_ksa__EphemerisGravitationalAcceleration(
        geo, {v.q.x, v.q.y, v.q.z + r}, t0, &g_pole);
    principia_ksa__EphemerisGravitationalAcceleration(
        geo, {v.q.x, v.q.y + r, v.q.z}, t0, &g_equator);
    double const μr2 = 3.986004418e14 / (r * r);
    double const pole_error = -g_pole.z / (μr2 * (1 - 3 * j2 * x)) - 1;
    double const equator_error = -g_equator.y / (μr2 * (1 + 1.5 * j2 * x)) - 1;
    std::printf("  J2 field vs formula: pole %.2e, equator %.2e (relative)\n",
                pole_error, equator_error);
    Expect(std::fabs(pole_error) < 1e-9 && std::fabs(equator_error) < 1e-9,
           "J2 field matches the formula to 1e-9");
    principia_ksa__EphemerisDestroy(geo);
  }
  principia_ksa_Body bad_axis[] = {
      {"Sun", 1.32712440018e20},
      {"Earth", 3.986004418e14, 6.371e6, {0, 0, 1}, {0, 0, 2}, t0, 7.29e-5,
       "Earth"}};
  Expect(principia_ksa__EphemerisCreate(bad_axis, states, 2, t0, 2100, 1e-3,
                                        &geo) == PRINCIPIA_KSA_INVALID_ARGUMENT,
         "prime meridian along the pole → INVALID_ARGUMENT");
  std::printf("    (%s)\n", principia_ksa__LastError());
  std::printf("vessels (massless bodies) in the n-body field:\n");
  {
    principia_ksa_Body sun_earth[] = {
        {"Sun", 1.32712440018e20},
        {"Earth", 3.986004418e14, 6.371e6, {0, 0, 1}, {1, 0, 0}, t0,
         7.2921159e-5, "Earth"}};
    principia_ksa_QP se[] = {{{0, 0, 0}, {0, 0, 0}},
                             {{au, 0, 0}, {0, 29784.7, 0}}};
    principia_ksa_Ephemeris* e = nullptr;
    principia_ksa__EphemerisCreate(sun_earth, se, 2, t0, 2100, 1e-3, &e);
    principia_ksa__EphemerisProlong(e, t0 + 2 * 86400);
    principia_ksa_QP earth0, earth1;
    principia_ksa__EphemerisBodyState(e, 1, t0, &earth0);
    principia_ksa__EphemerisBodyState(e, 1, t0 + 86400, &earth1);
    // ISS-like orbit: r = 6778 km, i = 51.6°, ascending node on +x.
    double const r = 6.778e6, i = 51.6 * std::numbers::pi / 180;
    double const v = std::sqrt(3.986004418e14 / r);
    principia_ksa_QP iss{{earth0.q.x + r, earth0.q.y, earth0.q.z},
                         {earth0.p.x, earth0.p.y + v * std::cos(i),
                          earth0.p.z + v * std::sin(i)}};
    principia_ksa_QP after;
    auto const start = std::chrono::steady_clock::now();
    Expect(principia_ksa__FlowMassless(e, &iss, t0, t0 + 86400, &after) ==
               PRINCIPIA_KSA_OK,
           "FlowMassless for a day in LEO");
    double const ms = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - start).count();
    principia_ksa_XYZ const q = Minus(after.q, earth1.q);
    principia_ksa_XYZ const p = Minus(after.p, earth1.p);
    principia_ksa_XYZ const h{q.y * p.z - q.z * p.y, q.z * p.x - q.x * p.z,
                              q.x * p.y - q.y * p.x};
    double const node = std::atan2(h.x, -h.y) * 180 / std::numbers::pi;  // Ω of h = (sin i sin Ω, −sin i cos Ω, cos i)
    std::printf("  1 day in %.1f ms: |r| = %.1f km, node moved %.3f°/day "
                "(J2 theory for the ISS orbit: ≈ −5.0°/day)\n",
                ms, Norm(q) / 1000, node);
    Expect(node < -4.5 && node > -5.5, "nodal regression from Earth's J2");
    // At this instant Earth's spin half-angle takes the slow path of
    // Principia's sin and cos (CORE-MATH), which in the first Windows build
    // returned a sine of 64 and a field of 1.7e15 m/s².
    principia_ksa_XYZ const slow{149591642300.29727 - 149597841546.37173,
                                 91721085.496203765 - 93395103.502674341,
                                 -2133888.81751918};
    principia_ksa_XYZ g_slow;
    Expect(principia_ksa__EphemerisRelativeAcceleration(
               e, 1, slow, 817735935.6740061, &g_slow) == PRINCIPIA_KSA_OK &&
               std::fabs(Norm(g_slow) - 8.7147) < 1e-3,
           "Earth's field where sin and cos take their slow path");
    Expect(principia_ksa__FlowMassless(e, &iss, t0 + 10, t0, &after) ==
               PRINCIPIA_KSA_INVALID_ARGUMENT,
           "t1 before t0 → INVALID_ARGUMENT (not a CHECK abort)");

    {
      auto const t = std::chrono::steady_clock::now();
      Prediction const day = Predict(e, &iss, t0, t0 + 86400, 100000);
      Expect(day.status == PRINCIPIA_KSA_OK &&
                 day.end == PRINCIPIA_KSA_PREDICTION_REACHED_END &&
                 day.count() >= 2 && day.times[0] == t0 &&
                 day.times[day.count() - 1] == t0 + 86400,
             "a day's prediction reaches its end, first and last kept");
      std::printf("  a day in %.1f ms: %lld steps, %d points kept within 10 m\n",
                  std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t).count(),
                  static_cast<long long>(day.steps), day.count());
      // Every step, unthinned; apsides exact relative to Earth.
      {
        Prediction const every =
            Predict(e, &iss, t0 + 7, t0 + 7 + 86400, 100000, 0, {1});
        bool increasing = true;
        for (int i = 1; i < every.count(); ++i) {
          increasing &= every.times[i] > every.times[i - 1];
        }
        int peri = 0, apo = 0;
        double min_peri = 1e30, max_apo = 0;
        for (auto const& a : every.apsides) {
          if (a.is_apoapsis) {
            ++apo;
            max_apo = std::fmax(max_apo, a.distance);
          } else {
            ++peri;
            min_peri = std::fmin(min_peri, a.distance);
          }
        }
        std::printf("  every step: %lld steps, %d points; %d periapsides (lowest %.1f km), "
                    "%d apoapsides (highest %.1f km)\n",
                    static_cast<long long>(every.steps), every.count(), peri,
                    min_peri / 1000, apo, max_apo / 1000);
        Expect(every.count() == every.steps + 1 && increasing,
               "unthinned, the points are the integrator's steps");
        Expect(day.count() < every.count(),
               "thinned to 10 m, fewer points than steps");
        Expect(peri >= 14 && peri <= 17 && apo >= 14 && apo <= 17,
               "about one periapsis and one apoapsis per 92 min orbit");
        // The thinned points' spline stays on the unthinned path.
        Prediction const thin = Predict(e, &iss, t0 + 7, t0 + 7 + 86400, 100000);
        double worst = 0;
        for (int i = 0; i < every.count(); ++i) {
          worst = std::fmax(worst, Norm(Minus(HermiteAt(thin, every.times[i]),
                                              every.states[i].q)));
        }
        std::printf("  thinned to %d points, its spline within %.2f m of every step\n",
                    thin.count(), worst);
        Expect(worst < 20, "the thinned spline follows the path within its tolerance");
      }
      // A long prediction flows in chunks of 50,000 steps and a month: 120
      // days of low orbit is several, which must join without a gap or a
      // doubled point or apsis.
      {
        // 2% faster than circular: e ≈ 0.02, one clear periapsis per orbit.
        principia_ksa_QP const eccentric{
            iss.q, {earth0.p.x + 1.02 * (iss.p.x - earth0.p.x),
                    earth0.p.y + 1.02 * (iss.p.y - earth0.p.y),
                    earth0.p.z + 1.02 * (iss.p.z - earth0.p.z)}};
        auto const start = std::chrono::steady_clock::now();
        Prediction const long_run =
            Predict(e, &eccentric, t0, t0 + 120 * 86400.0, 1'000'000, 10, {1});
        double const seconds = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - start).count();
        Expect(long_run.status == PRINCIPIA_KSA_OK &&
                   long_run.end == PRINCIPIA_KSA_PREDICTION_REACHED_END &&
                   long_run.steps > 50'000,
               "a 120-day low-orbit prediction reaches its end over several chunks");
        bool joined = long_run.times[0] == t0 &&
                      long_run.times[long_run.count() - 1] == t0 + 120 * 86400.0;
        for (int i = 1; i < long_run.count(); ++i) {
          joined &= long_run.times[i] > long_run.times[i - 1];
        }
        // Periapsides one orbit apart, never doubled or skipped.
        double previous = -1, shortest = 1e30, longest = 0;
        int peri = 0;
        for (auto const& a : long_run.apsides) {
          if (!a.is_apoapsis) {
            if (previous >= 0) {
              shortest = std::fmin(shortest, a.time - previous);
              longest = std::fmax(longest, a.time - previous);
            }
            previous = a.time;
            ++peri;
          }
        }
        std::printf("  120 days in %.2f s: %lld steps, %d points, %d periapsides "
                    "%.1f–%.1f min apart\n",
                    seconds, static_cast<long long>(long_run.steps),
                    long_run.count(), peri, shortest / 60, longest / 60);
        Expect(joined, "chunks join without gaps or doubled points");
        Expect(peri > 1700 && shortest > 85 * 60 && longest < 105 * 60,
               "chunks join without lost or doubled apsides");
      }
      // No time limit: a prediction runs until its steps run out, and
      // prolongs the ephemeris only a month or so beyond where it got.
      {
        double t_min, t_max_before, t_max_after;
        principia_ksa__EphemerisTimeRange(e, &t_min, &t_max_before);
        Prediction const open = Predict(e, &iss, t0, infinity, 2000);
        principia_ksa__EphemerisTimeRange(e, &t_min, &t_max_after);
        double const last = open.times[open.count() - 1];
        std::printf("  no time limit, 2000 steps: %.2f d, ephemeris to %.1f d\n",
                    (last - t0) / 86400, (t_max_after - t0) / 86400);
        Expect(open.status == PRINCIPIA_KSA_OK &&
                   open.end == PRINCIPIA_KSA_PREDICTION_MAX_STEPS &&
                   open.steps == 2000 && last > t0,
               "without a time limit, a prediction ends when its steps run out");
        Expect(t_max_after <= std::fmax(t_max_before, last + 31 * 86400.0) + 86400,
               "the ephemeris is prolonged only as far as the steps went");
      }
      // Steadiness: predictions of the same motion started 30 s apart (from
      // the 1 mm vessel track, as the game does) agree up to their
      // integration error.
      {
        principia_ksa_QP later;
        principia_ksa__FlowMassless(e, &iss, t0, t0 + 30, &later);
        Prediction const a = Predict(e, &iss, t0, t0 + 86400, 100000, 0, {1});
        Prediction const b =
            Predict(e, &later, t0 + 30, t0 + 30 + 86400, 100000, 0, {1});
        double worst_hour = 0, worst_day = 0;
        for (int i = 0; i < a.count(); ++i) {
          if (a.times[i] < t0 + 60) continue;
          double const d = Norm(Minus(HermiteAt(b, a.times[i]), a.states[i].q));
          if (a.times[i] <= t0 + 3600) worst_hour = std::fmax(worst_hour, d);
          worst_day = std::fmax(worst_day, d);
        }
        double first_ap_shift = 1e30;
        for (auto const& x : a.apsides) {
          for (auto const& y : b.apsides) {
            if (x.is_apoapsis && y.is_apoapsis && x.time > t0 + 30 &&
                x.time < t0 + 4 * 3600) {
              first_ap_shift = std::fmin(first_ap_shift, std::fabs(x.time - y.time));
            }
          }
        }
        std::printf("  predictions 30 s apart: differ by %.2f m within 1 h, %.1f m within 1 d; "
                    "apoapsis time differs by %.3f s\n", worst_hour, worst_day, first_ap_shift);
        Expect(worst_hour < 10, "successive predictions agree to metres over an hour");
      }
      // A 100 s prograde burn: 100 kN, 10 t, 30 kg/s → Δv = 3333 ln(10/7).
      {
        // 1000 AU out, where gravity (~1e-8 m/s²) can't blur the Δv.
        principia_ksa_QP deep{{1.5e14, 0, 0}, {0, 1000, 0}};
        Prediction const burn =
            Predict(e, &deep, t0, t0 + 100, 100000, 10, {}, {0, 1, 0}, 1e5, 1e4, 30);
        Prediction const coast = Predict(e, &deep, t0, t0 + 100, 100000);
        Expect(burn.status == PRINCIPIA_KSA_OK && coast.status == PRINCIPIA_KSA_OK,
               "a prediction under thrust");
        double const dv = Norm(Minus(burn.states[burn.count() - 1].p,
                                     coast.states[coast.count() - 1].p));
        double const expected = 1e5 / 30 * std::log(1e4 / (1e4 - 3000));
        std::printf("  100 s burn: Δv %.2f m/s (rocket equation %.2f m/s)\n", dv,
                    expected);
        Expect(std::fabs(dv / expected - 1) < 1e-6, "burn Δv matches the rocket equation");
        Expect(Predict(e, &iss, t0, t0 + 1000, 100000, 10, {}, iss.p, 1e5, 1e4, 30)
                       .status == PRINCIPIA_KSA_INVALID_ARGUMENT,
               "a burn that would use more than the whole mass → INVALID_ARGUMENT");
        Expect(Predict(e, &iss, t0, infinity, 100000, 10, {}, iss.p, 1e5, 1e4, 30)
                       .status == PRINCIPIA_KSA_INVALID_ARGUMENT,
               "a burn without a time limit → INVALID_ARGUMENT");
      }
      // Straight down at 1 km/s from 1000 km: hits Earth, also without a
      // time limit.
      principia_ksa_QP falling{{earth0.q.x + 7.371e6, earth0.q.y, earth0.q.z},
                               {earth0.p.x - 1000, earth0.p.y, earth0.p.z}};
      for (double const t_end : {t0 + 86400, infinity}) {
        Prediction const fall = Predict(e, &falling, t0, t_end, 100000);
        Expect(fall.status == PRINCIPIA_KSA_OK &&
                   fall.end == PRINCIPIA_KSA_PREDICTION_COLLISION &&
                   fall.count() >= 2 && fall.times[fall.count() - 1] < t0 + 3600,
               "a trajectory into Earth ends in a collision, not a crash");
        std::printf("    (stopped after %.0f s: %s)\n",
                    fall.times[fall.count() - 1] - t0, principia_ksa__LastError());
      }
      // A low orbit with too few steps for its length stops at the step
      // limit, and must not be reported as an impact.
      Prediction const short_of = Predict(e, &iss, t0, t0 + 14 * 86400, 2000);
      Expect(short_of.status == PRINCIPIA_KSA_OK &&
                 short_of.end == PRINCIPIA_KSA_PREDICTION_MAX_STEPS &&
                 short_of.count() >= 2 &&
                 short_of.times[short_of.count() - 1] < t0 + 14 * 86400,
             "a step-limited prediction ends at the step limit, not in an impact");
      std::printf("    (stopped after %.2f d)\n",
                  (short_of.times[short_of.count() - 1] - t0) / 86400);
    }

    principia_ksa_XYZ a;
    Expect(principia_ksa__EphemerisRelativeAcceleration(
               e, 1, {0, 0, 7.0e6}, t0, &a) == PRINCIPIA_KSA_OK,
           "EphemerisRelativeAcceleration");
    double const x = std::pow(6.3781363e6 / 7.0e6, 2);
    double const expected = 3.986004418e14 / 4.9e13 * (1 - 3 * 1.0826e-3 * x);
    std::printf("  relative gravity 7000 km over the pole: %.5f m/s² (J2: %.5f), "
                "horizontal (%.1e, %.1e) from tesseral terms\n",
                -a.z, expected, a.x, a.y);
    // The Sun's 6 mm/s² pull cancels; what is left sideways is Earth's
    // tesseral harmonics (C31, S31, …), ~1e-5 m/s² at the pole.
    Expect(std::fabs(-a.z / expected - 1) < 1e-4 &&
               std::fabs(a.x) < 1e-4 && std::fabs(a.y) < 1e-4,
           "relative acceleration is Earth's field; the Sun's pull cancels");
    principia_ksa__EphemerisDestroy(e);
  }

  // A vessel in low lunar orbit makes Principia evaluate high degrees of the
  // Moon's field; degree 31+ aborted the game before the model was limited.
  {
    principia_ksa_Body earth_moon[] = {
        {"Earth", 3.986004418e14},
        {"Moon", 4.9048695e12, 1.7374e6, {0, 0, 1}, {1, 0, 0}, t0, 2.6617e-6,
         "Moon"}};
    principia_ksa_QP em[] = {{{0, 0, 0}, {0, 0, 0}}, {{3.844e8, 0, 0}, {0, 1022, 0}}};
    principia_ksa_Ephemeris* e = nullptr;
    principia_ksa__EphemerisCreate(earth_moon, em, 2, t0, 2100, 1e-3, &e);
    std::printf("  Moon model: %s\n", principia_ksa__EphemerisBodyDescription(e, 1));
    principia_ksa__EphemerisProlong(e, t0 + 86400);
    double const r = 1.7374e6 + 20e3, v = std::sqrt(4.9048695e12 / r);
    principia_ksa_QP llo{{3.844e8 + r, 0, 0}, {0, 1022 + v, 0}}, after;
    Expect(principia_ksa__FlowMassless(e, &llo, t0, t0 + 3 * 3600, &after) ==
                   PRINCIPIA_KSA_OK &&
               Predict(e, &llo, t0, t0 + 3 * 3600, 10000).status ==
                   PRINCIPIA_KSA_OK,
           "20 km lunar orbit with the Moon's field (no degree-31 abort)");
    principia_ksa__EphemerisDestroy(e);
  }

  // A flow whose next-to-last step lands on t_end up to rounding ends with a
  // step below the clock's resolution, which Principia appended at the
  // repeated time and CHECKed: it aborted the game once. Far from J2000 a
  // tick is 2 ms, and a few thousand random ends are enough to hit it.
  {
    double const far = 1e13;
    principia_ksa_Body earth[] = {{"Earth", 3.986004418e14}};
    principia_ksa_QP origin[] = {{{0, 0, 0}, {0, 0, 0}}};
    principia_ksa_Ephemeris* e = nullptr;
    principia_ksa__EphemerisCreate(earth, origin, 1, far, 2100, 1e-3, &e);
    principia_ksa__EphemerisProlong(e, far + 86400);
    double const r = 6.571e6, v = std::sqrt(3.986004418e14 / r);
    principia_ksa_QP leo{{r, 0, 0}, {0, v, 0}}, after;
    bool ok = true;
    for (int i = 0; i < 50'000 && ok; ++i) {
      ok = principia_ksa__FlowMassless(e, &leo, far, far + 1500 + i * 0.0137,
                                       &after) == PRINCIPIA_KSA_OK;
    }
    for (int i = 0; i < 20'000 && ok; ++i) {
      Prediction const p = Predict(e, &leo, far, far + 1500 + i * 0.0137, 10000);
      ok = p.status == PRINCIPIA_KSA_OK &&
           p.end == PRINCIPIA_KSA_PREDICTION_REACHED_END;
    }
    Expect(ok, "70,000 flows ending a tick after their last step (no CHECK abort)");
    principia_ksa__EphemerisDestroy(e);
  }

  // Principia's RotatingBody CHECKs a nonzero spin; KSA's Sun has none. This
  // aborted the game once, so it must stay an error code.
  principia_ksa_Body still[] = {
      {"Sun", 1.32712440018e20, 6.96e8, {0, 0, 1}, {1, 0, 0}, t0, 0, "Sun"},
      {"Earth", 3.986004418e14}};
  Expect(principia_ksa__EphemerisCreate(still, states, 2, t0, 2100, 1e-3,
                                        &geo) == PRINCIPIA_KSA_INVALID_ARGUMENT,
         "zero angular frequency → INVALID_ARGUMENT (not a CHECK abort)");
  std::printf("    (%s)\n", principia_ksa__LastError());
  // Readers while another thread prolongs: the game's main thread reads the
  // range every frame, and its job threads read states, while a background
  // thread integrates the planets ahead. Principia holds its ephemeris lock
  // for a whole Prolong; one month in one go stalled a frame by 108 ms.
  std::printf("readers don't wait for a long prolongation:\n");
  {
    principia_ksa_Ephemeris* e = nullptr;
    principia_ksa__EphemerisCreate(bodies, states, n, t0, 2100, 1e-3, &e);
    double const far = t0 + 20 * year;
    std::atomic<bool> done{false};
    principia_ksa_Status prolonged = PRINCIPIA_KSA_INTERNAL;
    auto const start = std::chrono::steady_clock::now();
    std::thread prolonger([&] {
      prolonged = principia_ksa__EphemerisProlong(e, far);
      done = true;
    });
    principia_ksa_QP earth0;
    principia_ksa__EphemerisBodyState(e, 1, t0, &earth0);
    double const r_leo = 6.771e6, v_leo = std::sqrt(3.986004418e14 / r_leo);
    principia_ksa_QP const leo{{earth0.q.x + r_leo, earth0.q.y, earth0.q.z},
                               {earth0.p.x, earth0.p.y + v_leo, earth0.p.z}};
    double range_ms = 0, state_ms = 0, flow_ms = 0, acceleration_ms = 0;
    int reads = 0;
    while (!done) {
      auto const r0 = std::chrono::steady_clock::now();
      double t_min, t_max;
      principia_ksa__EphemerisTimeRange(e, &t_min, &t_max);
      auto const r1 = std::chrono::steady_clock::now();
      principia_ksa_QP earth_now;
      principia_ksa__EphemerisBodyState(e, 1, t0 + 0.5 * (t_max - t0), &earth_now);
      auto const r2 = std::chrono::steady_clock::now();
      // A vessel's step and its thrust gravity, as on KSA's job threads.
      principia_ksa_QP after;
      principia_ksa__FlowMassless(e, &leo, t0, t0 + 1, &after);
      auto const r3 = std::chrono::steady_clock::now();
      principia_ksa_XYZ g;
      principia_ksa__EphemerisRelativeAcceleration(e, 1, {r_leo, 0, 0}, t0, &g);
      auto const r4 = std::chrono::steady_clock::now();
      range_ms = std::fmax(range_ms, std::chrono::duration<double, std::milli>(r1 - r0).count());
      state_ms = std::fmax(state_ms, std::chrono::duration<double, std::milli>(r2 - r1).count());
      flow_ms = std::fmax(flow_ms, std::chrono::duration<double, std::milli>(r3 - r2).count());
      acceleration_ms =
          std::fmax(acceleration_ms, std::chrono::duration<double, std::milli>(r4 - r3).count());
      ++reads;
    }
    prolonger.join();
    double const total_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start).count();
    double t_min, t_max;
    principia_ksa__EphemerisTimeRange(e, &t_min, &t_max);
    std::printf("  20 years in %.0f ms; %d rounds of calls meanwhile, longest: range %.3f ms, "
                "state %.3f ms, vessel step %.3f ms, thrust gravity %.3f ms\n",
                total_ms, reads, range_ms, state_ms, flow_ms, acceleration_ms);
    Expect(prolonged == PRINCIPIA_KSA_OK && t_max >= far, "the prolongation reaches 20 years");
    Expect(range_ms < 2, "reading the range never waits for it (< 2 ms)");
    Expect(state_ms < 10, "reading a state waits a piece at most (< 10 ms)");
    Expect(flow_ms < 10, "a vessel step waits a piece at most (< 10 ms)");
    Expect(acceleration_ms < 10, "thrust gravity waits a piece at most (< 10 ms)");
    principia_ksa__EphemerisDestroy(e);
  }

  // The vessel's own calls while a long prediction runs: in the game a
  // vessel's step and its thrust gravity wait on KSA's job threads, and the
  // frame waits on them. A prediction reaching months ahead into planets not
  // yet integrated held them up ~110 ms a frame.
  std::printf("the vessel's calls don't wait for a long prediction:\n");
  {
    principia_ksa_Body fields[] = {
        {"Sun", 1.32712440018e20},
        {"Earth", 3.986004418e14, 6.371e6, {0, 0, 1}, {1, 0, 0}, t0,
         7.2921159e-5, "Earth"},
        {"Moon", 4.9048695e12, 1.7374e6, {0, 0, 1}, {1, 0, 0}, t0,
         2.6617e-6, "Moon"},
    };
    principia_ksa_Ephemeris* e = nullptr;
    principia_ksa__EphemerisCreate(fields, states, n, t0, 2100, 1e-3, &e);
    principia_ksa__EphemerisProlong(e, t0 + 3600);
    principia_ksa_QP earth;
    principia_ksa__EphemerisBodyState(e, 1, t0, &earth);
    // A high, eccentric orbit: 1,024 steps reach months ahead.
    double const rp = 6.771e6, ra = 3.0e8, a = 0.5 * (rp + ra);
    double const vp = std::sqrt(3.986004418e14 * (2 / rp - 1 / a));
    principia_ksa_QP const high{{earth.q.x + rp, earth.q.y, earth.q.z},
                                {earth.p.x, earth.p.y + vp, earth.p.z}};
    double const r = 6.771e6, v = std::sqrt(3.986004418e14 / r);
    principia_ksa_QP const leo{{earth.q.x, earth.q.y + r, earth.q.z},
                               {earth.p.x - v, earth.p.y, earth.p.z}};
    std::atomic<bool> done{false};
    Prediction long_prediction;
    auto const start = std::chrono::steady_clock::now();
    std::thread predictor([&] {
      long_prediction = Predict(e, &high, t0, infinity, 1 << 14);
      done = true;
    });
    double flow_ms = 0, acceleration_ms = 0;
    int calls = 0;
    bool ok = true;
    while (!done) {
      auto const c0 = std::chrono::steady_clock::now();
      principia_ksa_QP after;
      ok &= principia_ksa__FlowMassless(e, &leo, t0, t0 + 1, &after) == PRINCIPIA_KSA_OK;
      auto const c1 = std::chrono::steady_clock::now();
      principia_ksa_XYZ g;
      ok &= principia_ksa__EphemerisRelativeAcceleration(e, 1, {r, 0, 0}, t0, &g) ==
            PRINCIPIA_KSA_OK;
      auto const c2 = std::chrono::steady_clock::now();
      flow_ms = std::fmax(flow_ms, std::chrono::duration<double, std::milli>(c1 - c0).count());
      acceleration_ms =
          std::fmax(acceleration_ms, std::chrono::duration<double, std::milli>(c2 - c1).count());
      ++calls;
    }
    predictor.join();
    double const total_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - start).count();
    double const days = long_prediction.count() > 0
                            ? (long_prediction.times.back() - t0) / 86400 : 0;
    std::printf("  prediction: %.0f d in %lld steps, %.0f ms; %d vessel steps meanwhile, longest: "
                "flow %.3f ms, thrust gravity %.3f ms\n",
                days, static_cast<long long>(long_prediction.steps), total_ms, calls,
                flow_ms, acceleration_ms);
    Expect(long_prediction.status == PRINCIPIA_KSA_OK && days > 30 && ok,
           "a prediction months ahead, with vessel steps alongside");
    Expect(flow_ms < 10, "a vessel step never waits long for it (< 10 ms)");
    Expect(acceleration_ms < 10, "thrust gravity never waits long for it (< 10 ms)");
    principia_ksa__EphemerisDestroy(e);
  }

  principia_ksa__SetLogCallback(nullptr);
  std::printf(failures == 0 ? "ALL PASSED\n" : "%d FAILED\n", failures);
  return failures == 0 ? 0 : 1;
}
