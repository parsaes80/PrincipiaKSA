// Exercises libprincipia_ksa.so through its C API only, as C# does, with a
// Sun–Earth–Moon system. Run with `zig build test`.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <numbers>

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

// A prediction on a grid fitting `capacity` points, without apsides.
principia_ksa_Status Predict(principia_ksa_Ephemeris const* const e,
                             principia_ksa_QP const* const initial,
                             double const t0, double const t1,
                             int const max_steps, double* const times,
                             principia_ksa_QP* const states, int const capacity,
                             int* const count, principia_ksa_PredictionEnd* const end,
                             principia_ksa_XYZ const direction = {0, 0, 0},
                             double const thrust = 0, double const mass = 0,
                             double const mass_flow = 0) {
  int apsides = 0;
  return principia_ksa__PredictMasslessDetailed(
      e, initial, t0, t1, max_steps, direction, thrust, mass, mass_flow,
      (t1 - t0) / (capacity - 2), times, states, capacity, count, nullptr, 0,
      nullptr, 0, &apsides, end);
}

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
      static double times[500];
      static principia_ksa_QP points[500];
      int count = 0;
      principia_ksa_PredictionEnd end;
      auto const t = std::chrono::steady_clock::now();
      Expect(Predict(e, &iss, t0, t0 + 86400, 100000, times, points, 500, &count,
                     &end) == PRINCIPIA_KSA_OK &&
                 end == PRINCIPIA_KSA_PREDICTION_REACHED_END && count > 400 &&
                 count <= 500 && times[0] == t0 && times[count - 1] == t0 + 86400,
             "a day's prediction fills the buffer, first and last kept");
      std::printf("  prediction of a day in %.1f ms\n",
                  std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t).count());
      // Grid sampling and exact apsides relative to Earth.
      {
        static double gt[2000];
        static principia_ksa_QP gp[2000];
        static principia_ksa_Apsis aps[200];
        int gn = 0, an = 0, earth_index = 1;
        principia_ksa_PredictionEnd ge;
        Expect(principia_ksa__PredictMasslessDetailed(
                   e, &iss, t0 + 7, t0 + 7 + 86400, 100000, {0, 0, 0}, 0, 0, 0,
                   64, gt, gp, 2000, &gn, &earth_index, 1, aps, 200, &an,
                   &ge) == PRINCIPIA_KSA_OK,
               "PredictMasslessDetailed");
        bool on_grid = gt[0] == t0 + 7 && gt[gn - 1] == t0 + 7 + 86400;
        for (int i = 1; i < gn - 1; ++i) {
          on_grid &= std::fmod(gt[i], 64) == 0;
        }
        int peri = 0, apo = 0;
        double min_peri = 1e30, max_apo = 0;
        for (int i = 0; i < an; ++i) {
          if (aps[i].is_apoapsis) {
            ++apo;
            max_apo = std::fmax(max_apo, aps[i].distance);
          } else {
            ++peri;
            min_peri = std::fmin(min_peri, aps[i].distance);
          }
        }
        std::printf("  %d points on a 64 s grid; %d periapsides (lowest %.1f km), "
                    "%d apoapsides (highest %.1f km)\n",
                    gn, peri, min_peri / 1000, apo, max_apo / 1000);
        Expect(on_grid && gn == 1352, "sample points sit on the absolute time grid");
        Expect(peri >= 14 && peri <= 17 && apo >= 14 && apo <= 17,
               "about one periapsis and one apoapsis per 92 min orbit");
      }
      // A long prediction flows in chunks of 50,000 steps: 120 days of low
      // orbit is several chunks, which must join without a gap or a doubled
      // point or apsis.
      {
        static double lt[10000];
        static principia_ksa_QP lp[10000];
        static principia_ksa_Apsis la[5000];
        int ln = 0, lan = 0, earth_index = 1;
        principia_ksa_PredictionEnd le;
        double const span = 120 * 86400.0, grid = 2048;
        // 2% faster than circular: e ≈ 0.02, one clear periapsis per orbit.
        principia_ksa_QP const eccentric{
            iss.q, {earth0.p.x + 1.02 * (iss.p.x - earth0.p.x),
                    earth0.p.y + 1.02 * (iss.p.y - earth0.p.y),
                    earth0.p.z + 1.02 * (iss.p.z - earth0.p.z)}};
        auto const start = std::chrono::steady_clock::now();
        Expect(principia_ksa__PredictMasslessDetailed(
                   e, &eccentric, t0, t0 + span, 1'000'000, {0, 0, 0}, 0, 0, 0, grid,
                   lt, lp, 10000, &ln, &earth_index, 1, la, 5000, &lan,
                   &le) == PRINCIPIA_KSA_OK &&
                   le == PRINCIPIA_KSA_PREDICTION_REACHED_END,
               "a 120-day low-orbit prediction reaches its end");
        double const seconds = std::chrono::duration<double>(
                                   std::chrono::steady_clock::now() - start).count();
        // t0, the multiples of the grid strictly in between, t0 + span.
        int const multiples =
            static_cast<int>(std::ceil((t0 + span) / grid) - std::floor(t0 / grid)) - 1;
        bool grid_ok = ln == multiples + 2 && lt[0] == t0 && lt[ln - 1] == t0 + span;
        for (int i = 1; i < ln - 1; ++i) {
          grid_ok &= lt[i] == lt[i - 1] + (i == 1 ? lt[1] - lt[0] : grid) &&
                     std::fmod(lt[i], grid) == 0;
        }
        // Periapsides one orbit apart, never doubled or skipped.
        double previous = -1, shortest = 1e30, longest = 0;
        int peri = 0;
        for (int i = 0; i < lan; ++i) {
          if (!la[i].is_apoapsis) {
            if (previous >= 0) {
              shortest = std::fmin(shortest, la[i].time - previous);
              longest = std::fmax(longest, la[i].time - previous);
            }
            previous = la[i].time;
            ++peri;
          }
        }
        std::printf("  120 days in %.2f s: %d points, %d periapsides %.1f–%.1f min apart\n",
                    seconds, ln, peri, shortest / 60, longest / 60);
        Expect(grid_ok, "chunks join on the grid without gaps or doubled points");
        Expect(peri > 1700 && shortest > 85 * 60 && longest < 105 * 60,
               "chunks join without lost or doubled apsides");
      }
      // Steadiness: predictions of the same motion started 30 s apart (from
      // the 1 mm vessel track, as the game does) agree at common times up to
      // their integration error.
      {
        static double at[4000], bt[4000];
        static principia_ksa_QP ap[4000], bp[4000];
        static principia_ksa_Apsis aa[100], ba[100];
        int an = 0, bn = 0, aan = 0, ban = 0, earth_index = 1;
        principia_ksa_PredictionEnd ae, be;
        principia_ksa_QP later;
        principia_ksa__FlowMassless(e, &iss, t0, t0 + 30, &later);
        principia_ksa__PredictMasslessDetailed(e, &iss, t0, t0 + 86400, 100000, {0, 0, 0}, 0, 0, 0, 64,
                                               at, ap, 4000, &an, &earth_index, 1, aa, 100, &aan, &ae);
        principia_ksa__PredictMasslessDetailed(e, &later, t0 + 30, t0 + 30 + 86400, 100000, {0, 0, 0}, 0,
                                               0, 0, 64, bt, bp, 4000, &bn, &earth_index, 1, ba, 100, &ban,
                                               &be);
        double worst_hour = 0, worst_day = 0;
        for (int i = 0, j = 0; i < an; ++i) {
          while (j < bn && bt[j] < at[i]) ++j;
          if (j < bn && bt[j] == at[i]) {
            double const d = Norm(Minus(ap[i].q, bp[j].q));
            if (at[i] <= t0 + 3600) worst_hour = std::fmax(worst_hour, d);
            worst_day = std::fmax(worst_day, d);
          }
        }
        double first_ap_shift = 1e30;
        for (int i = 0; i < aan; ++i) {
          for (int j = 0; j < ban; ++j) {
            if (aa[i].is_apoapsis && ba[j].is_apoapsis && aa[i].time > t0 + 30 && aa[i].time < t0 + 4 * 3600) {
              first_ap_shift = std::fmin(first_ap_shift, std::fabs(aa[i].time - ba[j].time));
            }
          }
        }
        std::printf("  predictions 30 s apart: differ by %.2f m within 1 h, %.1f m within 1 d; "
                    "apoapsis time differs by %.3f s\n", worst_hour, worst_day, first_ap_shift);
        Expect(worst_hour < 10, "successive predictions agree to metres over an hour");

        // What 0.17 m/s of start error (KSA's conic just before a rebuild with
        // a 1 m position tolerance, in LEO under J2) does to the apsides.
        principia_ksa_QP nudged = iss;
        double const v = Norm(iss.p);
        nudged.p.y += 0.17 * iss.p.y / v;
        nudged.p.z += 0.17 * iss.p.z / v;
        principia_ksa__PredictMasslessDetailed(e, &nudged, t0, t0 + 86400, 100000, {0, 0, 0}, 0, 0, 0, 64,
                                               bt, bp, 4000, &bn, &earth_index, 1, ba, 100, &ban, &be);
        auto first = [](principia_ksa_Apsis const* a, int n, int apo) {
          for (int i = 0; i < n; ++i) if (a[i].is_apoapsis == apo) return a[i];
          return a[0];
        };
        auto const a0 = first(aa, aan, 1), a1 = first(ba, ban, 1);
        auto const p0 = first(aa, aan, 0), p1 = first(ba, ban, 0);
        std::printf("  0.17 m/s more at the start: first Ap moves %.0f m in altitude and %.0f s in time, "
                    "first Pe %.0f m and %.0f s\n",
                    a1.distance - a0.distance, a1.time - a0.time, p1.distance - p0.distance,
                    p1.time - p0.time);
      }
      // A 100 s prograde burn: 100 kN, 10 t, 30 kg/s → Δv = 3333 ln(10/7).
      {
        static double bt[50], ct[50];
        static principia_ksa_QP bp[50], cp[50];
        int bn = 0, cn = 0;
        principia_ksa_PredictionEnd be, ce;
        // 1000 AU out, where gravity (~1e-8 m/s²) can't blur the Δv.
        principia_ksa_QP deep{{1.5e14, 0, 0}, {0, 1000, 0}};
        Expect(Predict(e, &deep, t0, t0 + 100, 100000, bt, bp, 50, &bn, &be,
                       {0, 1, 0}, 1e5, 1e4, 30) == PRINCIPIA_KSA_OK &&
                   Predict(e, &deep, t0, t0 + 100, 100000, ct, cp, 50, &cn, &ce) ==
                       PRINCIPIA_KSA_OK,
               "a prediction under thrust");
        double const dv = Norm(Minus(bp[bn - 1].p, cp[cn - 1].p));
        double const expected = 1e5 / 30 * std::log(1e4 / (1e4 - 3000));
        std::printf("  100 s burn: Δv %.2f m/s (rocket equation %.2f m/s)\n", dv,
                    expected);
        Expect(std::fabs(dv / expected - 1) < 1e-6, "burn Δv matches the rocket equation");
        Expect(Predict(e, &iss, t0, t0 + 1000, 100000, bt, bp, 50, &bn, &be, iss.p,
                       1e5, 1e4, 30) == PRINCIPIA_KSA_INVALID_ARGUMENT,
               "a burn that would use more than the whole mass → INVALID_ARGUMENT");
      }
      // Straight down at 1 km/s from 1000 km: hits Earth.
      principia_ksa_QP falling{{earth0.q.x + 7.371e6, earth0.q.y, earth0.q.z},
                               {earth0.p.x - 1000, earth0.p.y, earth0.p.z}};
      Expect(Predict(e, &falling, t0, t0 + 86400, 100000, times, points, 500,
                     &count, &end) == PRINCIPIA_KSA_OK &&
                 end == PRINCIPIA_KSA_PREDICTION_COLLISION && count >= 2 &&
                 times[count - 1] < t0 + 3600,
             "a trajectory into Earth ends in a collision, not a crash");
      std::printf("    (stopped after %.0f s: %s)\n", times[count - 1] - t0,
                  principia_ksa__LastError());
      // A low orbit with too few steps for its length stops at the step
      // limit, and must not be reported as an impact.
      Expect(Predict(e, &iss, t0, t0 + 14 * 86400, 2000, times, points, 500,
                     &count, &end) == PRINCIPIA_KSA_OK &&
                 end == PRINCIPIA_KSA_PREDICTION_MAX_STEPS && count >= 2 &&
                 times[count - 1] < t0 + 14 * 86400,
             "a step-limited prediction ends at the step limit, not in an impact");
      std::printf("    (stopped after %.2f d: %s)\n", (times[count - 1] - t0) / 86400,
                  principia_ksa__LastError());
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
    static double times[100];
    static principia_ksa_QP points[100];
    int count;
    principia_ksa_PredictionEnd end;
    Expect(principia_ksa__FlowMassless(e, &llo, t0, t0 + 3 * 3600, &after) ==
                   PRINCIPIA_KSA_OK &&
               Predict(e, &llo, t0, t0 + 3 * 3600, 10000, times, points, 100,
                       &count, &end) == PRINCIPIA_KSA_OK,
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
    static double times[16];
    static principia_ksa_QP points[16];
    int count;
    principia_ksa_PredictionEnd end;
    for (int i = 0; i < 20'000 && ok; ++i) {
      ok = Predict(e, &leo, far, far + 1500 + i * 0.0137, 10000, times, points,
                   16, &count, &end) == PRINCIPIA_KSA_OK &&
           end == PRINCIPIA_KSA_PREDICTION_REACHED_END;
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
  principia_ksa__SetLogCallback(nullptr);
  std::printf(failures == 0 ? "ALL PASSED\n" : "%d FAILED\n", failures);
  return failures == 0 ? 0 : 1;
}
