// Implementation of the C API in principia_ksa.h over Principia's Ephemeris.

#include "principia_ksa.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include "absl/log/initialize.h"
#include "absl/log/internal/globals.h"
#include "absl/log/log.h"
#include "absl/log/log_entry.h"
#include "absl/log/log_sink.h"
#include "absl/log/log_sink_registry.h"
#include "absl/status/status.h"
#include "base/not_null.hpp"
#include "base/version.hpp"
#include "geometry/grassmann.hpp"
#include "geometry/instant.hpp"
#include "geometry/space.hpp"
#include "google/protobuf/text_format.h"
#include "integrators/embedded_explicit_runge_kutta_nyström_integrator.hpp"
#include "integrators/integrators.hpp"
#include "integrators/methods.hpp"
#include "integrators/symplectic_runge_kutta_nyström_integrator.hpp"
#include "ksp_plugin/frames.hpp"
#include "physics/degrees_of_freedom.hpp"
#include "physics/apsides.hpp"
#include "physics/discrete_trajectory.hpp"
#include "physics/discrete_trajectory_view.hpp"
#include "physics/ephemeris.hpp"
#include "physics/massive_body.hpp"
#include "physics/oblate_body.hpp"
#include "physics/rotating_body.hpp"
#include "quantities/named_quantities.hpp"
#include "quantities/parser.hpp"
#include "quantities/quantities.hpp"
#include "quantities/si.hpp"
#include "serialization/astronomy.pb.h"

namespace {

using namespace principia::base::_not_null;
using namespace principia::geometry::_grassmann;
using namespace principia::geometry::_instant;
using namespace principia::geometry::_space;
using namespace principia::integrators::_embedded_explicit_runge_kutta_nyström_integrator;  // NOLINT
using namespace principia::integrators::_integrators;
using namespace principia::integrators::_methods;
using namespace principia::integrators::_ordinary_differential_equations;
using namespace principia::integrators::_symplectic_runge_kutta_nyström_integrator;  // NOLINT
using namespace principia::ksp_plugin::_frames;
using namespace principia::physics::_degrees_of_freedom;
using namespace principia::physics::_apsides;
using namespace principia::physics::_discrete_trajectory;
using namespace principia::physics::_discrete_trajectory_view;
using namespace principia::physics::_ephemeris;
using namespace principia::physics::_massive_body;
using namespace principia::physics::_oblate_body;
using namespace principia::physics::_rotating_body;
using namespace principia::quantities::_named_quantities;
using namespace principia::quantities::_parser;
using namespace principia::quantities::_quantities;
using namespace principia::quantities::_si;

namespace serialization = principia::serialization;

// KSA's ecliptic axes, centred on the solar system's barycentre.
using Frame = Barycentric;

thread_local std::string last_error;

principia_ksa_Status Fail(principia_ksa_Status const status,
                          std::string message) {
  last_error = std::move(message);
  return status;
}

bool Finite(double const x) { return std::isfinite(x); }
bool Finite(principia_ksa_XYZ const& v) {
  return Finite(v.x) && Finite(v.y) && Finite(v.z);
}

// Principia's adaptive integrator, but never appending a state at the time of
// the previous one. Its last step is clipped to reach t_final; when the step
// before already landed on t_final up to rounding, the clipped step is below
// the resolution of the clock, and DiscreteTrajectory CHECKs (aborting the
// game) on the repeated time. Upstream checks a vanishing step everywhere but
// there.
class MonotonicIntegrator final
    : public AdaptiveStepSizeIntegrator<Ephemeris<Frame>::NewtonianMotionEquation> {
 public:
  using ODE = Ephemeris<Frame>::NewtonianMotionEquation;

  static MonotonicIntegrator const& Get() {
    static MonotonicIntegrator const integrator(
        EmbeddedExplicitRungeKuttaNyströmIntegrator<
            DormandالمكاوىPrince1986RKN434FM, ODE>());
    return integrator;
  }

  not_null<std::unique_ptr<Integrator<ODE>::Instance>> NewInstance(
      InitialValueProblem<ODE> const& problem,
      Integrator<ODE>::AppendState const& append_state,
      ToleranceToErrorRatio const& tolerance_to_error_ratio,
      Parameters const& parameters) const override {
    return integrator_.NewInstance(
        problem,
        [append_state, last = problem.initial_state.time.value](
            ODE::State const& state) mutable {
          if (last < state.time.value) {
            last = state.time.value;
            append_state(state);
          }
        },
        tolerance_to_error_ratio,
        parameters);
  }

  void WriteToMessage(not_null<serialization::AdaptiveStepSizeIntegrator*> const
                          message) const override {
    integrator_.WriteToMessage(message);
  }

 private:
  explicit MonotonicIntegrator(AdaptiveStepSizeIntegrator<ODE> const& integrator)
      : integrator_(integrator) {}

  AdaptiveStepSizeIntegrator<ODE> const& integrator_;
};

Instant ToInstant(double const seconds_since_j2000) {
  return Instant() + seconds_since_j2000 * Second;
}
double FromInstant(Instant const& t) { return (t - Instant()) / Second; }

Position<Frame> ToPosition(principia_ksa_XYZ const& q) {
  return Frame::origin +
         Displacement<Frame>({q.x * Metre, q.y * Metre, q.z * Metre});
}
Velocity<Frame> ToVelocity(principia_ksa_XYZ const& p) {
  return Velocity<Frame>(
      {p.x * Metre / Second, p.y * Metre / Second, p.z * Metre / Second});
}

template<typename Q>
principia_ksa_XYZ ToXYZ(Vector<Q, Frame> const& v, Q const& unit) {
  auto const c = v.coordinates();
  return {c.x / unit, c.y / unit, c.z / unit};
}
principia_ksa_QP ToQP(DegreesOfFreedom<Frame> const& dof) {
  return {ToXYZ(dof.position() - Frame::origin, Metre),
          ToXYZ(dof.velocity(), Metre / Second)};
}

// Forwards absl log messages, including CHECK failures, to the C# side.
std::atomic<principia_ksa_LogCallback> log_callback = nullptr;

class CallbackLogSink final : public absl::LogSink {
 public:
  void Send(absl::LogEntry const& entry) override {
    principia_ksa_LogCallback const callback = log_callback.load();
    if (callback != nullptr) {
      std::string const message(entry.text_message_with_prefix());
      callback(static_cast<int>(entry.log_severity()), message.c_str());
    }
  }
};

CallbackLogSink* log_sink = nullptr;

// Bodies of the loaded gravity model, by name.
std::mutex gravity_model_lock;
std::map<std::string, serialization::GravityModel::Body> gravity_model;

double Dot(principia_ksa_XYZ const& a, principia_ksa_XYZ const& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
principia_ksa_XYZ Cross(principia_ksa_XYZ const& a, principia_ksa_XYZ const& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
principia_ksa_XYZ Scale(double const k, principia_ksa_XYZ const& a) {
  return {k * a.x, k * a.y, k * a.z};
}

// A point mass, a RotatingBody, or an OblateBody with the gravity model's
// geopotential. Null, with `last_error` set, on invalid input.
std::unique_ptr<MassiveBody const> MakeBody(principia_ksa_Body const& body,
                                            std::string const& name) {
  double const μ = body.gravitational_parameter;
  if (!Finite(μ) || μ <= 0) {
    Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
         name + ": gravitational parameter must be > 0");
    return nullptr;
  }
  MassiveBody::Parameters const massive_parameters(
      name, μ * Unit<GravitationalParameter>);
  if (!Finite(body.mean_radius) || body.mean_radius <= 0) {
    return std::make_unique<MassiveBody const>(massive_parameters);
  }

  // The pole as right ascension and declination in our frame, and the prime
  // meridian as an angle from Principia's reference direction, the node of
  // the equator on the frame's xy plane at right ascension α + 90°
  // (RotatingBody::biequatorial).
  principia_ksa_XYZ const axis = body.polar_axis;
  double const axis_norm = std::sqrt(Dot(axis, axis));
  if (!Finite(axis) || !Finite(body.prime_meridian) ||
      !Finite(body.reference_time) || !Finite(body.angular_frequency) ||
      axis_norm < 0.5) {
    Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, name + ": invalid rotation");
    return nullptr;
  }
  // RotatingBody CHECKs this (rotating_body_body.hpp).
  if (body.angular_frequency == 0) {
    Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
         name + ": a rotating body needs a nonzero angular frequency");
    return nullptr;
  }
  principia_ksa_XYZ const pole = Scale(1 / axis_norm, axis);
  // Remove any component along the pole from the prime meridian.
  principia_ksa_XYZ const meridian = {
      body.prime_meridian.x - Dot(body.prime_meridian, pole) * pole.x,
      body.prime_meridian.y - Dot(body.prime_meridian, pole) * pole.y,
      body.prime_meridian.z - Dot(body.prime_meridian, pole) * pole.z};
  if (Dot(meridian, meridian) < 1e-12 * Dot(body.prime_meridian, body.prime_meridian)) {
    Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
         name + ": prime meridian is parallel to the pole");
    return nullptr;
  }
  double const α = std::atan2(pole.y, pole.x);
  double const δ = std::asin(std::clamp(pole.z, -1.0, 1.0));
  principia_ksa_XYZ const biequatorial = {-std::sin(α), std::cos(α), 0};
  double const reference_angle =
      std::atan2(Dot(Cross(biequatorial, meridian), pole),
                 Dot(biequatorial, meridian));
  RotatingBody<Frame>::Parameters const rotating_parameters(
      body.mean_radius * Metre,
      reference_angle * Radian,
      ToInstant(body.reference_time),
      body.angular_frequency * Radian / Second,
      α * Radian,
      δ * Radian);

  if (body.gravity_model_name != nullptr) {
    std::lock_guard<std::mutex> lock(gravity_model_lock);
    auto const it = gravity_model.find(body.gravity_model_name);
    if (it != gravity_model.end()) {
      serialization::GravityModel::Body const& model = it->second;
      if (model.has_reference_radius()) {
        Length const reference_radius =
            ParseQuantity<Length>(model.reference_radius());
        if (model.has_j2() && model.j2() > 0) {  // OblateBody CHECKs j2 > 0.
          return std::make_unique<OblateBody<Frame> const>(
              massive_parameters,
              rotating_parameters,
              OblateBody<Frame>::Parameters(model.j2(), reference_radius));
        }
        if (model.has_geopotential()) {
          return std::make_unique<OblateBody<Frame> const>(
              massive_parameters,
              rotating_parameters,
              OblateBody<Frame>::Parameters::ReadFromMessage(
                  model.geopotential(), reference_radius));
        }
      }
    }
  }
  return std::make_unique<RotatingBody<Frame> const>(massive_parameters,
                                                     rotating_parameters);
}

}  // namespace

struct principia_ksa_Ephemeris {
  std::unique_ptr<Ephemeris<Frame>> ephemeris;
  // Bodies in creation order; owned by `ephemeris`.
  std::vector<not_null<MassiveBody const*>> bodies;
};

namespace {

// Checks `t` against the range in which states can be evaluated; outside it,
// ContinuousTrajectory would fail a CHECK.
principia_ksa_Status CheckTime(principia_ksa_Ephemeris const* const e,
                               double const t) {
  if (!Finite(t)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "time is not finite");
  }
  Instant const t_min = e->ephemeris->t_min();
  Instant const t_max = e->ephemeris->t_max();
  Instant const instant = ToInstant(t);
  if (instant < t_min || instant > t_max) {
    return Fail(PRINCIPIA_KSA_OUT_OF_RANGE,
                "time " + std::to_string(t) + " is outside [" +
                    std::to_string(FromInstant(t_min)) + ", " +
                    std::to_string(FromInstant(t_max)) + "]");
  }
  return PRINCIPIA_KSA_OK;
}

}  // namespace

// Every entry point catches exceptions: one must never unwind into .NET.
#define PRINCIPIA_KSA_TRY try {
#define PRINCIPIA_KSA_CATCH                                          \
  }                                                                  \
  catch (std::exception const& e) {                                  \
    return Fail(PRINCIPIA_KSA_INTERNAL, e.what());                   \
  }                                                                  \
  catch (...) {                                                      \
    return Fail(PRINCIPIA_KSA_INTERNAL, "unknown C++ exception");    \
  }

extern "C" {

char const* principia_ksa__Version(void) {
  static std::string const version =
      std::string("Principia ") + principia::base::_version::Version +
      " (core built " + principia::base::_version::BuildDate +
      "), libprincipia_ksa " PLATFORM_WITH_CPU_FEATURES " built with clang " +
      __clang_version__;
  return version.c_str();
}

char const* principia_ksa__LastError(void) { return last_error.c_str(); }

principia_ksa_Status principia_ksa__LoadGravityModel(char const* const path) {
  PRINCIPIA_KSA_TRY
  if (path == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "path is null");
  }
  std::ifstream file(path);
  if (!file) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                std::string("cannot read ") + path);
  }
  std::stringstream text;
  text << file.rdbuf();
  serialization::SolarSystemFile message;
  if (!google::protobuf::TextFormat::ParseFromString(text.str(), &message) ||
      !message.has_gravity_model()) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                std::string(path) + " is not a gravity model");
  }
  std::lock_guard<std::mutex> lock(gravity_model_lock);
  gravity_model.clear();
  for (auto body : message.gravity_model().body()) {
#if !PRINCIPIA_GEOPOTENTIAL_MAX_DEGREE_50
    // As SolarSystem does: unless built for degree 50, Principia aborts on
    // geopotential degrees above 30 (the Moon's model goes to 50).
    if (body.has_geopotential()) {
      auto* const rows = body.mutable_geopotential()->mutable_row();
      for (int i = 0; i < rows->size();) {
        if (rows->Get(i).degree() > 30) {
          rows->DeleteSubrange(i, 1);
        } else {
          ++i;
        }
      }
      if (rows->empty()) {
        body.clear_geopotential();
        body.clear_reference_radius();
      }
    }
#endif
    gravity_model[body.name()] = body;
  }
  LOG(INFO) << "Loaded " << gravity_model.size() << " bodies from " << path;
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

principia_ksa_Status principia_ksa__GravityModelBody(
    char const* const name,
    double* const gravitational_parameter,
    double* const mean_radius,
    double* const right_ascension,
    double* const declination) {
  PRINCIPIA_KSA_TRY
  if (name == nullptr || gravitational_parameter == nullptr ||
      mean_radius == nullptr || right_ascension == nullptr ||
      declination == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "null argument");
  }
  std::lock_guard<std::mutex> lock(gravity_model_lock);
  auto const it = gravity_model.find(name);
  if (it == gravity_model.end()) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                std::string("the gravity model has no ") + name);
  }
  auto const& body = it->second;
  double const nan = std::numeric_limits<double>::quiet_NaN();
  *gravitational_parameter =
      ParseQuantity<GravitationalParameter>(body.gravitational_parameter()) /
      (Metre * Metre * Metre / (Second * Second));
  *mean_radius = body.has_mean_radius()
                     ? ParseQuantity<Length>(body.mean_radius()) / Metre
                     : nan;
  bool const pole =
      body.has_axis_right_ascension() && body.has_axis_declination();
  *right_ascension =
      pole ? ParseQuantity<Angle>(body.axis_right_ascension()) / Radian : nan;
  *declination =
      pole ? ParseQuantity<Angle>(body.axis_declination()) / Radian : nan;
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

void principia_ksa__SetLogCallback(principia_ksa_LogCallback const callback) {
  log_callback.store(callback);
  if (callback != nullptr && log_sink == nullptr) {
    if (!absl::log_internal::IsInitialized()) {
      absl::InitializeLog();
    }
    log_sink = new CallbackLogSink;
    absl::AddLogSink(log_sink);
    LOG(INFO) << "Principia logging routed to PrincipiaKSA";
  }
}

principia_ksa_Status principia_ksa__EphemerisCreate(
    principia_ksa_Body const* const bodies,
    principia_ksa_QP const* const states,
    int const count,
    double const t0,
    double const step,
    double const fitting_tolerance,
    principia_ksa_Ephemeris** const ephemeris) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "ephemeris is null");
  }
  *ephemeris = nullptr;
  if (bodies == nullptr || states == nullptr || count <= 0) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "no bodies");
  }
  if (!Finite(t0)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "t0 is not finite");
  }
  if (!Finite(step) || step <= 0) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "step must be > 0");
  }
  if (!Finite(fitting_tolerance) || fitting_tolerance <= 0) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                "fitting tolerance must be > 0");
  }

  std::vector<not_null<std::unique_ptr<MassiveBody const>>> massive_bodies;
  std::vector<DegreesOfFreedom<Frame>> initial_state;
  for (int i = 0; i < count; ++i) {
    std::string const name =
        bodies[i].name == nullptr ? "body " + std::to_string(i)
                                  : std::string(bodies[i].name);
    if (!Finite(states[i].q) || !Finite(states[i].p)) {
      return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                  name + ": state is not finite");
    }
    for (int j = 0; j < i; ++j) {
      if (states[i].q.x == states[j].q.x && states[i].q.y == states[j].q.y &&
          states[i].q.z == states[j].q.z) {
        return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                    name + ": same position as body " + std::to_string(j));
      }
    }
    std::unique_ptr<MassiveBody const> body = MakeBody(bodies[i], name);
    if (body == nullptr) {
      return PRINCIPIA_KSA_INVALID_ARGUMENT;  // MakeBody set last_error.
    }
    massive_bodies.emplace_back(std::move(body));
    initial_state.emplace_back(ToPosition(states[i].q),
                               ToVelocity(states[i].p));
  }

  // The same integrator as Principia's KSP plugin (ksp_plugin/integrators.cpp).
  Ephemeris<Frame>::FixedStepParameters const fixed_step_parameters(
      SymplecticRungeKuttaNyströmIntegrator<
          BlanesMoan2002SRKN14A,
          Ephemeris<Frame>::NewtonianMotionEquation>(),
      step * Second);
  Ephemeris<Frame>::AccuracyParameters const accuracy_parameters(
      fitting_tolerance * Metre,
      /*geopotential_tolerance=*/0x1.0p-24);

  auto result = std::make_unique<principia_ksa_Ephemeris>();
  result->ephemeris = std::make_unique<Ephemeris<Frame>>(
      std::move(massive_bodies),
      initial_state,
      ToInstant(t0),
      accuracy_parameters,
      fixed_step_parameters);
  result->bodies = result->ephemeris->bodies();

  // A fresh ephemeris has no polynomials yet, so nothing can be evaluated;
  // integrate a little so that `t0` itself is in range.
  absl::Status const status = result->ephemeris->Prolong(ToInstant(t0));
  if (!status.ok()) {
    return Fail(PRINCIPIA_KSA_PRINCIPIA_ERROR, std::string(status.message()));
  }
  LOG(INFO) << "Created an ephemeris of " << count << " bodies at t0 = " << t0
            << " s since J2000";
  *ephemeris = result.release();
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

void principia_ksa__EphemerisDestroy(principia_ksa_Ephemeris* const ephemeris) {
  delete ephemeris;
}

principia_ksa_Status principia_ksa__EphemerisProlong(
    principia_ksa_Ephemeris* const ephemeris,
    double const t) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "ephemeris is null");
  }
  if (!Finite(t)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "time is not finite");
  }
  absl::Status const status = ephemeris->ephemeris->Prolong(ToInstant(t));
  if (!status.ok()) {
    return Fail(PRINCIPIA_KSA_PRINCIPIA_ERROR, std::string(status.message()));
  }
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

principia_ksa_Status principia_ksa__EphemerisTimeRange(
    principia_ksa_Ephemeris const* const ephemeris,
    double* const t_min,
    double* const t_max) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr || t_min == nullptr || t_max == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "null argument");
  }
  *t_min = FromInstant(ephemeris->ephemeris->t_min());
  *t_max = FromInstant(ephemeris->ephemeris->t_max());
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

principia_ksa_Status principia_ksa__EphemerisBodyState(
    principia_ksa_Ephemeris const* const ephemeris,
    int const index,
    double const t,
    principia_ksa_QP* const state) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr || state == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "null argument");
  }
  if (index < 0 || index >= static_cast<int>(ephemeris->bodies.size())) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                "no body with index " + std::to_string(index));
  }
  if (auto const status = CheckTime(ephemeris, t);
      status != PRINCIPIA_KSA_OK) {
    return status;
  }
  *state = ToQP(ephemeris->ephemeris->trajectory(ephemeris->bodies[index])
                    ->EvaluateDegreesOfFreedom(ToInstant(t)));
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

principia_ksa_Status principia_ksa__EphemerisAllStates(
    principia_ksa_Ephemeris const* const ephemeris,
    double const t,
    principia_ksa_QP* const states,
    int const count) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr || states == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "null argument");
  }
  if (count != static_cast<int>(ephemeris->bodies.size())) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                "count is " + std::to_string(count) + ", the ephemeris has " +
                    std::to_string(ephemeris->bodies.size()) + " bodies");
  }
  if (auto const status = CheckTime(ephemeris, t);
      status != PRINCIPIA_KSA_OK) {
    return status;
  }
  Instant const instant = ToInstant(t);
  for (int i = 0; i < count; ++i) {
    states[i] = ToQP(ephemeris->ephemeris->trajectory(ephemeris->bodies[i])
                         ->EvaluateDegreesOfFreedom(instant));
  }
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

char const* principia_ksa__EphemerisBodyDescription(
    principia_ksa_Ephemeris const* const ephemeris,
    int const index) {
  thread_local std::string description;
  try {
    if (ephemeris == nullptr || index < 0 ||
        index >= static_cast<int>(ephemeris->bodies.size())) {
      description = "invalid";
    } else {
      MassiveBody const* const body = ephemeris->bodies[index];
      if (auto const oblate = dynamic_cast<OblateBody<Frame> const*>(body)) {
        std::ostringstream out;
        out << "oblate, J2 = " << oblate->j2() << ", degree "
            << oblate->geopotential_degree();
        description = out.str();
      } else if (dynamic_cast<RotatingBody<Frame> const*>(body) != nullptr) {
        description = "rotating, spherical";
      } else {
        description = "point mass";
      }
    }
  } catch (...) {
    description = "unknown";
  }
  return description.c_str();
}

principia_ksa_Status principia_ksa__EphemerisGravitationalAcceleration(
    principia_ksa_Ephemeris const* const ephemeris,
    principia_ksa_XYZ const position,
    double const t,
    principia_ksa_XYZ* const acceleration) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr || acceleration == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "null argument");
  }
  if (!Finite(position)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "position is not finite");
  }
  if (auto const status = CheckTime(ephemeris, t);
      status != PRINCIPIA_KSA_OK) {
    return status;
  }
  *acceleration = ToXYZ(
      ephemeris->ephemeris->ComputeGravitationalAccelerationOnMasslessBody(
          ToPosition(position), ToInstant(t)),
      Metre / Second / Second);
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

principia_ksa_Status principia_ksa__EphemerisRelativeAcceleration(
    principia_ksa_Ephemeris const* const ephemeris,
    int const index,
    principia_ksa_XYZ const relative_position,
    double const t,
    principia_ksa_XYZ* const acceleration) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr || acceleration == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "null argument");
  }
  if (index < 0 || index >= static_cast<int>(ephemeris->bodies.size())) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                "no body with index " + std::to_string(index));
  }
  if (!Finite(relative_position)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "position is not finite");
  }
  if (auto const status = CheckTime(ephemeris, t);
      status != PRINCIPIA_KSA_OK) {
    return status;
  }
  Instant const instant = ToInstant(t);
  auto const& e = *ephemeris->ephemeris;
  not_null<MassiveBody const*> const body = ephemeris->bodies[index];
  Position<Frame> const body_position =
      e.trajectory(body)->EvaluatePosition(instant);
  Displacement<Frame> const r({relative_position.x * Metre,
                               relative_position.y * Metre,
                               relative_position.z * Metre});
  *acceleration = ToXYZ(
      e.ComputeGravitationalAccelerationOnMasslessBody(body_position + r,
                                                       instant) -
          e.ComputeGravitationalAccelerationOnMassiveBody(body, instant),
      Metre / Second / Second);
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

principia_ksa_Status principia_ksa__FlowMassless(
    principia_ksa_Ephemeris const* const ephemeris,
    principia_ksa_QP const* const initial,
    double const t0,
    double const t1,
    principia_ksa_QP* const final_state) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr || initial == nullptr || final_state == nullptr) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "null argument");
  }
  if (!Finite(initial->q) || !Finite(initial->p)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "state is not finite");
  }
  for (double const t : {t0, t1}) {
    if (auto const status = CheckTime(ephemeris, t);
        status != PRINCIPIA_KSA_OK) {
      return status;
    }
  }
  if (t1 < t0) {
    // FlowODEWithAdaptiveStep CHECKs a positive first step.
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "t1 is before t0");
  }
  if (t1 == t0) {
    *final_state = *initial;
    return PRINCIPIA_KSA_OK;
  }
  // As Principia's DefaultPsychohistoryParameters, with a bounded step count
  // so that a pathological orbit can't hang a worker thread.
  static Ephemeris<Frame>::AdaptiveStepParameters const parameters(
      MonotonicIntegrator::Get(),
      /*max_steps=*/1'000'000,
      /*length_integration_tolerance=*/1 * Milli(Metre),
      /*speed_integration_tolerance=*/1 * Milli(Metre) / Second);
  DiscreteTrajectory<Frame> trajectory;
  absl::Status status = trajectory.Append(
      ToInstant(t0),
      DegreesOfFreedom<Frame>(ToPosition(initial->q), ToVelocity(initial->p)));
  if (!status.ok()) {
    return Fail(PRINCIPIA_KSA_PRINCIPIA_ERROR, std::string(status.message()));
  }
  status = ephemeris->ephemeris->FlowWithAdaptiveStep(
      &trajectory,
      Ephemeris<Frame>::NoIntrinsicAcceleration,
      ToInstant(t1),
      parameters,
      /*max_ephemeris_steps=*/0);
  if (!status.ok()) {
    return Fail(PRINCIPIA_KSA_PRINCIPIA_ERROR, std::string(status.message()));
  }
  *final_state = ToQP(trajectory.back().degrees_of_freedom);
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

principia_ksa_Status principia_ksa__Predict(
    principia_ksa_Ephemeris const* const ephemeris,
    principia_ksa_QP const* const initial,
    double const t0,
    double const t_end,
    std::int64_t const max_steps,
    double const length_tolerance,
    double const speed_tolerance,
    principia_ksa_XYZ const thrust_direction,
    double const thrust,
    double const initial_mass,
    double const mass_flow,
    double const downsampling_tolerance,
    int const* const apsis_bodies,
    int const apsis_body_count,
    int const apsis_capacity,
    double** const times,
    principia_ksa_QP** const states,
    int* const count,
    principia_ksa_Apsis** const apsides,
    int* const apsis_count,
    std::int64_t* const steps,
    principia_ksa_PredictionEnd* const end) {
  PRINCIPIA_KSA_TRY
  if (ephemeris == nullptr || initial == nullptr || times == nullptr ||
      states == nullptr || count == nullptr || apsides == nullptr ||
      apsis_count == nullptr || steps == nullptr || end == nullptr ||
      (apsis_body_count > 0 && apsis_bodies == nullptr)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "null argument");
  }
  *times = nullptr;
  *states = nullptr;
  *apsides = nullptr;
  *count = 0;
  *apsis_count = 0;
  *steps = 0;
  if (max_steps < 1 || apsis_body_count < 0 || apsis_capacity < 0 ||
      !Finite(length_tolerance) || !(length_tolerance > 0) ||
      !Finite(speed_tolerance) || !(speed_tolerance > 0) ||
      !Finite(downsampling_tolerance) || downsampling_tolerance < 0) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                "need max_steps ≥ 1, tolerances > 0, a downsampling "
                "tolerance ≥ 0");
  }
  for (int b = 0; b < apsis_body_count; ++b) {
    if (apsis_bodies[b] < 0 ||
        apsis_bodies[b] >= static_cast<int>(ephemeris->bodies.size())) {
      return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                  "no body with index " + std::to_string(apsis_bodies[b]));
    }
  }
  if (!Finite(initial->q) || !Finite(initial->p) || std::isnan(t_end)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "state or time not finite");
  }
  if (auto const status = CheckTime(ephemeris, t0);
      status != PRINCIPIA_KSA_OK) {
    return status;
  }
  if (!(t_end > t0)) {
    return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT, "t_end must be after t0");
  }

  Ephemeris<Frame>::IntrinsicAcceleration intrinsic_acceleration =
      Ephemeris<Frame>::NoIntrinsicAcceleration;
  if (thrust != 0) {
    double const norm = std::sqrt(Dot(thrust_direction, thrust_direction));
    if (!Finite(thrust) || thrust < 0 || !Finite(thrust_direction) ||
        !(norm > 0) || !Finite(initial_mass) || !(initial_mass > 0) ||
        !Finite(mass_flow) || mass_flow < 0 || !Finite(t_end) ||
        !(initial_mass - mass_flow * (t_end - t0) > 0)) {
      return Fail(PRINCIPIA_KSA_INVALID_ARGUMENT,
                  "invalid thrust, direction, mass or mass flow (t_end must "
                  "be finite and the mass stay positive until it)");
    }
    Vector<double, Frame> const direction(
        {thrust_direction.x / norm, thrust_direction.y / norm,
         thrust_direction.z / norm});
    Force const force = thrust * Newton;
    Mass const m0 = initial_mass * Kilogram;
    auto const dm_dt = mass_flow * Kilogram / Second;
    Instant const start = ToInstant(t0);
    intrinsic_acceleration = [=](Instant const& t) {
      return (force / (m0 - dm_dt * (t - start))) * direction;
    };
  }

  // The flow goes in chunks of at most a month and `chunk_steps` steps: the
  // ephemeris is prolonged only as far as the steps go. Each chunk keeps every
  // step (for the step count and exact apsides), then is copied into the
  // thinned trajectory returned and forgotten but for its last point, where
  // the next one starts. Apsides lie between successive points, so none is
  // lost at a boundary.
  constexpr std::int64_t chunk_steps = 50'000;
  Time const chunk_span = 30 * Day;
  DegreesOfFreedom<Frame> const initial_degrees_of_freedom(
      ToPosition(initial->q), ToVelocity(initial->p));
  DiscreteTrajectory<Frame> chunk;
  DiscreteTrajectory<Frame> thinned;
  if (downsampling_tolerance > 0) {
    thinned.segments().begin()->SetDownsampling(
        {.tolerance = downsampling_tolerance * Metre});
  }
  for (auto* const trajectory : {&chunk, &thinned}) {
    if (absl::Status const status =
            trajectory->Append(ToInstant(t0), initial_degrees_of_freedom);
        !status.ok()) {
      return Fail(PRINCIPIA_KSA_PRINCIPIA_ERROR, std::string(status.message()));
    }
  }

  // Apsides relative to each requested body.
  std::vector<principia_ksa_Apsis> found;
  auto find_apsides = [&]() {
    if (chunk.size() < 3) {
      return;
    }
    DiscreteTrajectoryView<Frame> const view(&chunk);
    for (int b = 0; b < apsis_body_count &&
                    static_cast<int>(found.size()) < apsis_capacity;
         ++b) {
      auto const& reference =
          *ephemeris->ephemeris->trajectory(ephemeris->bodies[apsis_bodies[b]]);
      DistinguishedPoints<Frame> apoapsides, periapsides;
      ComputeApsides(reference, view,
                     apsis_capacity - static_cast<int>(found.size()),
                     apoapsides, periapsides);
      for (int kind = 0; kind < 2; ++kind) {
        for (auto const& [time, dof] : kind == 0 ? periapsides : apoapsides) {
          if (static_cast<int>(found.size()) == apsis_capacity) {
            break;
          }
          found.push_back(
              {apsis_bodies[b], kind, FromInstant(time),
               (dof.position() - reference.EvaluatePosition(time)).Norm() /
                   Metre,
               ToQP(dof)});
        }
      }
    }
  };

  // How Principia's flow ends a chunk: kResourceExhausted when the step limit
  // is reached; OK at its target, or short of it on hitting a body (its
  // kOutOfRange is swallowed); kFailedPrecondition on falling into a centre.
  Instant const final_time =
      Finite(t_end) ? ToInstant(t_end) : InfiniteFuture;
  std::int64_t steps_left = max_steps;
  absl::Status status;
  for (;;) {
    Instant const from = chunk.back().time;
    Instant const target = final_time - from < chunk_span ? final_time
                                                          : from + chunk_span;
    Ephemeris<Frame>::AdaptiveStepParameters const parameters(
        MonotonicIntegrator::Get(),
        std::min(steps_left, chunk_steps),
        length_tolerance * Metre,
        speed_tolerance * Metre / Second);
    status = ephemeris->ephemeris->FlowWithAdaptiveStep(
        &chunk, intrinsic_acceleration, target, parameters);
    std::int64_t const taken = static_cast<std::int64_t>(chunk.size()) - 1;
    *steps += taken;
    steps_left -= taken;
    for (auto it = std::next(chunk.begin()); it != chunk.end(); ++it) {
      if (absl::Status const appended =
              thinned.Append(it->time, it->degrees_of_freedom);
          !appended.ok()) {
        return Fail(PRINCIPIA_KSA_PRINCIPIA_ERROR,
                    std::string(appended.message()));
      }
    }
    find_apsides();
    bool const at_target = chunk.back().time == target;
    if (status.ok() && at_target && target == final_time) {
      *end = PRINCIPIA_KSA_PREDICTION_REACHED_END;
      break;
    }
    bool const continues = (status.ok() && at_target) ||
                           absl::IsResourceExhausted(status);
    if (!continues || chunk.back().time == from) {
      if (status.ok() || absl::IsFailedPrecondition(status)) {
        *end = PRINCIPIA_KSA_PREDICTION_COLLISION;
        last_error = status.ok() ? "collision with a body"
                                 : std::string(status.message());
      } else {
        *end = PRINCIPIA_KSA_PREDICTION_MAX_STEPS;  // E.g. cancelled.
        last_error = std::string(status.message());
      }
      break;
    }
    if (steps_left <= 0) {
      *end = PRINCIPIA_KSA_PREDICTION_MAX_STEPS;
      break;
    }
    chunk.ForgetBefore(chunk.back().time);
  }

  // Into arrays the caller releases with principia_ksa__Free.
  int const n = static_cast<int>(thinned.size());
  *times = static_cast<double*>(std::malloc(sizeof(double) * n));
  *states = static_cast<principia_ksa_QP*>(
      std::malloc(sizeof(principia_ksa_QP) * n));
  if (*times == nullptr || *states == nullptr) {
    std::free(*times);
    std::free(*states);
    *times = nullptr;
    *states = nullptr;
    return Fail(PRINCIPIA_KSA_PRINCIPIA_ERROR, "out of memory");
  }
  int i = 0;
  for (auto const& [time, dof] : thinned) {
    (*times)[i] = FromInstant(time);
    (*states)[i] = ToQP(dof);
    ++i;
  }
  *count = n;
  if (!found.empty()) {
    *apsides = static_cast<principia_ksa_Apsis*>(
        std::malloc(sizeof(principia_ksa_Apsis) * found.size()));
    if (*apsides == nullptr) {
      return Fail(PRINCIPIA_KSA_PRINCIPIA_ERROR, "out of memory");
    }
    std::copy(found.begin(), found.end(), *apsides);
    *apsis_count = static_cast<int>(found.size());
  }
  return PRINCIPIA_KSA_OK;
  PRINCIPIA_KSA_CATCH
}

void principia_ksa__Free(void* const array) { std::free(array); }

}  // extern "C"
