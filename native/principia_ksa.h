// The C API of libprincipia_ksa.so: what PrincipiaKSA.dll (C#) calls.
//
// Conventions for every function:
//  * Frame: KSA's ecliptic frame (Ecl), right-handed, X = J2000 equinox,
//    Z = ecliptic north. Principia calls it `Barycentric`; the C# side shifts
//    the origin to the system's centre of mass before sending states.
//  * Units: SI. Metres, metres per second, m³/s² for μ, m/s² for accelerations.
//  * Time: seconds since J2000.0 (TT), i.e. Principia's `Instant`. See
//    GameTime.cs for the conversion from KSA's UniverseTime.
//  * Errors: functions return a principia_ksa_Status. On failure,
//    principia_ksa__LastError() describes the problem (per thread). Inputs are
//    validated before they reach Principia, whose failed CHECKs would abort
//    the game.
//  * Threads: Prolong takes a lock inside Principia; reading states and
//    accelerations is safe from several threads at once, for times within
//    [t_min, t_max].

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#  if defined(PRINCIPIA_KSA_BUILD)
#    define PRINCIPIA_KSA_API __declspec(dllexport)
#  else
#    define PRINCIPIA_KSA_API
#  endif
#else
#  define PRINCIPIA_KSA_API __attribute__((visibility("default")))
#endif

typedef struct principia_ksa_XYZ {
  double x;
  double y;
  double z;
} principia_ksa_XYZ;

// A state: position q and velocity p, as in Principia's interface.
typedef struct principia_ksa_QP {
  principia_ksa_XYZ q;
  principia_ksa_XYZ p;
} principia_ksa_QP;

typedef struct principia_ksa_Body {
  char const* name;                // UTF-8, copied; may be null.
  double gravitational_parameter;  // μ = G·M in m³/s², must be > 0.

  // Rotation, as the game draws it. With mean_radius ≤ 0 the body is a point
  // mass and the rest of this struct is ignored.
  double mean_radius;                // m
  principia_ksa_XYZ polar_axis;      // Unit vector, north pole (spin axis).
  principia_ksa_XYZ prime_meridian;  // Longitude 0 on the equator, at
  double reference_time;             // this time (s since J2000).
  double angular_frequency;          // rad/s about polar_axis; may be < 0
                                     // but not 0 (give non-rotating bodies a
                                     // negligible rate).

  // Name of this body in the gravity model loaded with
  // principia_ksa__LoadGravityModel (e.g. "Moon" for KSA's Luna), whose
  // geopotential (J2 and beyond) is then used. Null or unknown: spherical.
  char const* gravity_model_name;
} principia_ksa_Body;

typedef enum principia_ksa_Status {
  PRINCIPIA_KSA_OK = 0,
  PRINCIPIA_KSA_INVALID_ARGUMENT = 1,  // Bad input: null, NaN, μ ≤ 0, ...
  PRINCIPIA_KSA_OUT_OF_RANGE = 2,      // Time outside [t_min, t_max].
  PRINCIPIA_KSA_PRINCIPIA_ERROR = 3,   // Principia returned an error status.
  PRINCIPIA_KSA_INTERNAL = 4,          // A C++ exception was caught.
} principia_ksa_Status;

typedef struct principia_ksa_Ephemeris principia_ksa_Ephemeris;

// severity: 0 info, 1 warning, 2 error, 3 fatal (the process aborts after).
typedef void (*principia_ksa_LogCallback)(int severity, char const* message);

// Version of Principia and of the compiler, for the log.
PRINCIPIA_KSA_API char const* principia_ksa__Version(void);

// The message for the last failed call on this thread.
PRINCIPIA_KSA_API char const* principia_ksa__LastError(void);

// Routes Principia's log (absl LOG and CHECK messages) to `callback`, which
// may be called from any thread. Pass null to stop.
PRINCIPIA_KSA_API void principia_ksa__SetLogCallback(
    principia_ksa_LogCallback callback);

// Loads a Principia gravity model file (astronomy/*_gravity_model.proto.txt)
// for the geopotentials of later EphemerisCreate calls. Only the
// geopotential coefficients and their reference radius are taken from it;
// μ, radius and rotation come from the game.
PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__LoadGravityModel(
    char const* path);

// The gravity model's body `name`: μ (m³/s²), mean radius (m) and pole (ICRS
// right ascension and declination, radians), NaN where the model has none;
// INVALID_ARGUMENT if the model has no such body.
PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__GravityModelBody(
    char const* name,
    double* gravitational_parameter,
    double* mean_radius,
    double* right_ascension,
    double* declination);

// Creates an ephemeris of `count` massive bodies with the given states at
// time `t0`. `step` is the integration step in seconds (Principia's KSP
// plugin uses 35 min = 2100 s); `fitting_tolerance` is the Чебышёв fit
// tolerance in metres (1e-3). On success, `*ephemeris` is valid at `t0`.
PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__EphemerisCreate(
    principia_ksa_Body const* bodies,
    principia_ksa_QP const* states,
    int count,
    double t0,
    double step,
    double fitting_tolerance,
    principia_ksa_Ephemeris** ephemeris);

PRINCIPIA_KSA_API void principia_ksa__EphemerisDestroy(
    principia_ksa_Ephemeris* ephemeris);

// Integrates the bodies until at least time `t`. Cost is proportional to
// (t − t_max) / step; earlier times return at once.
PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__EphemerisProlong(
    principia_ksa_Ephemeris* ephemeris,
    double t);

PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__EphemerisTimeRange(
    principia_ksa_Ephemeris const* ephemeris,
    double* t_min,
    double* t_max);

// State of body `index` (in creation order) at time `t` in [t_min, t_max].
PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__EphemerisBodyState(
    principia_ksa_Ephemeris const* ephemeris,
    int index,
    double t,
    principia_ksa_QP* state);

// States of all `count` bodies at time `t`, into a caller-owned array.
PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__EphemerisAllStates(
    principia_ksa_Ephemeris const* ephemeris,
    double t,
    principia_ksa_QP* states,
    int count);

// How body `index` is modelled, for the log: "point mass", "rotating", or
// "oblate, J2 = …, degree n". Valid until the next call on this thread.
PRINCIPIA_KSA_API char const* principia_ksa__EphemerisBodyDescription(
    principia_ksa_Ephemeris const* ephemeris,
    int index);

// The n-body gravitational acceleration at `position` at time `t`.
PRINCIPIA_KSA_API principia_ksa_Status
principia_ksa__EphemerisGravitationalAcceleration(
    principia_ksa_Ephemeris const* ephemeris,
    principia_ksa_XYZ position,
    double t,
    principia_ksa_XYZ* acceleration);

// The gravitational acceleration at `relative_position` from body `index`,
// minus the acceleration of that body itself: the acceleration of a point
// relative to the body, as seen from a frame that moves with the body (KSA's
// parent-centred frames). It includes the body's own gravity with its
// geopotential, and the tides of every other body.
PRINCIPIA_KSA_API principia_ksa_Status
principia_ksa__EphemerisRelativeAcceleration(
    principia_ksa_Ephemeris const* ephemeris,
    int index,
    principia_ksa_XYZ relative_position,
    double t,
    principia_ksa_XYZ* acceleration);

// Integrates a massless body (a vessel) from `initial` at `t0` to `t1` in the
// gravitational field of the ephemeris, with Principia's KSP settings for
// vessel histories (Dormand–El-Mikkawy–Prince RKN 4(3)FM, 1 mm, 1 mm/s).
// Both times must be in [t_min, t_max] and t1 ≥ t0. Fails (status 3) if the
// body hits a massive body or the integration can't reach t1.
PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__FlowMassless(
    principia_ksa_Ephemeris const* ephemeris,
    principia_ksa_QP const* initial,
    double t0,
    double t1,
    principia_ksa_QP* final_state);

// Why a prediction ended.
typedef enum principia_ksa_PredictionEnd {
  PRINCIPIA_KSA_PREDICTION_REACHED_END = 0,  // Got to t_end.
  PRINCIPIA_KSA_PREDICTION_MAX_STEPS = 1,    // Took all its steps first.
  PRINCIPIA_KSA_PREDICTION_COLLISION = 2,    // Hit a body (or another error).
} principia_ksa_PredictionEnd;

// An apsis of a predicted trajectory relative to a body, computed exactly on
// the full-resolution trajectory.
typedef struct principia_ksa_Apsis {
  int body;                // Index in the ephemeris.
  int is_apoapsis;         // 0: periapsis, 1: apoapsis.
  double time;             // s since J2000.
  double distance;         // m, from the body's centre.
  principia_ksa_QP state;  // Absolute.
} principia_ksa_Apsis;

// Predicts a massless body from `initial` at `t0` as Principia predicts a
// vessel: with its adaptive integrator for predictions (Dormand–El-Mikkawy–
// Prince RKN 4(3)FM) at `length_tolerance` (m) and `speed_tolerance` (m/s),
// until it has taken `max_steps` steps, reached `t_end`, or hit a body. The
// ephemeris is prolonged as the flow goes, a month at a time, so a prediction
// that runs out of steps early prolongs it no further than it needs.
// An engine may burn throughout: `thrust` (N, 0 to coast) along the fixed
// inertial `thrust_direction`, the mass falling from `initial_mass` (kg) at
// `mass_flow` (kg/s), as KSA flies a planned burn; `t_end` must then be
// finite and the mass stay positive until it.
// The trajectory is every step, thinned as Principia thins vessel histories
// to the points a cubic Hermite spline through them needs to stay within
// `downsampling_tolerance` (m; 0 keeps every step). Its `*count` points are
// returned in `*times` and `*states`, and the apsides relative to each of
// `apsis_bodies` (computed on every step; at most `apsis_capacity` in all,
// earliest first per body) in `*apsides`: arrays the library allocates, to be
// released with principia_ksa__Free (null when empty). `*steps` is the number
// of steps taken, `*end` why the prediction stopped.
PRINCIPIA_KSA_API principia_ksa_Status principia_ksa__Predict(
    principia_ksa_Ephemeris const* ephemeris,
    principia_ksa_QP const* initial,
    double t0,
    double t_end,
    int64_t max_steps,
    double length_tolerance,
    double speed_tolerance,
    principia_ksa_XYZ thrust_direction,
    double thrust,
    double initial_mass,
    double mass_flow,
    double downsampling_tolerance,
    int const* apsis_bodies,
    int apsis_body_count,
    int apsis_capacity,
    double** times,
    principia_ksa_QP** states,
    int* count,
    principia_ksa_Apsis** apsides,
    int* apsis_count,
    int64_t* steps,
    principia_ksa_PredictionEnd* end);

// Releases an array returned by principia_ksa__Predict.
PRINCIPIA_KSA_API void principia_ksa__Free(void* array);

// Where the calling thread's last principia_ksa__Predict spent its time (ms),
// for finding slow predictions.
typedef struct principia_ksa_PredictTimings {
  double total;
  double prolong;          // Integrating the planets ahead of the vessel...
  double give_way;         // ...of which waiting for the game's own calls.
  int64_t prolong_pieces;  // Pieces of six hours integrated.
  double flow;             // Integrating the vessel.
  double apsides;          // Finding apsides.
  double thinning;         // Downsampling the points kept.
  int64_t chunks;          // Chunks of the flow (a month or 50,000 steps).
} principia_ksa_PredictTimings;

PRINCIPIA_KSA_API void principia_ksa__LastPredictTimings(
    principia_ksa_PredictTimings* timings);

#ifdef __cplusplus
}  // extern "C"
#endif
