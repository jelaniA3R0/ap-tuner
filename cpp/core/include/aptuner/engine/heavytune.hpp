// Model-based tuning for heavy and slow multirotors / QuadPlanes (the method of the tandem QuadPlane hand tune,
// docs/how-it-works.md "Model-based tuning"):
//
//   flights (logs or live telemetry)  ->  hovers  ->  diagnostics (torque imbalance, mixer limit, CG offset, cascade
//   oscillation, saturation, slew limiter, vibration, hover throttle, altitude hold, compass)  ->  identification as a
//   SET of plants (yaw: windowed integral regression with a trim per hover; roll / pitch: marginal-mode identification
//   at a persistent oscillation)  ->  validation (closed-loop replay of every hover with the flown gains)  ->  robust
//   design over the set (margins, noise, time simulations, stress tests, limits from physics, one gyro filter for all
//   axes)  ->  steps (.param + UNDO + predictions + test card)  ->  next-flight check against a saved plan.
//
// Proposals only: nothing here writes to a vehicle.
#pragma once

#include <string>
#include <vector>

#include "aptuner/core/json.hpp"
#include "aptuner/engine/flightrec.hpp"

namespace aptuner::engine::heavy {

struct Options {
  /** a plan saved by an earlier analysis of the same vehicle (null: none): enables the next-flight check */
  Json plan;
  /** front-rear motor spacing (m), for "move the battery X cm"; 0 = not known */
  double motorSpacingM = 0;
  /** extra plain-language facts from the user (e.g. "nothing physical changed") */
  std::vector<std::string> userNotes;
};

/**
 * Analyse one or more flights of the same vehicle. The result (JSON) holds:
 *  vehicle, flights, hovers, heavyClass {heavy, why}, issues [{level, axis, title, text, fix, cantTune}],
 *  axes {roll, pitch, yaw: {ident, validation, flown, design, replay}}, steps [{n, title, rows, predicted, testCard,
 *  paramText, undoText}], check (next-flight check against opts.plan), plan (to save for the next flight), summary [text].
 */
Json analyzeFlights(const std::vector<FlightRecord>& flights, const Options& opts = {});

/**
 * "Check my values": the settings flown with some values replaced (rows [{name, value}], e.g. typed by the user) judged on
 * the models of an analysis result: every value checked as for a parameter file (exact float32 / integer form, ranges),
 * then per axis the predictions and the same acceptance tests the design used. Returns
 * {ok, rows [{name, value, text, ok, errors, warnings}], axes {roll, pitch, yaw: {touched, pass, why [text], predicted}}}.
 */
Json evaluateSettings(const Json& result, const Json& rows);

/** the exact value a parameter file carries for a value (float32 shortest form, whole numbers for integer parameters) */
double exactValue(const std::string& name, double value);

}  // namespace aptuner::engine::heavy
