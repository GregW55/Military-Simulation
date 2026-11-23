// Retired test suite.
//
// These tests exercised src/utils/TrackCorrelation.h, an older position/
// heuristic correlation helper that is not called by the current radar path.
//
// Current production tracking tests belong in:
//   - TrackingSystemTests.cpp       (local EKF measurement association)
//   - TrackFusionTests.cpp          (global track distance and CI fusion)
//   - SensorFusionSystemTests.cpp   (local-to-global association/lifecycle)
//
// Keep this empty translation unit temporarily if the test executable uses
// an explicit source list that still names TrackCorrelationTests.cpp. Remove
// it from the test target and delete it after updating that build list.
