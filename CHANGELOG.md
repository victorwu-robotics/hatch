# Changelog

All notable changes to Hatch will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [1.0.0] — 2026-06-30

### Added

- Initial public release
- Core services: TransformRegistry, StateChannel, KinematicModel
- Robot drivers: SimulatedRobot, RealRobot (UR RTDE)
- VTK visualization: KinematicDisplay, VisualizerEngine
- UI panels: JointControl, CartesianControl, RobotConnection
- Documentation: 10 documents covering user, developer, and philosophy
- MIT License

### Known Limitations

- No automated tests
- No configuration file support
- No sensor calibration pipeline
- No 7‑DOF or parallel robot support

---

## [Unreleased]

### Planned

- TCP switching (multiple tool endpoints)
- Dynamic object detection and collision monitoring
- Configuration file (YAML/JSON)
- Point cloud processing pipeline
- Calibration tools (hand‑eye, TCP)
- Automated test suite