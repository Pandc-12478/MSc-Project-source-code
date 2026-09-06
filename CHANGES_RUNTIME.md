# Runtime packaging changes — 2026-09-06

Base: a7075ca0add633e870ffd14cae0b9278e7fc8aa7.

- Added English README with dependencies, missing source files, startup order, six-condition commands, logger outputs and troubleshooting.
- Added shared repository-relative paths and configurable ROS/workspace/data locations.
- Corrected the bridge's missing v2 HTML path to the bundled v4 file; added an optional HTML override.
- Made both controller and initial-pose script read the same configurable waypoint path. No control algorithm or numeric parameter was changed.
- Added a read-only runtime checker. Missing calibrated waypoints and the external localization package remain explicit blockers.
- Changed base startup to a foreground support-process group, including the previously omitted map-zone trigger and web bridge. Stop the trial before shutting down the group.
- Consolidated the duplicate logger launcher into a compatibility wrapper.
- Made the logger launcher reject an unreadable cmd_vel publisher count instead of displaying it as zero; added the zone topic to the non-logger launcher's prerequisites.

Not supplied: guessed waypoints, a reconstructed localization launch, or unverified final-experiment parameter changes.

Before replacing files in the experimental workspace, preserve the original workspace. This is a post-study packaging revision, not a claim that new launch behavior was used to collect the dissertation data.
