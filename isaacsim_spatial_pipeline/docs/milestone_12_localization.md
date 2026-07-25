# Milestone 12: Saved-Map Localization and Relocalization

## Status

`BLOCKED — required map/runtime inputs and installed localization packages are unavailable.`

Direct checks on 2026-07-25 found:

```text
repository saved map.yaml/map.pgm artifacts: none
repository slam_toolbox posegraph artifacts: none
nav2_map_server: Package not found
nav2_amcl: Package not found
nav2_lifecycle_manager: Package not found
slam_toolbox localization_slam_toolbox_node: installed
```

The Milestone 8 document records a real map-quality `WARN`, but its final saved
map artifacts were never documented. `slam_toolbox` localization needs a
serialized posegraph, while AMCL needs the unavailable Nav2 packages plus a
validated occupancy map. Creating either input would fabricate evidence.

No localization launch, map mutation, convergence, alternate-start, or recovery
result is claimed. The smallest unblock is to restore the GPU runtime, produce
and validate immutable map artifacts (and a posegraph if using slam_toolbox),
then choose an already-installed localization backend or explicitly authorize
the missing Nav2 packages.
