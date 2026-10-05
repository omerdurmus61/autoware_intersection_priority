# Configuration

`autoware_intersection_priority.param.yaml` is loaded by the Python launch file.
Parameters are read at node startup; restart the node after changing them.
The supplied YAML manages both configured intersections in one node. Use the same
parameter file with `ros2 run` if you need the same behavior as the launch file.
Without a parameter file, legacy single-intersection defaults still apply.

- `intersection_ids` selects the intersections whose lights are published.
  `intersections.<id>.virtual_traffic_light_id` maps each intersection to its VTL.
  The supplied mappings are `1 -> 2012980` and `2 -> 2013058`.
- VTL type/rate and all four topic names are configurable and shared.
- `tracked_object_timeout_sec` (YAML: 4.0; C++ fallback: 1.0): remove an unseen UUID's states and queue
  entries with `TRACKED_OBJECT_TIMEOUT`, not `LEFT_INTERSECTION`.
- `tracked_objects_freshness_sec` (0.5): maximum age of the latest valid object
  message received in the map frame. An empty message is a valid observation.
- `conflict_clear_duration_sec` (0.5): all grant conditions must remain valid
  continuously for this duration: ego first in queue, ego in PRIORITY, no tracked
  CONFLICT object in that intersection, and fresh object data. Both priority and
  conflict geometry must exist. Any failure restarts that intersection's interval.
- `allow_unknown_objects` (true): accept UNKNOWN or missing classifications.
  The highest-probability finite classification is used; PEDESTRIAN and BICYCLE
  are excluded. Reclassification removes previous state/queue membership with
  `TRACKED_OBJECT_FILTERED`.

Timeout, freshness and grant-delay checks use monotonic reception time, so they
continue when simulated ROS time pauses. Queue timestamps still use message
stamps. Stale-object cleanup runs at the configured VTL publication rate.
Invalid object frames/positions cannot establish fresh data for a new grant.

Tracked PRIORITY -> OUTSIDE preserves queue membership and arrival time.
CONFLICT -> OUTSIDE or timeout removes membership. Ego exit behavior is unchanged.
Once granted, approval stays latched despite later data/queue changes until the
observed ego exit. A map update clears pending grants and tracking data while
preserving exit detection for an already-latched grant.

Publish rate, timeout and freshness must be finite and positive; clear duration
must be finite and nonnegative. Zero clear duration disables the waiting interval.

## VTL output routing

Run `autoware_virtual_traffic_light_mux` alongside this node. The intersection
output is `/vtl/intersection_states`; the mux combines it with `/vtl/driver_states`
and publishes the final array on `/awapi/tmp/virtual_traffic_light_states`.
Only the mux should publish to that final topic. The launch file here starts
only the intersection node; start the mux with its own launch file.
Adding an intersection requires no ID configuration in the mux; it forwards
all states received on this node's output topic.

## Adding intersections

Extend both the ID list and the corresponding mapping in the existing YAML:

```yaml
/**:
  ros__parameters:
    intersection_ids: ["1", "2", "3"]
    intersections:
      "1":
        virtual_traffic_light_id: "2012980"
      "2":
        virtual_traffic_light_id: "2013058"
      "3":
        virtual_traffic_light_id: "REPLACE_WITH_NEW_VTL_ID"
```

Keep the other parameters in the file. Tag the new OSM priority and conflict
polygons with the same `intersection_id` (for example `3`). The VTL ID is the
instrument ID used by Autoware's VTL configuration, independent of polygon IDs.
Rebuild/install the package after editing its source configuration and restart
the node. No C++ changes are needed to add another mapping.

Every timer tick publishes one `VirtualTrafficLightStateArray` on
`/vtl/intersection_states`, containing all
configured lights, including lights with `approval=false`. Each intersection
has its own queue, clear interval, approval latch, and waiting diagnostics.
Occupancy or ego exit in one intersection does not change another's approval.
Object-stream freshness is shared because all intersections use the same input.
New approval is disabled for a configured intersection with missing priority or
conflict polygons, with a warning when the map is loaded. Existing latched
approvals still survive map updates until ego's observed exit.

Intersection IDs and VTL IDs must be nonempty and unique; incomplete or duplicate
mappings cause startup to fail. Polygons with unconfigured intersection IDs still
participate in existing tracking/queue logs but have no VTL published.

For compatibility, omitting `intersection_ids` activates
the old `intersection_id` / `virtual_traffic_light_id` parameters, defaulting to
`1` / `2012980`. A nonempty `intersection_ids` list takes precedence over those
legacy parameters. Parameters are startup configuration, not live remapping.

## Field-test diagnostics

Startup `CONFIGURATION` logs show the effective IDs, publication rate, timeout,
freshness, clear duration, and UNKNOWN setting. `TOPICS` shows resolved topic
names, including ROS remappings.

While ego is in PRIORITY or CONFLICT without approval, `WAITING` reports the
primary blocking reason and all relevant gate values. INFO logs appear only
when the reason, ego/queue state, data status, or conflicting UUID list changes.
When multiple conditions fail, consult the full snapshot and `WAITING_BLOCKER`
lines, not only the primary reason.

- `MAP_POLYGONS_MISSING`: the configured intersection lacks priority or conflict
  geometry; check the OSM tags and the loaded map.
- `CONFLICT_OCCUPIED`: each `WAITING_BLOCKER` lists a conflicting UUID, its
  monotonic `last_seen_age`, and the configured timeout.
- `OBJECT_DATA_STALE`: `object_data_status` distinguishes `NOT_RECEIVED`, `STALE`,
  `INVALID_FRAME`, and `INVALID_POSITION`. `last_message_age` includes invalid
  incoming messages; `data_age` measures the current usable observation.
- `EGO_NOT_FIRST` / `EGO_NOT_IN_QUEUE`: inspect queue position (0 means absent),
  size, head identifier/state, and the head object's last-seen age.
  A retained OUTSIDE queue head can explain waiting after boundary noise.
- `EGO_NOT_IN_PRIORITY`: ego is already in CONFLICT and cannot receive a new grant.
- `CLEAR_DURATION_PENDING`: compare `clear_elapsed` with `clear_required`.
  `unavailable` means the valid-condition interval has not started.

Enable DEBUG for updated snapshots at most once per second while the situation
stays unchanged (stop the launched instance before starting another publisher):

```bash
ros2 run autoware_intersection_priority autoware_intersection_priority_node \
  --ros-args --params-file "$(ros2 pkg prefix autoware_intersection_priority)/share/autoware_intersection_priority/config/autoware_intersection_priority.param.yaml" \
  --log-level autoware_intersection_priority:=debug
```

A last-seen age repeatedly near zero means the UUID is still arriving, even if
its stored zone never changes. An increasing age followed by timeout indicates
a missing track. Timeout/filter logs also include the last-seen age; -1 denotes
unavailable history. These diagnostics do not change approval or queue logic.

For the next field test, record the input/output topics in a separate terminal:

```bash
ros2 bag record /map/vector_map \
  /perception/object_recognition/tracking/objects \
  /localization/kinematic_state /vtl/intersection_states /vtl/driver_states \
  /awapi/tmp/virtual_traffic_light_states /clock
```

Use your configured topic names if they differ from these defaults. Preserve
this node's console logs and the effective parameter YAML alongside the bag.

## Regression tests

The ROS integration tests use unique test topics and synthetic maps. They check
independent queues and conflict blockers, per-intersection clear intervals and
latches, map reloads, timeout cleanup, multiple states per VTL message, additional
mappings, missing geometry, legacy parameters, and invalid configuration.

```bash
colcon build --packages-select autoware_intersection_priority --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select autoware_intersection_priority --event-handlers console_direct+
colcon test-result --verbose
```
