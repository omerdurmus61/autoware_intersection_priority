# Configuration

`autoware_intersection_priority.param.yaml` is loaded by the Python launch file.
Parameters are read at node startup; restart the node after changing them.
The same defaults apply when using `ros2 run` without a parameter file.

- IDs, VTL type/rate and all four topic names are configurable. One intersection
  is mapped to one virtual traffic light.
- `tracked_object_timeout_sec` (1.0): remove an unseen UUID's states and queue
  entries with `TRACKED_OBJECT_TIMEOUT`, not `LEFT_INTERSECTION`.
- `tracked_objects_freshness_sec` (0.5): maximum age of the latest valid object
  message received in the map frame. An empty message is a valid observation.
- `conflict_clear_duration_sec` (0.5): all grant conditions must remain valid
  continuously for this duration: ego first in queue, ego in PRIORITY, no tracked
  CONFLICT object, and fresh object data. Any failure restarts this interval.
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

## Field-test diagnostics

Startup `CONFIGURATION` logs show the effective IDs, publication rate, timeout,
freshness, clear duration, and UNKNOWN setting. `TOPICS` shows resolved topic
names, including ROS remappings.

While ego is in PRIORITY or CONFLICT without approval, `WAITING` reports the
primary blocking reason and all relevant gate values. INFO logs appear only
when the reason, ego/queue state, data status, or conflicting UUID list changes.
When multiple conditions fail, consult the full snapshot and `WAITING_BLOCKER`
lines, not only the primary reason.

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
  /localization/kinematic_state /awapi/tmp/virtual_traffic_light_states /clock
```

Use your configured topic names if they differ from these defaults. Preserve
this node's console logs and the effective parameter YAML alongside the bag.
