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
