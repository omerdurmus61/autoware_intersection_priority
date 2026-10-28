# Intersection Priority Queue panel

RViz2 panel bundled in `autoware_intersection_priority`, showing the actual decision queues published by
`autoware_intersection_priority`. No queue or right-of-way decisions are made here.

## Build and use

In a terminal with the ROS 2 Humble and Autoware environments sourced:

```bash
cd ~/autoware_intersection_priority_ws
colcon build --packages-select autoware_intersection_priority
source install/setup.bash
ros2 launch autoware_intersection_priority autoware_intersection_priority.launch.py
```

Start RViz2 from another terminal with the same workspace sourced (restart an
already running RViz so it discovers the new plugin). In RViz select
**Panels → Add New Panel → autoware_intersection_priority/QueuePanel**.
Drag the panel's title bar to the left docking area. Save the RViz configuration
with **File → Save Config** to preserve the panel, docking position and topic.

Only the intersection whose queue contains AV in PRIORITY or CONFLICT is shown.
Its heading changes automatically as AV moves to another intersection; outside
all queues, the panel shows “No active intersection”. If zones overlap, the current
selection is retained while AV remains in it. Four fixed rectangles show the
first four queue entries, with permanent rank numbers 1–4 on the left. AV is
yellow, other vehicles are light blue, and empty slots use the same light blue and show “—”.
AV, vehicle IDs and empty-slot marks all use the same dark text color. Other
vehicles use an unambiguous UUID prefix; full UUID and zone state are available
in the tooltip. When the queue advances, labels and AV color move to the correct
slots. If AV is beyond the first four entries, “AV position: N” appears below.
The panel shows queue order; a first-place AV is not itself permission to proceed.

Existing algorithm semantics are preserved: tracked PRIORITY → OUTSIDE retains
the queue entry (OUTSIDE is shown in its tooltip). CONFLICT → OUTSIDE or tracking timeout
removes it. Hiding a retained entry would understate the actual vehicles ahead.
Direct conflict entrants can block approval without appearing in the arrival queue.

The topic defaults to `/intersection_priority/queues`; it must match the
node's startup `queue_state_topic` parameter. For a custom topic, edit `QueueTopic`
in the saved RViz configuration. The topic is not displayed in the panel. The snapshot uses
`diagnostic_msgs/msg/DiagnosticArray`, reliable/transient-local depth 1, at the
existing VTL publication rate. Each status represents one configured intersection:
`name` is intersection ID, `hardware_id` is VTL ID, `message` is APPROVED or WAITING,
and ordered `values` contain AV/full UUID keys and zone-state values. WARN means
object data is stale/unavailable. This dedicated topic is not `/diagnostics`.

After two seconds without snapshots the panel grays out the slots and labels it
as last known data, rather than showing a frozen queue as live. For publication
rates below 0.5 Hz this indicator may appear between snapshots.
