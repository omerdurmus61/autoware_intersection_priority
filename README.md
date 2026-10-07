# autoware_intersection_priority

A C++17 package for ROS 2 Humble and Autoware that tracks intersection polygons,
maintains arrival queues, and publishes Virtual Traffic Light (VTL) approvals at
unsignalized intersections. One node supports multiple intersections, with
independent object/ego states, queues, approval waiting periods, and latched approvals.

## Inputs and output

| Direction | Default topic | Message type | Purpose |
| --- | --- | --- | --- |
| Input | `/map/vector_map` | `autoware_map_msgs/msg/LaneletMapBin` | Lanelet2 map and intersection polygons |
| Input | `/perception/object_recognition/tracking/objects` | `autoware_perception_msgs/msg/TrackedObjects` | Object positions identified by UUID |
| Input | `/localization/kinematic_state` | `nav_msgs/msg/Odometry` | Ego position and longitudinal speed |
| Output | `/vtl/intersection_states` | `tier4_v2x_msgs/msg/VirtualTrafficLightStateArray` | VTL states for configured intersections |

The map subscription uses reliable, transient local, depth-1 QoS. Object and
odometry subscriptions use reliable, volatile, depth-1 QoS. VTL states are
published at 10 Hz by default, including lights whose approval is false.
Each light uses `type="virtual"` and `is_finalized=false` by default.

Object and odometry messages must have `header.frame_id = map`. Polygon coordinates
are assumed to use the same frame. Although TF2 dependencies are declared, the
current node does not perform TF transformations; inputs in other frames are skipped.

## Map tags

The node searches the Lanelet2 Area and Polygon layers. Approach and conflict
polygons belonging to the same intersection must share an `intersection_id` tag.

Approach polygons:

```text
subtype=intersection_priority
intersection_id=1
```

Central conflict area:

```text
subtype=intersection_conflict
intersection_id=1
```

Multiple priority polygons are supported per intersection; exactly four are not
required. One conflict polygon is expected. If more are found, the node logs a
warning and considers all of them. Polygons are not limited to four vertices,
and Area holes are preserved. Points on the outer boundary are not considered inside.

Polygons are grouped by `intersection_id`. Their Lanelet2 IDs are reported in logs
but are not used to select an intersection. Polygons without `intersection_id`
are stored separately and do not contribute to an intersection queue. A configured
intersection with missing priority or conflict polygons cannot receive a new VTL approval.

## Zone tracking and arrival queues

Objects are evaluated using the `x/y` coordinates of their pose center. Ego uses
`pose.pose.position.x/y` from odometry. Ego speed is logged from
`twist.twist.linear.x`; no speed threshold is applied to approval decisions.

| State | Condition |
| --- | --- |
| `CONFLICT` | The center point is inside any conflict polygon |
| `PRIORITY` | The center point is outside conflict polygons and inside any priority polygon |
| `OUTSIDE` | The center point is outside both zones |

`CONFLICT` takes precedence when polygons overlap. Object states are tracked by
UUID and `intersection_id`; ego states are tracked separately for each intersection.

- An `OUTSIDE -> PRIORITY` transition adds the object or ego to the queue.
  A participant cannot appear more than once in the same active queue.
- Arrival time is the `header.stamp` of the message in which the transition is
  observed. Entries are sorted by this timestamp; equal timestamps retain
  insertion order.
- For tracked objects, `PRIORITY -> OUTSIDE` retains the queue entry and original
  arrival time to tolerate fluctuations near polygon boundaries.
- For tracked objects, `CONFLICT -> OUTSIDE` removes the queue entry. A direct
  `OUTSIDE -> CONFLICT` transition does not add an entry but still makes the
  conflict zone occupied.
- When ego transitions to `OUTSIDE`, it is removed from that intersection's queue
  and its active visit information is cleared.
- An unseen UUID is removed from state records and queues after
  `tracked_object_timeout_sec`. This is logged as `TRACKED_OBJECT_TIMEOUT`
  rather than `LEFT_INTERSECTION`.

Classification uses the label with the highest finite probability. `PEDESTRIAN`
and `BICYCLE` are excluded. `UNKNOWN` objects and objects with no classification
are accepted when `allow_unknown_objects=true`. Bounding box shape and the full
object footprint are not used; polygon membership is checked only at the center
point. Stationary UNKNOWN objects can also keep the conflict zone occupied.

## VTL approval logic

All approvals start as `false`. Before an intersection can receive approval,
all of the following conditions must remain valid continuously for
`conflict_clear_duration_sec`:

1. The intersection has both priority and conflict polygons.
2. Ego is first in that intersection's arrival queue.
3. Ego is currently in `PRIORITY`.
4. No tracked object is in `CONFLICT` at that intersection.
5. The valid tracked-object message stream is recent enough to satisfy
   `tracked_objects_freshness_sec`.

A valid, empty `TrackedObjects` message counts as a recent observation. A stopped
message stream is not an observation of an empty intersection. An unseen object's
timeout and the message stream's freshness check are separate mechanisms.

Once granted, approval is latched. It remains true despite queue or perception
changes while ego crosses, and resets on the corresponding `EGO_LEFT_INTERSECTION`
event. Exiting one intersection does not affect another intersection's approval.
A map update clears queues and pending approvals while preserving existing
latched approvals and the ego state needed to detect exit.

Timeout, freshness, and approval waiting checks use monotonic receipt time, so
they continue even when simulation time pauses. Arrival queues use message
timestamps; object and odometry timestamps must therefore share a time base.

## Configuration

The Python launch file automatically loads the
[parameter file](config/autoware_intersection_priority.param.yaml).
Parameters are read at startup; restart the node after changing them.

| Parameter | Shipped YAML value |
| --- | --- |
| `intersection_ids` | `["1", "2"]` |
| `intersections.1.virtual_traffic_light_id` | `"2012980"` |
| `intersections.2.virtual_traffic_light_id` | `"2013058"` |
| `virtual_traffic_light_type` | `"virtual"` |
| `virtual_traffic_light_publish_rate_hz` | `10.0` |
| `tracked_object_timeout_sec` | `4.0` |
| `conflict_clear_duration_sec` | `0.5` |
| `tracked_objects_freshness_sec` | `0.5` |
| `allow_unknown_objects` | `true` |

Topic names are configurable in the same YAML file. To add an intersection,
extend `intersection_ids`, add its `virtual_traffic_light_id` mapping under
`intersections`, and tag its OSM polygons with the matching `intersection_id`.
The VTL ID is the virtual light's instrument ID in the Autoware map, independent
of polygon IDs. No C++ changes or ID configuration in the mux are required.

Without YAML, the node uses legacy single-intersection parameters:
`intersection_id="1"` and `virtual_traffic_light_id="2012980"`. The C++ fallback
object timeout is also `1.0` second. Load the parameter file to reproduce launch
behavior. See the [configuration guide](config/README.md) for parameter details
and examples of adding intersections.

## Build and run

ROS 2 Humble and the required Autoware dependencies must be available.
Main dependencies include `rclcpp`, `autoware_map_msgs`,
`autoware_perception_msgs`, `nav_msgs`, `tier4_v2x_msgs`, Lanelet2, and
`autoware_lanelet2_extension`. See [package.xml](package.xml) for the complete list.

For the current workspace layout:

```bash
source /opt/ros/humble/setup.bash
source ~/autoware/install/setup.bash
cd ~/autoware_intersection_priority_ws
colcon build --packages-select autoware_intersection_priority autoware_virtual_traffic_light_mux
source install/setup.bash
ros2 launch autoware_intersection_priority autoware_intersection_priority.launch.py
```

Adjust the `source` path if Autoware is installed elsewhere. The launch file starts
only this node; map, perception, and localization publishers must be started
separately through Autoware.

## Mux integration

```text
autoware_intersection_priority -> /vtl/intersection_states --+
                                                          +-> mux -> /awapi/tmp/virtual_traffic_light_states
driver approval               -> /vtl/driver_states -------+
```

Start the mux in another terminal:

```bash
source ~/autoware_intersection_priority_ws/install/setup.bash
ros2 launch autoware_virtual_traffic_light_mux autoware_virtual_traffic_light_mux.launch.py
```

The driver approval package should publish to `/vtl/driver_states`. Only the mux
should publish to the final topic. The mux concatenates the latest input state
arrays without ID mapping or right-of-way decisions. Sources that time out are
omitted from its output; it does not synthesize `approval=false` records.
See the [mux README](../autoware_virtual_traffic_light_mux/README.md) for details.

## Logs and troubleshooting

`CONFIGURATION` and `TOPICS` report startup settings. Polygon logs include the
intersection ID, Lanelet2 ID, and point count. Object and ego states are logged
only on transitions. Queue changes report the order and ego position.
Approval changes are reported as `RIGHT_OF_WAY_GRANTED` / `RIGHT_OF_WAY_REVOKED`.

If ego is in a zone but does not receive approval, inspect `WAITING` and
`WAITING_BLOCKER`:

| Reason | What to check |
| --- | --- |
| `MAP_POLYGONS_MISSING` | Map tags and the intersection's priority/conflict polygons |
| `EGO_NOT_IN_QUEUE` / `EGO_NOT_FIRST` | Ego's queue entry and participants ahead of it |
| `EGO_NOT_IN_PRIORITY` | Ego may have entered the conflict zone before receiving approval |
| `CONFLICT_OCCUPIED` | The blocking UUID and its `last_seen_age` |
| `OBJECT_DATA_STALE` | The message stream, frame, and valid position data |
| `CLEAR_DURATION_PENDING` | How long all approval conditions have remained valid |

To see updated DEBUG details while the situation remains unchanged, stop the
current instance and run with the same parameter file:

```bash
ros2 run autoware_intersection_priority autoware_intersection_priority_node \
  --ros-args \
  --params-file "$(ros2 pkg prefix autoware_intersection_priority)/share/autoware_intersection_priority/config/autoware_intersection_priority.param.yaml" \
  --log-level autoware_intersection_priority:=debug
```

See [config/README.md](config/README.md) for the bag recording command and detailed
log field descriptions.

## Tests and current scope

```bash
colcon build --packages-select autoware_intersection_priority --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select autoware_intersection_priority --event-handlers console_direct+
colcon test-result --verbose
```

The [ROS integration tests](test/test_multi_intersection.py) use synthetic maps
to check independent intersection queues and approvals, timeout cleanup, map
updates, missing geometry, and invalid configuration.

Current decisions use observed arrival order and conflict occupancy. The node
does not implement the right-hand rule, simultaneous-arrival right-of-way rules,
left-turn priority, predicted vehicle trajectories, or polygon intersection with
the full vehicle footprint.
