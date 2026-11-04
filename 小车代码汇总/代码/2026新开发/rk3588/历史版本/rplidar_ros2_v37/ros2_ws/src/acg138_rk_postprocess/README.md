# acg138_rk_postprocess

This package is the RK-side, read-only presentation layer for the v42 FPGA
navigation events. It does not alter or republish the raw event topics.

The node subscribes to:

- `/fpga/state` (`std_msgs/msg/String`): the raw STATE event represented as
  flat JSON with the fixed 160-byte field names from the FPGA contract.
- `/fpga/plan_result` (`std_msgs/msg/String`): the raw PLAN_RESULT event as
  flat JSON with the fixed 64-byte field names.
- `/fpga/planned_path` (`nav_msgs/msg/Path`): the compatibility path produced
  by the existing bridge.
- `/base/diagnostics` (`std_msgs/msg/String`): read-only source diagnostics.

It publishes derived, observational topics:

- `/fpga/gui_state` and `/fpga/phone_state` (`std_msgs/msg/String`)
- `/fpga/gui_plan_result` and `/fpga/phone_plan_result` (`std_msgs/msg/String`)
- `/fpga/display_path` (`nav_msgs/msg/Path`), with bounded interior smoothing
  and exact endpoint preservation
- `/fpga/diagnostics_summary` (`std_msgs/msg/String`), grouped by source,
  reason, and gate

Malformed state or plan input is published as `valid:false` JSON containing the
escaped original payload and an error. This keeps an operator-visible record
without manufacturing a valid FPGA sample. The pure core in
`include/acg138_rk_postprocess/postprocess.hpp` has no ROS dependency and is
covered by the direct host test runner in `tests/run_host_tests.py`.

The package has no publishers or service clients for execution, motor, or
velocity control. The raw `/fpga/state` and `/fpga/plan_result` contracts remain
owned by their producer; this package only consumes their JSON adapter form.
