# Leader coverage preview implementation plan

> **For agentic workers:** use superpowers:executing-plans inline.

**Goal:** run leader's C++ coverage algorithm on ROS 2 with polygon selection.
**Architecture:** independent overlay, generated C++ adaptation, own service,
Python selection node, warehouse simulation with a dedicated RViz view.
**Tech Stack:** Jazzy rclcpp/rclpy, Nav2 costmap, OpenCV, Eigen, Gazebo.
**Spec:** design.md (scope approved in chat).

## Global constraints
Preserve leader source and existing Nav2/RPP configuration. No robot motion.
No invented robot_msg definitions. Do not commit/push unrelated local work.

## Review focus
Invalid/self-crossing/nonfinite polygons; unknown cells; unavailable map;
map boundaries and segment collisions; UI reset and stale async responses.

## Tasks
- [x] 1. Tests first for polygon/selection validation and actual planner
  obstacle avoidance, both patterns, no-map and invalid-request responses.
  Record red, generate source translation, build interface/core/server, green.
- [x] 2. Implement selection state and ROS UI, closed polygon triggers service,
  explicit reset/undo/plan/pattern controls, retained visualization and status.
  Test state transitions and invalid closure; live topic/service probes.
- [x] 3. Add launch/build/run/stop helpers, dedicated RViz and warehouse demo
  polygon. Build overlay, run integration, inspect screenshots and path output,
  review changes, document commands and limitations. Leave scene open for user.
