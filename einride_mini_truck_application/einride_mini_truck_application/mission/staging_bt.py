"""Behavior tree for NavigateToPose to a staging pose. No ROS imports.

Nav2's default tree (navigate_to_pose_w_replanning_and_recovery.xml) retries 6
times, cycling clear costmaps / Spin 90 deg / Wait 5 s / BackUp 0.3 m. Next to
an obstacle that loop spun the robot away from the tag for minutes
(2026-10-03, "Collision Ahead - Exiting Spin"). This is the same tree with:
  - no Spin: the lidar sees 360 degrees, so turning shows nothing new, and
    skid-steer turns are where odometry slips;
  - no Wait, no clearing of the global costmap in the recoveries: it holds
    what the lidar has seen of obstacles from earlier viewpoints, which
    staging relies on;
  - 2 retries, so Nav2 gives up quickly and the mission picks another
    staging pose instead.
Kept to nodes in every Jazzy release (no WouldA...RecoveryHelp conditions).
bt_navigator loads it from a file named in the goal (`write_tree`).
"""

import os

XML = """\
<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="2" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{selected_controller}" \
default_controller="FollowPath" topic_name="controller_selector"/>
        <PlannerSelector selected_planner="{selected_planner}" \
default_planner="GridBased" topic_name="planner_selector"/>
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" \
error_code_id="{compute_path_error_code}"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" \
service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="{selected_controller}" \
error_code_id="{follow_path_error_code}"/>
          <ClearEntireCostmap name="ClearLocalCostmap-Context" \
service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>
      </PipelineSequence>
      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>
        <RoundRobin name="RecoveryActions">
          <ClearEntireCostmap name="ClearLocalCostmap-Subtree" \
service_name="local_costmap/clear_entirely_local_costmap"/>
          <BackUp backup_dist="0.15" backup_speed="0.15" error_code_id="{backup_code_id}"/>
        </RoundRobin>
      </ReactiveFallback>
    </RecoveryNode>
  </BehaviorTree>
</root>
"""


def write_tree(directory: str) -> str:
    """Write the tree to `directory` and return its path."""
    path = os.path.join(directory, 'navigate_to_staging.xml')
    with open(path, 'w') as f:
        f.write(XML)
    return path
