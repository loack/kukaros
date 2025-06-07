class PathPlanner:
    def __init__(self, move_group):
        self.move_group = move_group

    def plan_path(self, target_pose):
        self.move_group.set_pose_target(target_pose)
        plan = self.move_group.plan()
        return plan

    def execute_plan(self, plan):
        success = self.move_group.execute(plan, wait=True)
        return success