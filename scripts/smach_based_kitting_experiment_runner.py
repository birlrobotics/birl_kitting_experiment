#!/usr/bin/env python
from smach_based_introspection_framework.online_part import (
    smach_runner
)
from birl_kitting_experiment.smach_FSM import (
    assembly_user_defined_sm
)

if __name__ == '__main__':
    sm = assembly_user_defined_sm()
    smach_runner.run(sm)
