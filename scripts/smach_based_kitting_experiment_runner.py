#!/usr/bin/env python
from smach_based_introspection_framework.online_part import (
    smach_runner
)
from birl_kitting_experiment.smach_FSM import (
    assembly_user_defined_sm
)

from birl_kitting_experiment.hardcoded_data import (
    reverting_statistics
)

from birl_kitting_experiment.Q_table import (
    Q_table_statistics
)
if __name__ == '__main__':
    sm = assembly_user_defined_sm()
    smach_runner.run(sm, Q_table_statistics)
