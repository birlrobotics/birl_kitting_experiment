#!/usr/bin/env python
from optparse import OptionParser
import rospy
from baxter_interface import DigitalIO
from baxter_core_msgs.msg import EndpointState
from smach_based_introspection_framework.configurables import dmp_cmd_fields
from birl_skill_management.util import get_eval_postfix
import numpy
import os,ipdb
list_of_postfix = get_eval_postfix(dmp_cmd_fields, 'pose')

dir_of_this_script = os.path.dirname(os.path.realpath(__file__))
demonstrations_dir = os.path.join(dir_of_this_script, '..', 'data', 'demonstrations')

timeseries = []
sample = [None]*len(list_of_postfix)
def cb(msg, right_lower_cuff):
    global sample
    if right_lower_cuff.state:
        for idx, field in enumerate(list_of_postfix):
            sample[idx] = eval('msg.pose'+field)
        timeseries.append(sample[:])
        print sample

if __name__ == '__main__':
    parser = OptionParser(usage='usage: %prog --name demonstration_name')
    parser.add_option(
        "--name",
        action="store",
        type='string',
        dest='name',
        help='Name of the demonstration', 
    )
    (options, args) = parser.parse_args()

    if options.name is None:
        parser.error("Specify demonstration name please.")

    rospy.init_node("record_demonstration_py")

    right_lower_cuff = DigitalIO("right_lower_cuff")
    rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, cb, right_lower_cuff)
    
    rospy.spin()

    arr = numpy.array(timeseries)    

    if not os.path.isdir(demonstrations_dir):
        os.makedirs(demonstrations_dir)
    numpy.save(os.path.join(demonstrations_dir, options.name), arr)
