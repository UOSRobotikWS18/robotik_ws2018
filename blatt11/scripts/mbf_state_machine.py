#!/usr/bin/env python
import rospy
import smach
import smach_ros
import random

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import ExePathResult

from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import GetPathResult

from mbf_msgs.msg import RecoveryAction
from mbf_msgs.msg import RecoveryResult
from geometry_msgs.msg import Twist

from geometry_msgs.msg import PoseStamped

from ceres_navigation.srv import *

def pub_zero_cmd():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    tmsg = Twist()

    pub.publish(tmsg)
CONTROLLERS = ['dwa', 'teb', 'eband']
# 0: dwa, 1: teb, 2: eband
global CONTROLLER
CONTROLLER = 'eband'

def set_controller(controller):
    global CONTROLLER
    print("CB: current controller: " + CONTROLLER)
    if(controller.controller in CONTROLLERS):
        print('service call: setting controller to: ' + controller.controller)
        CONTROLLER = controller.controller
        return SetControllerResponse(True)
    else:
        print("controler: " + controller.controller +" not found")
        return SetControllerResponse(False)


def child_term(data):
    print (data)
    if data['NAVIGATION'] is not None:
        print('navigation finished with outcome "' + data['NAVIGATION'] + '": terminating concurrent states')
        return True
    elif data['REPLAN'] == 'invalid':
        print("replan found: terminating concurrent states")
        return True
    else:
        return False

@smach.cb_interface(input_keys=['target_pose'])
def get_path_goal_cb(userdata, goal):
    goal.use_start_pose = False
    goal.tolerance = 0.2
    goal.target_pose = userdata.target_pose
    goal.planner = 'planner'



@smach.cb_interface(
    output_keys=['outcome', 'message', 'path'],
    outcomes=['succeeded', 'failure', 'preempted'])
def get_path_result_cb(userdata, status, result):
    userdata.message = result.message
    userdata.outcome = result.outcome
    userdata.path = result.path

    print("get_path_result:")
    print(result.outcome)

    if result.outcome == GetPathResult.SUCCESS:
        return 'succeeded'
    elif result.outcome == 101:
        return 'preempted'
    else:
        return 'failure'


@smach.cb_interface(input_keys=['path'])
def ex_path_goal_cb(userdata, goal):
    goal.path = userdata.path
    current_controller = CONTROLLER
    print(current_controller)
    goal.controller = current_controller


@smach.cb_interface(
    output_keys=['outcome', 'message', 'final_pose', 'dist_to_goal'],
    outcomes=['succeeded', 'failure',  'preempted'])
def ex_path_result_cb(userdata, status, result):
    userdata.message = result.message
    userdata.outcome = result.outcome
    userdata.dist_to_goal = result.dist_to_goal
    userdata.final_pose = result.final_pose


    print("ex_path_result:")
    print(result.outcome)

    if result.outcome == ExePathResult.SUCCESS:
        return 'succeeded'
    elif result.outcome == ExePathResult.CANCELED:
        return 'preempted'
    else:
        return 'failure'


@smach.cb_interface(input_keys=['recovery_flag'], output_keys=['recovery_flag'])
def recovery_goal_cb(userdata, goal):
    # TODO implement a more clever way to call the right behavior
    if not userdata.recovery_flag:
        goal.behavior = 'clear_costmap'
        userdata.recovery_flag = True
    else:
        goal.behavior = 'straf_recovery'
        userdata.recovery_flag = False


@smach.cb_interface(
    output_keys=['outcome', 'message'],
    outcomes=['succeeded', 'failure', 'preempted'])
def recovery_result_cb(userdata, status, result):
    print("recovery result outcome code: " + str(result.outcome))
    if result.outcome == RecoveryResult.SUCCESS:
        return 'succeeded'
    elif result.outcome == 101 or result.outcome == 151:
        return 'preempted'
    else:
        return 'failure'

@smach.cb_interface(output_keys=['target_pose'],
    outcomes=['valid','invalid','preempted'])
def monitor_pose_cb(userdata, msg):
    userdata.target_pose = msg
    return False


def main():
    rospy.init_node('mbf_state_machine')
    rospy.Service('~set_controller', SetController, set_controller)
    mbf_sm = smach.StateMachine(outcomes=['preempted', 'succeeded', 'aborted'])
    mbf_sm.userdata.recovery_flag = False

    with mbf_sm:


        smach.StateMachine.add('WAIT_FOR_GOAL',
                               smach_ros.MonitorState("/move_base_simple/goal",
                                                      PoseStamped,
                                                      monitor_pose_cb,
                                                      output_keys=['target_pose']),
                               transitions={'valid': 'CON',
                                            'preempted': 'WAIT_FOR_GOAL',
                                            'invalid': 'CON'})

        mbf_nav_sm = smach.StateMachine(outcomes=['preempted', 'nav_finished_succeeded_very_good_finish_sub_mbf_sm', 'aborted'],
                                        input_keys=['target_pose'])
        mbf_nav_sm.userdata.recovery_flag = False

        with mbf_nav_sm:
            ## Navigation Stack
            # GET_PATH
            # EXE_PATH
            # RECOVERY

            smach.StateMachine.add('GET_PATH',
                               smach_ros.SimpleActionState('move_base_flex/get_path',
                                                           GetPathAction,
                                                           goal_cb=get_path_goal_cb,
                                                           result_cb=get_path_result_cb),
                               transitions={'succeeded': 'EXE_PATH',
                                            'failure': 'aborted',
                                            'preempted': 'preempted'})

            smach.StateMachine.add('EXE_PATH',
                                smach_ros.SimpleActionState('move_base_flex/exe_path',
                                                            ExePathAction,
                                                            goal_cb=ex_path_goal_cb,
                                                            result_cb=ex_path_result_cb),
                                transitions={'succeeded': 'nav_finished_succeeded_very_good_finish_sub_mbf_sm',
                                             'failure': 'RECOVERY',
                                             'preempted': 'preempted'})

            smach.StateMachine.add('RECOVERY',
                                smach_ros.SimpleActionState('move_base_flex/recovery',
                                                            RecoveryAction,
                                                            goal_cb=recovery_goal_cb,
                                                            result_cb=recovery_result_cb),
                                transitions={'succeeded': 'GET_PATH',
                                             'failure': 'aborted',
                                             'preempted': 'preempted'})

        mbf_con_sm = smach.Concurrence(outcomes=['succeeded', 'preempted','aborted', 'replan'],
                                default_outcome='aborted',
                                input_keys=['target_pose'],
                                output_keys=['target_pose'],
                                outcome_map={'succeeded' :
                                                {'NAVIGATION' : 'nav_finished_succeeded_very_good_finish_sub_mbf_sm'},
                                             'aborted' :
                                                {'NAVIGATION' : 'aborted'},
                                             'replan' :
                                                {'REPLAN' : 'invalid'},
                                             'preempted' :
                                                {'NAVIGATION' : 'preempted', 'REPLAN' :  'preempted'},
                                },
                                child_termination_cb=child_term
                                )
        mbf_con_sm.userdata.recovery_flag = False

        with mbf_con_sm:

            # Navigation Stack
            smach.Concurrence.add('NAVIGATION', mbf_nav_sm)
            # Replan State with MonitorState
            smach.Concurrence.add('REPLAN',
                                  smach_ros.MonitorState("/move_base_simple/goal",
                                                         PoseStamped,
                                                         monitor_pose_cb,
                                                         output_keys=['target_pose']))
            # smach.Concurrence.add('GOAL_CHECKER')

        smach.StateMachine.add('CON',
                               mbf_con_sm,
                               transitions={
                                   'succeeded' : 'WAIT_FOR_GOAL',
                                   'aborted' : 'WAIT_FOR_GOAL',
                                   'replan' : 'CON',
                                   'preempted': 'WAIT_FOR_GOAL'})

    sis = smach_ros.IntrospectionServer('mbf_state_machine_server', mbf_sm, '/SM_ROOT')
    sis.start()
    outcome = mbf_sm.execute()
    rospy.spin()
    sis.stop()


if __name__ == "__main__":
    while not rospy.is_shutdown():
        main()
