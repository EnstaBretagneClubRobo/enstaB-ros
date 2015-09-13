#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import Empty 

############# INITIALISATION ###################################
class Init(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['endInitGPS','endInitGPSArdu','endInitTeleOp'])

    def execute(self, userdata):
        rospy.loginfo("init...")
        #get Data for what to do, level of autonomous
        #wait for GPS
        #wait all First algo en ligne
        #verification branchement des sensors
        #ccny hokuyo test mavros 
        rospy.wait_for_service('start_node_srv')



        rospy.wait_for_service('/camera_rgb_frame_tf/get_loggers')
        rospy.wait_for_service('/pwm_serial_send')
        rospy.wait_for_service('/hokuyo_node/self_test')
        return 'endInit'
#################################################################

################# Go Building ###################################
class GoBuildingGPS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gps_done'])

    def execute(self, userdata):
        rospy.loginfo("GoBuildingGPS")
        #wait for GPS waipoint(s)
        #launch GPS algo plus evitement obstacle
        #make sure Cytron is good
        #needed PKG gps algo move color detection mavros pwm_serial_send
        return 'endGoBuilding'

def goBGPS_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def goBGPS_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'

##
class GoBuildingGPSArdu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endGoBuilding'])

    def execute(self, userdata):
        rospy.loginfo("GoBuilfingGPSArdu")
        #wait for GPS waipoint(s)
        #tell ardu GPS waypoint 
        #make sure Cytron is good
        #needed PKG color detection mavros ccny
        return 'endGoBuilding'

def goBGPSArdu_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def goBGPSArdu_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class GoBuildingTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endGoBuilding'])

    def execute(self, userdata):
        rospy.loginfo("GoBuilfingGPSArdu")
        #wait for GPS waipoint(s)
        #tell ardu GPS waypoint 
        #make sure Cytron is good 
        #needed PKG mavros pwm_serial_send ccny
        return 'endGoBuilding'

def goBTeleOp_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def goBTeleOp_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class EmergencyStopGo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergPrepEntry','endEmergTeleOp','endEmergGPSArdu','endEmergGPS'])

    def execute(self, userdata):
        rospy.loginfo("EmergencyStopGo")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG pwm_serial_send 
        return 'endEmergencyStopGo'

##################################################################

################ Preparing Entry in building #####################

class InitEntryBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEntryBuilding'])

    def execute(self, userdata):
        rospy.loginfo("InitEntryBuilding")
        #wait for GPS waipoint(s)
        #tell ardu GPS waypoint 
        #make sure Cytron is good
        #needed PKG mavros diagnostic
        return 'endEntryBuilding'

def entry_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def entry_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class EmergencyStopEntry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergPrep','endEmergCarto'])

    def execute(self, userdata):
        rospy.loginfo("EmergencyStopEntry")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG
        return 'endEmergencyStopEntry'
###################################################################



################ InteriorCartoGraphie #####################

class CartographieBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endCartographieBuilding'])

    def execute(self, userdata):
        rospy.loginfo("CartographieBuilding")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG : ccny hector hokuyo pwm_serial_send car_controller (astar_path) 
        return 'endCartographieBuilding'

def intercarto_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def intercarto_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class ProceduralStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Proc_done','Proc_stop'])

    def execute(self, userdata):
        rospy.loginfo("ProceduralStop")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG sauvegarde ?
        return 'endProceduralStop'

def proc_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def proc_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class EmergencyStopInt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmer_Exit','endEmer_Proc','endEmer_Carto'])

    def execute(self, userdata):
        rospy.loginfo("EmergencyStopInt")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG sauvegarde ?
        return 'endEmergencyStopInt'



###################################################################


###################### Exit Building ##############################

class ExitBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endExitBuilding'])

    def execute(self, userdata):
        rospy.loginfo("ExitBuilding")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG astar_path pwm_serial_send mavros gps ?
        return 'endExitBuilding'

def exit_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def exit_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class ExitBuildingTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endExitBuilding'])

    def execute(self, userdata):
        rospy.loginfo("ExitBuilding")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG astar_path pwm_serial_send mavros gps ?
        return 'endExitBuilding'

def exitTeleOp_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def exitTeleOp_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class EmergencyStopExit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmer_Exit','endEmer_End'])

    def execute(self, userdata):
        rospy.loginfo("EmergencyStopInt")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG sauvegarde ?
        return 'endEmergencyStopInt'



###################################################################


###################### End Exit Building ##########################

class EndExitBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEx_CartoGPS','endEx_CartoGPSArdu','endEx_CartoTeleOp'])

    def execute(self, userdata):
        rospy.loginfo("EndExitBuilding")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG sauvegarde ?
        return 'endEx_CartoGPS'

###################################################################


###################### Return Home ################################

class ReturnHomeGPS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeGPS'])

    def execute(self, userdata):
        rospy.loginfo("ReturnHomeGPS")
        #need restart algo ?
        #start algo deplacement 
        #needed PKG mavros pwm_serial_send algo
        return 'endReturnHomeGPS'

def retGPS_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def retGPS_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class ReturnHomeArdu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeArdu'])

    def execute(self, userdata):
        rospy.loginfo("ReturnHomeArdu")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG mavros 
        return 'endReturnHomeArdu'

def retGPSArdu_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def retGPSArdu_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class ReturnHomeTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeTeleOp'])

    def execute(self, userdata):
        rospy.loginfo("ReturnHomeTeleOp")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG mavros pwm_serial_send 
        return 'endReturnHomeTeleOp'


def retTeleOp_cb(outcome_map):
    if outcome_map['Angle_calc'] == 'angle_succeeded':
        return True
    elif outcome_map['Angle_STOP'] == 'invalid':
        return True
    else:
        return False


def retTeleOp_out_cb(outcome_map):
    if outcome_map['Angle_STOP'] == 'invalid':
        return 'angle_stop'
    elif outcome_map['Angle_calc'] == 'angle_succeeded':
        return 'angle_done'
    else:
        return 'angle_stop'
##
class EmergencyStopReturn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergGPS','endEmergGPSArdu','endEmergTeleOp','endEmergEnd'])

    def execute(self, userdata):
        rospy.loginfo("EmergencyStopInt")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG sauvegarde ?
        return 'endEmergencyStopInt'


###################################################################
def ShutdownCallback():
    print "shutdown"

def monitor_cb(ud, msg):
    return False

##################################################################
#################### Start of node ###############################
##################################################################

rospy.init_node('ai_mapping_robot')
rospy.on_shutdown(ShutdownCallback)
sm_cal = smach.StateMachine(outcomes=['endMapping'])

##
gobuildingGPS_concurrence = smach.Concurrence(outcomes=['gps_done','gps_stop'],
                                      default_outcome='gps_done',
                                      child_termination_cb = goBGPS_cb,
                                      outcome_cb = goBGPS_out_cb)

gobuildingGPSArdu_concurrence = smach.Concurrence(outcomes=['gps_done','gps_stop'],
                                      default_outcome='gps_done',
                                      child_termination_cb = goBGPSArdu_cb,
                                      outcome_cb = goBGPSArdu_out_cb)

gobuildingTeleOp_concurrence = smach.Concurrence(outcomes=['TeleOp_done','TeleOp_stop'],
                                      default_outcome='TeleOp_done',
                                      child_termination_cb = goBTeleOp_cb,
                                      outcome_cb = goBTeleOp_out_cb)
##
initEntry_concurrence = smach.Concurrence(outcomes=['preEntry_done','preEntry_stop'],
                                      default_outcome='preEntry_done',
                                      child_termination_cb = entry_cb,
                                      outcome_cb = entry_out_cb)
##
interiorCartoGraphie_concurrence = smach.Concurrence(outcomes=['carto_done','carto_proc','carto_stop'],
                                      default_outcome='carto_done',
                                      child_termination_cb = intercarto_cb,
                                      outcome_cb = intercarto_out_cb)

procedural_stop_concurrence = smach.Concurrence(outcomes=['Proc_done','Proc_stop'],
                                      default_outcome='Proc_done',
                                      child_termination_cb = proc_cb,
                                      outcome_cb = proc_out_cb)
##
exitBuilding_concurrence = smach.Concurrence(outcomes=['exit_done','exit_teleop','exit_stop'],
                                      default_outcome='exit_done',
                                      child_termination_cb = exit_cb,
                                      outcome_cb = exit_out_cb)

exitBuildingTeleOp_concurrence = smach.Concurrence(outcomes=['exit_done','exit_stop'],
                                      default_outcome='exit_done',
                                      child_termination_cb = exitTeleOp_cb,
                                      outcome_cb = exitTeleOp_out_cb)
##
returnHomeGPS_concurrence = smach.Concurrence(outcomes=['gps_done','gps_stop'],
                                      default_outcome='gps_done',
                                      child_termination_cb = retGPS_cb,
                                      outcome_cb = retGPS_out_cb)

returnHomeGPSArdu_concurrence = smach.Concurrence(outcomes=['gps_done','gps_stop'],
                                      default_outcome='gps_done',
                                      child_termination_cb = retGPSArdu_cb,
                                      outcome_cb = retGPSArdu_out_cb)

returnHomeTeleOp_concurrence = smach.Concurrence(outcomes=['TeleOp_done','TeleOp_stop'],
                                      default_outcome='TeleOp_done',
                                      child_termination_cb = retTeleOp_cb,
                                      outcome_cb = retTeleOp_out_cb)


with gobuildingGPS_concurrence:
    smach.Concurrence.add('GPS_norm', GoBuildingGPS())
    smach.Concurrence.add('GPS_STOP', smach_ros.MonitorState("/GPS_stop",Empty,monitor_cb))

with gobuildingGPSArdu_concurrence:
    smach.Concurrence.add('Angle_calc', GoBuildingGPSArdu())
    smach.Concurrence.add('Angle_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))


with gobuildingTeleOp_concurrence:
    smach.Concurrence.add('Lin_calc', GoBuildingTeleOp())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

with initEntry_concurrence:
    smach.Concurrence.add('Lin_calc', InitEntryBuilding())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))


with interiorCartoGraphie_concurrence:
    smach.Concurrence.add('Lin_calc', CartographieBuilding())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

with procedural_stop_concurrence:
    smach.Concurrence.add('Lin_calc', ProceduralStop())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

with exitBuilding_concurrence:
    smach.Concurrence.add('Lin_calc', ExitBuilding())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

with exitBuildingTeleOp_concurrence:
    smach.Concurrence.add('Lin_calc', ExitBuildingTeleOp())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

with returnHomeGPS_concurrence:
    smach.Concurrence.add('Lin_calc', ReturnHomeGPS())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

with returnHomeGPSArdu_concurrence:
    smach.Concurrence.add('Lin_calc', ReturnHomeArdu())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

with returnHomeTeleOp_concurrence:
    smach.Concurrence.add('Lin_calc', ReturnHomeTeleOp())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

#

with sm_cal:
    smach.StateMachine.add('Init',Init(),transitions={'endInitGPS':'GoBuildingGPS',
                                                      'endInitGPSArdu':'GoBuildingGPSArdu',
                                                      'endInitTeleOp':'GoBuildingTeleOp'})
    #Go building
    smach.StateMachine.add('GoBuildingGPS',gobuildingGPS_concurrence,transitions={'gps_done':'PrepEntryBuilding',
                                                                                  'gps_stop':'EmergencyStopGo'})

    smach.StateMachine.add('GoBuildingGPSArdu',gobuildingGPSArdu_concurrence,transitions={'gps_done':'PrepEntryBuilding',
                                                                                          'gps_stop':'EmergencyStopGo'})

    smach.StateMachine.add('GoBuildingTeleOp',gobuildingTeleOp_concurrence,transitions={'TeleOp_done':'PrepEntryBuilding',
                                                                                        'TeleOp_stop':'EmergencyStopGo'})

    smach.StateMachine.add('EmergencyStopGo',EmergencyStopGo(),transitions={'endEmergGPS':'GoBuildingGPS',
                                                                            'endEmergGPSArdu':'GoBuildingGPSArdu',
                                                                            'endEmergTeleOp':'GoBuildingTeleOp',
                                                                            'endEmergPrepEntry':'PrepEntryBuilding'})
    #Prepare entry
    smach.StateMachine.add('PrepEntryBuilding',initEntry_concurrence,transitions={'preEntry_done':'InteriorCarto',
                                                                                  'preEntry_stop':'EmergencyStopPrepEntry'})

    smach.StateMachine.add('EmergencyStopPrepEntry',EmergencyStopEntry(),transitions={'endEmergPrep':'PrepEntryBuilding',
                                                                                      'endEmergCarto':'InteriorCarto'})
    #Interior cartographie
    smach.StateMachine.add('InteriorCarto',interiorCartoGraphie_concurrence ,transitions={'carto_done':'ExitBuilding',
                                                                              'carto_proc':'ProceduralStop',
                                                                              'carto_stop':'EmergencyStopCarto'})

    smach.StateMachine.add('ProceduralStop',procedural_stop_concurrence,transitions={'Proc_done':'InteriorCarto',
                                                                                     'Proc_stop':'EmergencyStopCarto'})

    smach.StateMachine.add('EmergencyStopCarto',EmergencyStopInt(),transitions={'endEmer_Carto':'InteriorCarto',
                                                                                'endEmer_Proc': 'ProceduralStop',
                                                                                'endEmer_Exit':'ExitBuilding'})
    #Exit building
    smach.StateMachine.add('ExitBuilding',exitBuilding_concurrence ,transitions={'exit_done':'EndExit',
                                                                                 'exit_teleop':'ExitTeleOp',
                                                                                 'exit_stop':'EmergencyStopExit'})

    smach.StateMachine.add('ExitTeleOp',exitBuildingTeleOp_concurrence,transitions={'exit_done':'EndExit',
                                                                                    'exit_stop':'EmergencyStopExit'})

    smach.StateMachine.add('EmergencyStopExit', EmergencyStopExit(),transitions={'endEmer_End':'EndExit',
                                                                                 'endEmer_Exit':'ExitBuilding'})

    #End Exit
    smach.StateMachine.add('EndExit',EndExitBuilding(),transitions={'endEx_CartoGPS':'ReturnGPS',
                                                                    'endEx_CartoGPSArdu':'ReturnGPSArdu',
                                                                    'endEx_CartoTeleOp':'ReturnTeleOp',})

    #Return Home

    smach.StateMachine.add('ReturnGPS',returnHomeGPS_concurrence ,transitions={'gps_done':'endMapping',
                                                                               'gps_stop':'EmergencyStopReturn'})
    smach.StateMachine.add('ReturnGPSArdu',returnHomeGPSArdu_concurrence,transitions={'gps_done':'endMapping',
                                                                                      'gps_stop':'EmergencyStopReturn'})
    smach.StateMachine.add('ReturnTeleOp',returnHomeTeleOp_concurrence,transitions={'TeleOp_done':'endMapping',
                                                                                    'TeleOp_stop':'EmergencyStopReturn'})
    smach.StateMachine.add('EmergencyStopReturn',EmergencyStopReturn(),transitions={'endEmergGPS':'ReturnGPS',
                                                                                    'endEmergGPSArdu':'ReturnGPSArdu',
                                                                                    'endEmergTeleOp':'ReturnTeleOp',
                                                                                    'endEmergEnd':'endMapping'})


sis = smach_ros.IntrospectionServer('AI_EURATHLON', sm_cal, '/GENERAL_BEHAVIOUR')
sis.start()
outcome = sm_cal.execute()
#rospy.spin()
sis.stop()
