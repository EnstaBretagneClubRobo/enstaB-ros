#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time
from std_msgs.msg import Empty 
from gps_handler.srv import *
from proxy_eura_smach import ErrorMessage

global autonomous_level #0 teleOP 1semi autonomous  2 full autonomous

############# INITIALISATION ###################################
class Init(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['endInitGPS','endInitGPSArdu','endInitTeleOp'])

    def execute(self, userdata):
        rospy.loginfo("init...")
        #get Data for what to do, level of autonomous
        
        #verification branchement des sensors
        #ccny hokuyo test mavros 
        rospy.wait_for_service('start_node_srv')
        #wait parameter


        rospy.wait_for_service('/camera_rgb_frame_tf/get_loggers')
        rospy.wait_for_service('/pwm_serial_send')
        rospy.wait_for_service('/hokuyo_node/self_test')
        rospy.wait_for_service('IsNear')
        is_near_srv = rospy.ServiceProxy('IsNear', IsNear)
        start= rospy.get_time()
        while  rospy.get_time()-start < 5*60  and not gpsdata:
             rospy.sleep(1.0/20.0)

        if not gpsdata:
           autonomous_level = 0
           return 'endInitTeleOp'
     
        start= rospy.get_time()
        try:
          (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          exit(0)
        Lat = 10,600
        Long = 42,4555
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = trans1[0]
        msgSRV.yLat1 = trans1[1]
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 500
        res = is_near_srv(msgSRV).response
        while rospy.get_time()-start < 5*60 and not res:#wait for GPS for 5 min
             try:
                (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             rospy.sleep(1.0/20.0)
        if not res:
           return 'endInitTeleOp'
        return 'endInit'
#################################################################

################# Go Building ###################################
class GoBuildingGPS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gps_done'])

    def execute(self, userdata):
        rospy.loginfo("GoBuildingGPS")
        
        #launch GPS algo plus evitement obstacle
        #wait for GPS waipoint(s)
        #make sure Cytron is good
        #launch algo colour movie recorder
        #needed PKG gps algo move color detection mavros pwm_serial_send
        Lat = 10,600#last coord
        Long = 42,4555
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = trans1[0]
        msgSRV.yLat1 = trans1[1]
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res:#wait for GPS for 5 min
             if self.preempt_requested():
                ROS_INFO("Go building GPS is being preempted")
                self.service_preempt()
                return 'preempted'

             try:
                (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             #if end algo change algo to direct one
        return 'endGoBuilding'

def goBGPS_cb(outcome_map):
    if outcome_map['GPS_norm'] == 'endGoBuilding':
        return True
    elif outcome_map['GPS_STOP'] == 'invalid':
        return True
    else:
        return False


def goBGPS_out_cb(outcome_map):
    if outcome_map['GPS_STOP'] == 'invalid':
        return 'gps_stop'
    elif outcome_map['GPS_norm'] == 'endGoBuilding':
        return 'gps_done'
    else:
        return 'gps_stop'

##
class GoBuildingGPSArdu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endGoBuilding'])

    def execute(self, userdata):
        rospy.loginfo("GoBuilfingGPSArdu")
        #wait for GPS waipoint(s)
        #tell ardu GPS waypoint 
        #launch algo colour movie recorder
        Lat = 10,600#last coord
        Long = 42,4555
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = trans1[0]
        msgSRV.yLat1 = trans1[1]
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res:#wait for GPS for 5 min
             if self.preempt_requested():
                ROS_INFO("Go building GPS Ardu is being preempted")
                self.service_preempt()
                return 'preempted'
             try:
                (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             #if end algo change algo to direct one
        #stop autonomous ardu
        return 'endGoBuilding'

def goBGPSArdu_cb(outcome_map):
    if outcome_map['GPSA_norm'] == 'endGoBuilding':
        return True
    elif outcome_map['GPSA_STOP'] == 'invalid':
        return True
    else:
        return False


def goBGPSArdu_out_cb(outcome_map):
    if outcome_map['GPSA_STOP'] == 'invalid':
        return 'gps_stop'
    elif outcome_map['GPSA_norm'] == 'endGoBuilding':
        return 'gps_done'
    else:
        return 'gps_stop'
##
class GoBuildingTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endGoBuilding'])

    def execute(self, userdata):
        rospy.loginfo("GoBuilfingGPSArdu")
        #wait for GPS waipoint(s)
        #make sure Cytron is good 
        #needed PKG pwm_serial_send ccny
        # signal to end  
        while not teleOpGOend:#send by callback
            if self.preempt_requested():
                ROS_INFO("Go building TeleOp is being preempted")
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0/20.0)
        return 'endGoBuilding'

def goBTeleOp_cb(outcome_map):
    if outcome_map['TeleOp'] == 'endGoBuilding':
        return True
    elif outcome_map['TeleOp_STOP'] == 'invalid':
        return True
    else:
        return False


def goBTeleOp_out_cb(outcome_map):
    if outcome_map['TeleOp_STOP'] == 'invalid':
        return 'TeleOp_stop'
    elif outcome_map['TeleOp'] == 'endGoBuilding':
        return 'TeleOp_done'
    else:
        return 'TeleOp_stop'
##
class EmergencyStopGo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergPrepEntry','endEmergTeleOp','endEmergGPSArdu','endEmergGPS'])

    def execute(self, userdata):
        rospy.loginfo("EmergencyStopGo")
        #send 1500 1500
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
       #check if orentation send
        #checkorientation
        #calc size map needed
        #make sure Cytron is good
        #launch ccny hector static
        #needed PKG mavros diagnostic
        
        return 'endEntryBuilding'

def entry_cb(outcome_map):
    if outcome_map['InitEntry'] == 'endEntryBuilding':
        return True
    elif outcome_map['Entry_STOP'] == 'invalid':
        return True
    else:
        return False


def entry_out_cb(outcome_map):
    if outcome_map['Entry_STOP'] == 'invalid':
        return 'preEntry_stop'
    elif outcome_map['InitEntry'] == 'endEntryBuilding':
        return 'preEntry_done'
    else:
        return 'preEntry_stop'
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
        #if not teleOp launch algo point gps donne a l'avance ou lit ici
        #needed PKG : ccny hector hokuyo pwm_serial_send car_controller (astar_path) 
        while not cartoEnd:#send by callback
            if self.preempt_requested():
                ROS_INFO("interior Cartographie is being preempted")
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0/20.0)
        return 'endCartographieBuilding'

def intercarto_cb(outcome_map):
    if outcome_map['InterCarto'] == 'endCartographieBuilding':
        return True
    elif outcome_map['Carto_STOP'] == 'invalid':
        return True
    else:
        return False


def intercarto_out_cb(outcome_map):
    if outcome_map['Carto_STOP'] == 'invalid':
        return 'carto_proc'
    elif outcome_map['InterCarto'] == 'endCartographieBuilding':
        return 'carto_done'
    else:
        return 'carto_proc'
##
class ProceduralStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['Proc_done','Proc_stop'])

    def execute(self, userdata):
        rospy.loginfo("ProceduralStop")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG sauvegarde ?
        if self.preempt_requested():
            ROS_INFO("ProceduralStop is being preempted")
            self.service_preempt()
            return 'preempted'
        return 'endProceduralStop'

def proc_cb(outcome_map):
    if outcome_map['ProcStop'] == 'endProceduralStop':
        return True
    elif outcome_map['ProcStop_STOP'] == 'invalid':
        return True
    else:
        return False


def proc_out_cb(outcome_map):
    if outcome_map['ProcStop_STOP'] == 'invalid':
        return 'Proc_stop'
    elif outcome_map['ProcStop'] == 'endProceduralStop':
        return 'Proc_done'
    else:
        return 'Proc_stop'
##
class EmergencyStopInt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmer_Exit','endEmer_Proc','endEmer_Carto'])

    def execute(self, userdata):
        rospy.loginfo("EmergencyStopInt")
        #make sure Cytron is good
        #check algo state       
        #needed PKG sauvegarde ?
        return 'endEmergencyStopInt'



###################################################################


###################### Exit Building ##############################

class ExitBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endExitBuilding'])

    def execute(self, userdata):
        rospy.loginfo("ExitBuilding")
        if not autonomous_level:
           return 'teleop' 
        #start algo deplacement       
        #needed PKG astar_path pwm_serial_send mavros gps ?
        Lat = 10,600#coord of entry
        Long = 42,4555
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = trans1[0]
        msgSRV.yLat1 = trans1[1]
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res or not arrived:#wait for GPS for 5 min
             if self.preempt_requested():
                ROS_INFO("Exit Building is being preempted")
                self.service_preempt()
                return 'preempted'
             try:
                (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
        return 'endExitBuilding'

def exit_cb(outcome_map):
    if outcome_map['ExitBuild'] == 'endExitBuilding':
        return True
    elif outcome_map['ExitB_STOP'] == 'invalid':
        return True
    else:
        return False


def exit_out_cb(outcome_map):
    if outcome_map['ExitB_STOP'] == 'invalid':
        return 'exit_stop'
    elif outcome_map['ExitBuild'] == 'endExitBuilding':
        return 'exit_done'
    else:
        return 'exit_stop'
##
class ExitBuildingTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endExitBuilding'])

    def execute(self, userdata):
        rospy.loginfo("ExitBuilding") 
        while not exitTeleopEnd: 
             if self.preempt_requested():
                ROS_INFO("Exit Building Telop is being preempted")
                self.service_preempt()
                return 'preempted'
             rospy.sleep(1.0/20.0)
        #needed PKG  pwm_serial_send mavros gps ?
        return 'endExitBuilding'

def exitTeleOp_cb(outcome_map):
    if outcome_map['ExitTeleOp'] == 'endExitBuilding':
        return True
    elif outcome_map['ExitTeleOp_STOP'] == 'invalid':
        return True
    else:
        return False


def exitTeleOp_out_cb(outcome_map):
    if outcome_map['ExitTeleOp_STOP'] == 'invalid':
        return 'exit_stop'
    elif outcome_map['ExitTeleOp'] == 'endExitBuilding':
        return 'exit_done'
    else:
        return 'exit_stop'
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
        #launch GPS algo plus evitement obstacle
        #wait for GPS waipoint(s)
        Lat = 10,600#last coord of waypoint
        Long = 42,4555
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = trans1[0]
        msgSRV.yLat1 = trans1[1]
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res:#wait for GPS for 5 min
             if self.preempt_requested():
                ROS_INFO("Return Home GPS is being preempted")
                self.service_preempt()
                return 'preempted'
             try:
                (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             #if end algo change algo to direct one
        return 'endReturnHomeGPS'

def retGPS_cb(outcome_map):
    if outcome_map['RetHomeGPS'] == 'endReturnHomeGPS':
        return True
    elif outcome_map['RetHomeGPS_STOP'] == 'invalid':
        return True
    else:
        return False


def retGPS_out_cb(outcome_map):
    if outcome_map['RetHomeGPS_STOP'] == 'invalid':
        return 'gps_stop'
    elif outcome_map['RetHomeGPS'] == 'endReturnHomeGPS':
        return 'gps_done'
    else:
        return 'gps_stop'
##
class ReturnHomeArdu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeArdu'])

    def execute(self, userdata):
        rospy.loginfo("ReturnHomeArdu")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG mavros
        #wait for GPS waipoint(s)
        #tell ardu GPS waypoint 
        #launch algo colour movie recorder
        Lat = 10,600#last coord
        Long = 42,4555
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = trans1[0]
        msgSRV.yLat1 = trans1[1]
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res:#wait for GPS for 5 min
             if self.preempt_requested():
                ROS_INFO("Return Home GPS Ardu is being preempted")
                self.service_preempt()
                return 'preempted'
             try:
                (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             #if end algo change algo to direct one
        #stop autonomous ardu
        return 'endReturnHomeArdu'

def retGPSArdu_cb(outcome_map):
    if outcome_map['RetHomeGPSArdu'] == 'endReturnHomeArdu':
        return True
    elif outcome_map['RetHomeGPSArdu_STOP'] == 'invalid':
        return True
    else:
        return False


def retGPSArdu_out_cb(outcome_map):
    if outcome_map['RetHomeGPSArdu_STOP'] == 'invalid':
        return 'gps_stop'
    elif outcome_map['RetHomeGPSArdu'] == 'endReturnHomeArdu':
        return 'gps_done'
    else:
        return 'gps_stop'
##
class ReturnHomeTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeTeleOp'])

    def execute(self, userdata):
        rospy.loginfo("ReturnHomeTeleOp")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG mavros pwm_serial_send 
        # signal to end  
        while not teleOpGOend:#send by callback
            if self.preempt_requested():
                ROS_INFO("Exit Building is being preempted")
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0/20.0)
        return 'endReturnHomeTeleOp'


def retTeleOp_cb(outcome_map):
    if outcome_map['RetTeleOp_calc'] == 'endReturnHomeTeleOp':
        return True
    elif outcome_map['RetTeleOp_STOP'] == 'invalid':
        return True
    else:
        return False


def retTeleOp_out_cb(outcome_map):
    if outcome_map['RetTeleOp_STOP'] == 'invalid':
        return 'TeleOp_stop'
    elif outcome_map['RetTeleOp'] == 'endReturnHomeTeleOp':
        return 'TeleOp_done'
    else:
        return 'TeleOp_stop'
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
################### Failed Node Monitoring ########################
###################################################################
def goBGPS_monitoring(ud, msg):
    checkNode = ['mavros','pwm_serial']

def goBGPSA_monitoring(ud,msg):
    print "t"

def goBTeleOp_monitoring(ud, msg):
    print "t"

def prepEntry_monitoring(ud, msg):
    print "t"

def intercarto_monitoring(ud, msg):
    print "t"

def procStop_monitoring(ud, msg):
    print "t"

def exitB_monitoring(ud, msg):
    print "t"

def exitBTeleOp_monitoring(ud, msg):
    print "t"

def retGPS_monitoring(ud, msg):
    print "t"

def retGPSA_monitoring(ud, msg):
    print "t"

def retTeleOp_monitoring(ud, msg):
    print "t"

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
interiorCartoGraphie_concurrence = smach.Concurrence(outcomes=['carto_done','carto_proc'],
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
    smach.Concurrence.add('GPS_STOP_STUCK', smach_ros.MonitorState("/GPS_stop_stuck",Empty,monitor_cb))

with gobuildingGPSArdu_concurrence:
    smach.Concurrence.add('GPSA_norm', GoBuildingGPSArdu())
    smach.Concurrence.add('GPSA_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))

with gobuildingTeleOp_concurrence:
    smach.Concurrence.add('TeleOp', GoBuildingTeleOp())
    smach.Concurrence.add('TeleOp_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))

with initEntry_concurrence:
    smach.Concurrence.add('InitEntry', InitEntryBuilding())
    smach.Concurrence.add('Entry_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))


with interiorCartoGraphie_concurrence:
    smach.Concurrence.add('InterCarto', CartographieBuilding())
    smach.Concurrence.add('Carto_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))

with procedural_stop_concurrence:
    smach.Concurrence.add('ProcStop', ProceduralStop())
    smach.Concurrence.add('ProcStop_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))


with exitBuilding_concurrence:
    smach.Concurrence.add('ExitBuild', ExitBuilding())
    smach.Concurrence.add('ExitB_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))

with exitBuildingTeleOp_concurrence:
    smach.Concurrence.add('ExitTeleOp', ExitBuildingTeleOp())
    smach.Concurrence.add('ExitTeleOp_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))

with returnHomeGPS_concurrence:
    smach.Concurrence.add('RetHomeGPS', ReturnHomeGPS())
    smach.Concurrence.add('RetHomeGPS_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))

with returnHomeGPSArdu_concurrence:
    smach.Concurrence.add('RetHomeGPSArdu', ReturnHomeArdu())
    smach.Concurrence.add('RetHomeGPSArdu_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))

with returnHomeTeleOp_concurrence:
    smach.Concurrence.add('RetTeleOp', ReturnHomeTeleOp())
    smach.Concurrence.add('RetTeleOp_STOP', smach_ros.MonitorState("/stop_command",Empty,monitor_cb))

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
                                                                              'carto_proc':'ProceduralStop'})

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
