#!/usr/bin/env python

import rospy
import smach
import smach_ros
import time


############# INITIALISATION ###################################
class Init(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['endInitGBGPS','endInitGBGPSArdu','endInitGoTeleOp'])

    def execute(self, userdata):
        rospy.loginfo("init...")
        #wait for GPS
        #wait all First algo en ligne
        #verification branchement des sensors
        #ccny hokuyo test mavros
        rospy.wait_for_service('/pwm_serial_send')
        return 'endInit'
#################################################################

################# Go Building ###################################
class GoBuildingGPS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endGoBuilding'])

    def execute(self, userdata):
        rospy.loginfo("GoBuildingGPS")
        #wait for GPS waipoint(s)
        #launch GPS algo plus evitement obstacle
        #make sure Cytron is good
        #needed PKG gps algo move color detection mavros pwm_serial_send
        return 'endGoBuilding'


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


class EmergencyStopGo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergencyStopGo'])

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


class EmergencyStopEntry(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergencyStopEntry'])

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


class ProceduralStop(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endProceduralStop'])

    def execute(self, userdata):
        rospy.loginfo("ProceduralStop")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG sauvegarde ?
        return 'endProceduralStop'

class EmergencyStopInt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergencyStopInt'])

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

class ExitBuildingTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endExitBuilding'])

    def execute(self, userdata):
        rospy.loginfo("ExitBuilding")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG astar_path pwm_serial_send mavros gps ?
        return 'endExitBuilding'

class EmergencyStopExit(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergencyStopInt'])

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

class ReturnHomeTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeTeleOp'])

    def execute(self, userdata):
        rospy.loginfo("ReturnHomeTeleOp")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG mavros pwm_serial_send 
        return 'endReturnHomeTeleOp'


class ReturnHomeArdu(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeArdu'])

    def execute(self, userdata):
        rospy.loginfo("ReturnHomeArdu")
        #need restart algo ?
        #start algo deplacement       
        #needed PKG mavros 
        return 'endReturnHomeArdu'


class ReturnHomeGPS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeGPS'])

    def execute(self, userdata):
        rospy.loginfo("ReturnHomeGPS")
        #need restart algo ?
        #start algo deplacement 
        #needed PKG mavros pwm_serial_send algo
        return 'endReturnHomeGPS'

class EmergencyStopReturn(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEmergencyStopInt'])

    def execute(self, userdata):
        rospy.loginfo("EmergencyStopInt")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG sauvegarde ?
        return 'endEmergencyStopInt'


###################################################################



rospy.init_node('ai_mapping_robot')
rospy.on_shutdown(ShutdownCallback)
sm_cal = smach.StateMachine(outcomes=['endMapping'])

##
gobuildingGPS_concurrence = smach.Concurrence(outcomes=['gps_done','gps_stop'],
                                      default_outcome='gps_done',
                                      child_termination_cb = child_term_cb,
                                      outcome_cb = out_cb)

gobuildingGPSArdu_concurrence = smach.Concurrence(outcomes=['gps_done','gps_stop'],
                                      default_outcome='gps_done',
                                      child_termination_cb = child_term_cb,
                                      outcome_cb = out_cb)

gobuildingTeleOp_concurrence = smach.Concurrence(outcomes=['TeleOp_done','TeleOp_stop'],
                                      default_outcome='TeleOp_done',
                                      child_termination_cb = child_term_cb,
                                      outcome_cb = out_cb)
##
initEntry_concurrence = smach.Concurrence(outcomes=['preEntry_done','preEntry_stop'],
                                      default_outcome='preEntry_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)
##
interiorCartoGraphie_concurrence = smach.Concurrence(outcomes=['carto_done','carto_proc','carto_stop'],
                                      default_outcome='carto_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)

procedural_stop_concurrence = smach.Concurrence(outcomes=['Proc_done','Proc_stop'],
                                      default_outcome='Proc_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)
##
exitBuilding_concurrence = smach.Concurrence(outcomes=['exit_done','exit_teleop','exit_stop'],
                                      default_outcome='exit_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)

exitBuildingTeleOp_concurrence = smach.Concurrence(outcomes=['exit_done','exit_stop'],
                                      default_outcome='exit_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)
##
returnHomeGPS_concurrence = smach.Concurrence(outcomes=['gps_done','gps_stop'],
                                      default_outcome='gps_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)

returnHomeGPSArdu_concurrence = smach.Concurrence(outcomes=['gps_done','gps_stop'],
                                      default_outcome='gps_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)

returnHomeTeleOp_concurrence = smach.Concurrence(outcomes=['TeleOp_done','TeleOp_stop'],
                                      default_outcome='TeleOp_done',
                                      child_termination_cb = lin_child_term_cb,
                                      outcome_cb = lin_out_cb)


with gobuildingGPS_concurrence:
    smach.Concurrence.add('GPS_norm', GoBuildingGPS())
    smach.Concurrence.add('GPS_STOP', smach_ros.MonitorState("/GPS_stop",uint16,monitor_cb))

with gobuildingGPSArdu_concurrence:
    smach.Concurrence.add('Angle_calc', AngleCalib())
    smach.Concurrence.add('Angle_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))


with lin_concurrence:
    smach.Concurrence.add('Lin_calc', LinCalib())
    smach.Concurrence.add('Lin_STOP', smach_ros.MonitorState("/sm_stop",Empty,monitor_cb))

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
    smach.StateMachine.add('PrepEntryBuilding',initEntry_concurrence,transitions={'preEntry_done':'PrepEntryBuilding',
                                                                                  'preEntry_stop':'EmergencyStopPrepEntry'})

    smach.StateMachine.add('EmergencyStopPrepEntry',EmergencyStopEntry(),transitions={'endEmergPrep':'PrepEntryBuilding',
                                                                                      'endEmergCarto':'InteriorCarto'})
    #Interior cartographie
    smach.StateMachine.add('InteriorCarto',interiorCartoGraphie_concurrence ,transitions={'carto_done':'ExitBuilding',
                                                                              'carto_proc':'ProceduralStop',
                                                                              'carto_stop':'EmergencyStopPrepEntry'})

    smach.StateMachine.add('ProceduralStop',procedural_stop_concurrence,transitions={'Proc_done':'InteriorCarto',
                                                                                     'Proc_Stop':'EmergencyStopPrepEntry'})

    smach.StateMachine.add('EmergencyStopPrepEntry',EmergencyStopInt(),transitions={'endEmer_Carto':'InteriorCarto',
                                                                                    'endEmer_Proc': 'ProceduralStop',
                                                                                    'endEmer_Exit':'ExitBuilding'})
    #Exit building
    smach.StateMachine.add('ExitBuilding',exitBuilding_concurrence ,transitions={'exit_done':'returnHome',
                                                                                 'exit_teleop':'ExitTeleOp',
                                                                                 'exit_stop':'EmergencyStopExit'})

    smach.StateMachine.add('ExitTeleOp',exitBuildingTeleOp_concurrence,transitions={'exit_done':'InteriorCarto',
                                                                                    'exit_stop':' EmergencyStopExit'})

    smach.StateMachine.add('EmergencyStopExit', EmergencyStopExit(),transitions={'endEmer':'EndExit',
                                                                                 'endEmer_Proc': 'ProceduralStop',
                                                                                 'endEmer_Exit':'ExitBuiding'})

    #End Exit
    smach.StateMachine.add('EndExit',EndExitBuilding(),transitions={'endEx_CartoGPS':'ReturnGPS',
                                                                        'endEx_CartoGPSArdu':'ReturnGPSArdu',
                                                                        'endEx_CartoTeleOp':'ReturnTeleOp',})

    #Return Home

    smach.StateMachine.add('ReturnGPS',returnHomeGPS_concurrence ,transitions={'gps_done':'endMapping',
                                                                               'gps_stop':'EmergencyStopGo'})
    smach.StateMachine.add('ReturnGPSArdu',returnHomeGPSArdu_concurrence,transitions={'gps_done':'endMapping',
                                                                                      'gps_stop':'EmergencyStopGo'})
    smach.StateMachine.add('ReturnTeleOp',returnHomeTeleOp_concurrence,transitions={'TeleOp_done':'endMapping',
                                                                                    'TeleOp_stop':'EmergencyStopGo'})
    smach.StateMachine.add('EmergencyStopReturn',EmergencyStopReturn(),transitions={'endEmergGPS':'ReturnGPS',
                                                                            'endEmergGPSArdu':'ReturnGPSArdu',
                                                                            'endEmergTeleOp':'ReturnTeleOp',
                                                                            'endEmergEnd':'endMapping'})


sis = smach_ros.IntrospectionServer('AI_EURATHLON', sm_cal, '/GENERAL_BEHAVIOUR')
sis.start()
outcome = sm_cal.execute()
#rospy.spin()
sis.stop()
