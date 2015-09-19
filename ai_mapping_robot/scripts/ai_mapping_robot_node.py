#!/usr/bin/env python

import rospy,os
import smach
import smach_ros
import time
from std_msgs.msg import Empty ,Int8,String,Bool
import std_srvs.srv as sSrv
from gps_handler.srv import *
from proxy_eura_smach.msg import ErrorMessage
import waiter_subscriber as WS
from ai_mapping_robot.msg import InitData
from ai_mapping_robot.srv import ChangeInitData
import math as m
import LatLongUTMconversion as LLtoUTM
import tf
from start_node.msg import StartKillMsg
from save_node.srv import *
from autonmous_move_handling.srv import *
from autonmous_move_handling.msg import  AstarPoint
from car_controller.srv import * 

global initData #0 teleOP 1semi autonomous  2 full autonomous
global waitGPSData

global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,listener,startKillPub
############# INITIALISATION ###################################
class Init(smach.State):  
    def __init__(self):
        smach.State.__init__(self, outcomes=['endInitGPS','endInitGPSArdu','endInitTeleOp'])
        

    def execute(self, userdata):
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,listener,startKillPub
        
        statePub.publish(String("Initialisation"))

        rospy.loginfo("init...")
        #TODO verification branchement des sensors
        #ccny hokuyo test mavros 
        os.system("rosrun start_node start_node_node.py &")
        rospy.wait_for_service('start_node_srv')
        startKillPub = rospy.Publisher("/start_kill_node",StartKillMsg)
        # 1 rosrun :
        # 0 hokuyu 1 gps_follow 2 save_node 3 pwm_send
        # 0 roslaunch 
        #0 openni 1 rgbd 2 hector_mapping 3 mavros 4 support.launch
        skm = StartKillMsg()
        skm.action  = 1
        skm.type = 0
        skm.nId = 4
        startKillPub.publish(skm)#support.launch: gps_handler start_node diagnostic drift_detection mode stuck rc_receive state_integrateur  proxy_eura_smach
        #rospy.wait_for_service('/IsNear') 
        is_near_srv = rospy.ServiceProxy('IsNear',IsNear)
        rospy.sleep(3)
        stateInterPub.publish(Int8(0))#now we can publish this message not listened before
        skm.action  = 1
        skm.type = 1
        skm.nId = 3
        startKillPub.publish(skm)#pwm serial send 
        #rospy.wait_for_service('/pwm_serial_send')
        skm.action  = 1
        skm.type = 0
        skm.nId = 0
        startKillPub.publish(skm)#kinect
        #rospy.wait_for_service('/camera_rgb_frame_tf/get_loggers') then movie_save
        skm.action  = 1
        skm.type = 1
        skm.nId = 0
        startKillPub.publish(skm) #hokuyo
        rospy.wait_for_service('/hokuyo_node/self_test')
        skm.action  = 1
        skm.type = 0
        skm.nId = 3
        startKillPub.publish(skm) #Mavros #Remote and/or pwm should be online ! otherwise ardu going into failsafe
        rospy.sleep(8)
        #wait parameter
        #get Data for what to do, level of autonomous
        initData = WS.waitForInitData(2*60)
        if initData=='Error':
           exit(0)

        
        start= rospy.get_time()
        try:
          (trans1,rot1) = self.listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          exit(0)
        Lat = 42.954306
        Long = 10.599778
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = trans1[0]
        msgSRV.yLat1 = trans1[1]
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 500
        res = is_near_srv(msgSRV).response
        rospy.loginfo("Wait gps")
        while rospy.get_time()-start < 5*60 and not res:#wait for GPS for 5 min
             try:
                (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             rospy.sleep(1.0/20.0)

        if not res:
           return 'endInitTeleOp' 

        os.system("rosrun tf_dyn_static_broadcaster tf_dyn_static_broadcaster_node local_origin fcu local_origin start_mission &")
        if not initData.autonomous_level:
              return 'endInitTeleOp'

        
        return 'endInitGPS' #ardu not an option as we can't use it properly with computer for now
#################################################################
global waypointsGPS
waypointsGPS = 0
################# Go Building ###################################
class GoBuildingGPS(smach.State): 
    def __init__(self):
        smach.State.__init__(self, outcomes=['endGoBuilding'])#gps_done','gps_stop'])

    def execute(self, userdata):
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,listener,startKillPub,result,info_state_for_emer,waypointsGPS
        info_state_for_emer = 0;
        statePub.publish(String("GoBuildingGPS"))
        stateInterPub.publish(Int8(1))
        servStartDiag()
        rospy.loginfo("GoBuildingGPS")
        
        #launch GPS algo plus evitement obstacle
        
        #launch algo colour movie recorder
        #needed PKG gps algo move color detection mavros pwm_serial_send 
        
        Lat = 42.954306
        Long = 10.599778
                   
        #Start gps follow car
        skm = StartKillMsg()
        skm.action  = 1
        skm.type = 1
        skm.nId = 1
        startKillPub.publish(skm)
        rospy.sleep(2)

        r = rospy.Publisher('start_gps_follow',Int8)
        
        #wait for GPS waipoint(s)
        result = WS.waitForGPSData(self,2*60)
        if result=='preempted':
           return 'preempted'
        elif result=='Error':
             err  = ErrorMessage()
             err.type1=3
             selfErrorPub.publish(err)
             rospy.sleep(1)
             if self.preempt_requested():
                rospy.loginfo("Go building GPS is being preempted")
                self.service_preempt()
                #Kill gps follow car
                skm = StartKillMsg()
                skm.action  = 0
                skm.type = 1
                skm.nId = 1
                startKillPub.publish(skm)
                return 'preempted'
        else:
           Lat =result[0][0]#last coord of list of waypoints
           Long = result[0][1]
           waypointsGPS = result[1]
        r.publish(Int8(1))
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = 11
        msgSRV.yLat1 = 45
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res:#check arrive on place
             if self.preempt_requested():
                r.publish(Int8(0))
                r.unregister()
                rospy.loginfo("Go building GPS is being preempted")
                self.service_preempt()
                #Kill gps follow car
                skm = StartKillMsg()
                skm.action  = 0
                skm.type = 1
                skm.nId = 1
                startKillPub.publish(skm)
                return 'preempted'

             try:
                (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             #if end algo change algo to direct one
        
        r.publish(Int8(0))
        r.unregister()
        servStartDiag()
                   
        #Kill gps follow car
        skm = StartKillMsg()
        skm.action  = 0
        skm.type = 1
        skm.nId = 1
        startKillPub.publish(skm)
        WS.sendCommand(1500,1500)
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,listener,info_state_for_emer
        info_state_for_emer = 1;
        rospy.loginfo("GoBuilfingGPSArdu")
        statePub.publish(String("GoBuildingGPSArdu"))
        stateInterPub.publish(Int8(2))
        servStartDiag()
        #wait for GPS waipoint(s)
        #tell ardu GPS waypoint 
        #launch algo colour movie recorder
        Long = 10,600#last coord
        Lat = 42,4555
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = 11
        msgSRV.yLat1 = 45
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res:#while we're not 1m close to gps location
             if self.preempt_requested():
                rospy.loginfo("Go building GPS Ardu is being preempted")
                self.service_preempt()
                return 'preempted'
             try:
                (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             #if end algo change algo to direct one
        #stop autonomous ardu
        WS.sendCommand(1500,1500)
        servStartDiag()
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,info_state_for_emer
        info_state_for_emer = 2;
        statePub.publish(String("GoBuildingTeleOp"))
        stateInterPub.publish(Int8(3))
        servStartDiag()
        rospy.loginfo("GoBuildingTeleOp")
        #needed PKG pwm_serial_send ccny
        # signal to end 
        global teleOpGOend
        s = rospy.Subscriber('/restart_msg',Empty,remoteCallback) 
        teleOpGOend = 1
        while not teleOpGOend:#send by callback
            if self.preempt_requested():
                rospy.loginfo("Go building TeleOp is being preempted")
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0/20.0)
        s.unregister()
        servStartDiag()
        return 'endGoBuilding'

def remoteCallback(msg):
    global teleOpGOend
    teleOpGOend = 0

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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,info_state_for_emer
        WS.sendCommand(1500,1500)
        servStartDiag()
        statePub.publish(String("EmergencyStopGo"))
        rospy.loginfo("EmergencyStopGo")

        msg = WS.waitForRemoteGo(60*60)
        if msg.data:
           return 'endEmergPrepEntry'
        else:
          if state==0:
            return 'endEmergGPS'
          elif state==1:
            return 'endEmergGPSArdu'
          else:
            return 'endEmergTeleOp'
           
        return 'endEmergTeleOp'

##################################################################

################ Preparing Entry in building #####################

class InitEntryBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEntryBuilding'])

    def execute(self, userdata):
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,listener,startKillPub,LatEntry,LongEntry
        statePub.publish(String("InitEntryBuilding"))
        stateInterPub.publish(Int8(4))
        servStartDiag()
        WS.sendCommand(1500,1500)
        rospy.loginfo("InitEntryBuilding")
        
        
 
        #check if orentation send
        lat1=42.954303
        long1 = 10.599793
        (e,xB1,yB1) = LLtoUTM(lat1,long1)#west north end
        lat2 = 42.954285
        long2 = 10.600006
        (e,xB2,yB2) = LLtoUTM(lat2,long2)#east north end
        lat3 = 42.954209 #not the end of the building
        long3 = 10.599991
        (e,xB3,yB3) = LLtoUTM(lat3,long3)
        lat4 = 42.954093  #south of building not a corner
        long4 = 10.599861
        (e,xB4,yB4) = LLtoUTM(lat4,long4)
        ############ Setting for init Map ######################
        
        largeur1 = m.sqrt((xB1-xB2)**2+(yB1-yB2)**2)
        a1 = yB2-yB1
        b1 = xB1-xB2
        c1 = -(b1.yB1+a1.xB2)
        longeur1 = abs(a1*xB4+b1*yB4+c1)/(m.sqrt(a1**2+b1**2))
        a1 = yB2-yB1
        b1 = xB1-xB2
        c1 = -(b1.yB1+a1.xB2)
        a2 = yB2-yB3
        b2 = xB3-xB2
        c2 = -(b2.yB3+a2.xB3)
        d1 = abs(a1*trans1[0]+b1*trans[1]+c1)/(m.sqrt(a1**2+b1**2))
        d2 = abs(a2*trans1[0]+b2*trans[1]+c2)/(m.sqrt(a2**2+b2**2))
        angle1 = atan(a1)+m.pi/2.0+m.pi;
        angle2 = angle1+m.pi
        angle3 = atan(a1)
        ########################
        heading = [angle1,angle2,angle3]
        if initData.autonomous_level:
           WS.findHeading(listener,heading[initData.heading])
        else:  #if teleOp wait for signal from remote to indicate that we are in good position
           WS.waitForRemote(60)
        #calc size map needed
        Lat = 42.954306
        Long = 10.599778#coordonate of building
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = 11
        msgSRV.yLat1 = 45
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 20
        res = 0
        while not res and rospy.get_time()-start<1*60:#wait for GPS for 1 min
             if self.preempt_requested():
                rospy.loginfo("Init Entry  is being preempted")
                self.service_preempt()
                return 'preempted'
             try:
                (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
        LatEntry = trans1[0]
        LongEntry = trans1[1]
        if initData.heading == 0:
          widthMap = largeur1+6
          heigthMap = d1+longueur1+1+2
          dist = max(widthMap,heigthMap)
          poseY = 1/dist
          poseX = (dist-(3+d2))/dist
        elif initData.heading == 1: 
          widthMap = largeur1+d2+1+2
          heigthMap = longueur1+6
          dist = max(widthMap,heigthMap)
          poseY = (3+d1)/dist
          poseX = 1/dist
        elif initData.heading == 2:
          widthMap = d2+1+2
          heigthMap = longueur1+6
          dist = max(widthMap,heigthMap)
          poseY = (dist-d1-3)/heightMap
          poseX = 1/dist
        else:
          dist = largeur1*2.1
          poseX=0.5
          poseY=0.5
        w = dist/0.05
        os.system("ruby /home/nuc1/ruby/launchFilecreate.rb %d %d %d"%(w,poseX,poseY))
        
        skm = StartKillMsg()
        skm.action  = 1
        skm.type = 0
        skm.nId = 1
        startKillPub.publish(skm)#start ccny_rgbd
        rospy.sleep(5)
        os.system("rosrun tf static_transforme_publisher %f %f %f 0 0 0 /map /gps_origin 100 &")%(-trans1[0],-trans1[1],-trans1[2])#start tf (local_origin fcu) as map to gps_origin
        os.system("rosrun tf_dyn_static_broadcaster tf_dyn_static_broadcaster_node &") #launch ccny hector static

        skm.action  = 1
        skm.type = 0
        skm.nId = 2
        startKillPub.publish(skm)#start hector Mapping
        skm.action  = 1
        skm.type = 1
        skm.nId = 2
        startKillPub.publish(skm)#start save_node
        rospy.sleep(5)
        servStartDiag()
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData
        WS.sendCommand(1500,1500)
        servStartDiag()
        statePub.publish(String("EmergencyStopEntry"))
        rospy.loginfo("EmergencyStopEntry")

        WS.waitForRemoteGo(60*60)
        return 'endEmergPrep' #we have to go back to make sur algos are good
###################################################################

################ InteriorCartoGraphie #####################
global restartAlgo
restartAlgo = 0
class CartographieBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endCartographieBuilding'])
        global  cartoExitMode
        cartoExitMode = 0
    def execute(self, userdata):
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,cartoExitMode,restartAlgo
        statePub.publish(String("CartographyBuilding"))
        stateInterPub.publish(Int8(5))
        
        if restartAlgo:
           print "tt" #restr ccny hector etc 

        servStartDiag()
        rospy.loginfo("CartographieBuilding")
        #make sure Cytron is good
        #start algo deplacement 
        drivStart = rospy.ServiceProxy('car_controller/starter',Start_Control)
        ast = rospy.Publisher("/start_astar",Bool)
        if initData.autonomous_level:
           skm = StartKillMsg()
           skm.action  = 1
           skm.type = 1
           skm.nId = 4
           startKillPub.publish(skm)#start algo gestion maenouvre   
           skm.action  = 1
           skm.type = 0
           skm.nId = 5
           startKillPub.publish(skm) #start driving algo
           skm.action  = 1
           skm.type = 0
           skm.nId = 6
           startKillPub.publish(skm) #start astar not calculing
           rospy.sleep(2)
           try:
              drivStart(True)
           except rospy.ServiceException, e:
              print "Service call failed: %s"%e
           
        #if not teleOp launch algo point gps donne a l'avance ou lit ici
        #needed PKG : ccny hector hokuyo pwm_serial_send car_controller (astar_path) 
        if cartoExitMode == 3:
           servStartDiag()
           return 'endCartographieBuilding'
        cartoExitMode =0 #assumption mode preempted
        skm = StartKillMsg()
        skm.action  = 1
        skm.type = 1
        skm.nId = 5
        startKillPub.publish(skm)#start drift detection
        if initData.autonomous_level == 2:
           ast.publish(Bool(True))

        start = rospy.get_time()
        while rospy.get_time()-start<2*60:#send by callback wait 2 min then procedural stop except if preempted
            if self.preempt_requested():
                rospy.loginfo("interior Cartographie is being preempted")
                skm.action  = 0
                skm.type = 1
                skm.nId = 5
                startKillPub.publish(skm)#stop drift detection
                try:
                   drivStart(False)
                except rospy.ServiceException, e:
                   print "Service call failed: %s"%e
                self.service_preempt()
                ast.publish(Bool(False))
                return 'preempted'
            rospy.sleep(1.0/20.0)
        try:
           drivStart(False)
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
        cartoExitMode = 1 #mode normal : going into procStop then re Carto
        #get status mapping
        skm.action  = 0
        skm.type = 1
        skm.nId = 5
        startKillPub.publish(skm)#stop drift detection
        status = rospy.ServiceProxy("get_mapping_status",GetMappingStatus)
        if status():
           cartoExitMode = 3
        if cartoExitMode == 1 :
           self.service_preempt()
           try:
              drivStart(False)
           except rospy.ServiceException, e:
              print "Service call failed: %s"%e
           ast.publish(Bool(False))
           return 'preempted'
        WS.sendCommand(1500,1500)
        try:
           drivStart(False)
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
        servStartDiag()
        ast.publish(Bool(False))
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,cartoExitMode
        statePub.publish(String("ProceduralStop"))
        stateInterPub.publish(Int8(6))
        WS.sendCommand(1500,1500)
        if not cartoExitMode or self.preempt_requested():
           self.service_preempt()
           return 'preempted'
        servStartDiag()        
        rospy.loginfo("ProceduralStop")
        save = rospy.ServiceProxy("/save_shot",Save_inst_srv)
        save(Save_inst_srv(True))

        if self.preempt_requested():
            rospy.loginfo("ProceduralStop is being preempted")
            self.service_preempt()
            return 'preempted'
        servStartDiag()
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,cartoExitMode,firstTime
        WS.sendCommand(1500,1500)
        servStartDiag()
        statePub.publish(String("EmergencyStopInt"))   
        rospy.loginfo("EmergencyStopInt")
        #check algo state       
        #needed PKG sauvegarde 
        save = rospy.ServiceProxy("/save_shot",Save_inst_srv)
        save(Save_inst_srv(True))
        result = WS.waitForRemoteGo(60*60)
        if result.data:
           return 'endEmer_Carto'
        return 'endEmer_Exit'



###################################################################


###################### Exit Building ##############################

class ExitBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endExitBuilding'])

    def execute(self, userdata):
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,listener,LatEntry,LongEntry,astar_arrived
        statePub.publish(String("ExitBuilding"))
        stateInterPub.publish(Int8(7))
        servStartDiag()
        rospy.loginfo("ExitBuilding")
        if not initData.autonomous_level:
           servStartDiag()
           return 'teleop' 
        ast = rospy.Publisher("/start_astar",Bool)
        ast.publish(Bool(True))
        astar_arrived = False
        arrivedSub = rospy.Subscriber("/astar_arrived",Empty,astar_arrived_cb)
        #start algo deplacement 
        drivStart = rospy.ServiceProxy('car_controller/starter',Start_Control)
        try:
           drivStart(True)
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
        s = rospy.Publisher("/exit_build",Empty)
        s.publish(Empty())
        s.unregister()
        msgA = AstarPoint()
        (e,msgA.x,msgA.y) =LLtoUTM.LLtoUTM(LatEntry,LongEntry)
        s = rospy.publisher("/astar_set_point", AstarPoint)
        s.publish(msgA)
        s.unregister()
        #needed PKG astar_path pwm_serial_send mavros gps ?
        Lat = LatEntry #coord of entry 
        Long = LongEntry
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = trans1[0]
        msgSRV.yLat1 = trans1[1]
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
       
        while not res or not astar_arrived:#wait for GPS for 5 min
             if self.preempt_requested():
                rospy.loginfo("Exit Building is being preempted")
                self.service_preempt()
                ast.publish(Bool(False))
                try:
                  drivStart(True)
                except rospy.ServiceException, e:
                  print "Service call failed: %s"%e
                return 'preempted'
             try:
                (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
        try:
           drivStart(False)
        except rospy.ServiceException, e:
           print "Service call failed: %s"%e
        ast.publish(Bool(False))
        WS.sendCommand(1500,1500)
        servStartDiag()
        return 'endExitBuilding'

def astar_arrived_cb(msg):
    global astar_arrived
    astar_arrived = True

def exit_cb(outcome_map):
    if outcome_map['ExitBuild'] == 'teleop':
        return True
    elif outcome_map['ExitBuild'] == 'endExitBuilding':
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
    elif outcome_map['ExitBuild'] == 'teleop':
        return 'exit_teleop'
    else:
        return 'exit_stop'
##
class ExitBuildingTeleOp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endExitBuilding'])

    def execute(self, userdata):
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv
        statePub.publish(String("ExitBuildingTeleOp"))
        stateInterPub.publish(Int8(8))
        #start astar path and driving algo too
        servStartDiag()
        rospy.loginfo("ExitBuildingTeleOp") 
        exitTeleopEnd = 1
        while not exitTeleopEnd: 
             if self.preempt_requested():
                rospy.loginfo("Exit Building Telop is being preempted")
                self.service_preempt()
                return 'preempted'
             rospy.sleep(1.0/20.0)
        #needed PKG  pwm_serial_send mavros gps ?
        servStartDiag()
        WS.sendCommand(1500,1500)
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData
        WS.sendCommand(1500,1500)
        servStartDiag()
        rospy.loginfo("EmergencyStopExitBuilding")
        msg = WS.waitForRemoteGo(60*60)
        if msg.data:
           return 'endEmer_End'
        else:
            return 'endEmer_Exit'



###################################################################


###################### End Exit Building ##########################

class EndExitBuilding(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endEx_CartoGPS','endEx_CartoGPSArdu','endEx_CartoTeleOp'])

    def execute(self, userdata):
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData
        statePub.publish(String("EndExit"))
        stateInterPub.publish(Int8(9))
        servStartDiag()
        rospy.loginfo("EndExitBuilding")
        #make sure Cytron is good
        #start algo deplacement       
        #needed PKG sauvegarde ?

        servStartDiag()
        #stop ccny hector ??
        if initData.autonomous_level:
           return 'endEx_CartoGPS'
        
        return 'endEx_CartoTeleOp'

###################################################################


###################### Return Home ################################

class ReturnHomeGPS(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['endReturnHomeGPS'])

    def execute(self, userdata):
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData, listener, waypointsGPS
        statePub.publish(String("ReturnHomeGPS"))
        stateInterPub.publish(Int8(10))
        servStartDiag()
        rospy.loginfo("ReturnHomeGPS")
        #need restart algo ?
        #start algo deplacement 
        #needed PKG mavros pwm_serial_send algo
        #launch GPS algo plus evitement obstacle
        #wait for GPS waipoint(s)
        skm = StartKillMsg()
        skm.action  = 1
        skm.type = 1
        skm.nId = 1
        startKillPub.publish(skm)#start algo deplacement
        rospy.sleep(2)

        Lat = 42.954306
        Long = 10.599778
        r = rospy.Publisher('start_gps_follow',Int8)
        
        result = WS.waitForGPSData(self,2*60)
        if result=='preempted':
           return 'preempted'
        elif result=='Error' and waypointGPS == 0:
             err  = ErrorMessage()
             err.type1=3
             selfErrorPub.publish(err)
             rospy.sleep(1)
             if self.preempt_requested():
                rospy.loginfo("Go building GPS is being preempted")
                self.service_preempt()
                return 'preempted'
        elif waypointGPS != 0:
           text
           for i in waypointsGPS[::-1]:
              text = i+'@'
           text  = text[:-1]
           temp = rospy.Publisher('gps_string',String)
           temp.publish(String(txt))
           temp.unregister()
        else:
           Lat =result[0][0]#last coord of list of waypoints
           Long = result[0][1]

        r.publish(Int8(1))#algo follow can really start
        msgSRV = IsNearRequest()
        msgSRV.type1 = 0
        msgSRV.type2 = 1
        msgSRV.xLont1 = 11
        msgSRV.yLat1 = 45
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res:#wait for GPS for 5 min and check when we are in range 
             if self.preempt_requested():
                r.publish(Int8(0))
                r.unregister()
                rospy.loginfo("Go building GPS is being preempted")
                self.service_preempt()
                return 'preempted'

             try:
                (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             #if end algo change algo to direct one
        r.publish(Int8(0))
        r.unregister()
        WS.sendCommand(1500,1500)
        servStartDiag()
        skm.action  =0
        skm.type = 1
        skm.nId = 1
        startKillPub.publish(skm)#kill gps_follow
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData,listener
        statePub.publish(String("ReturnHomeArdu"))
        stateInterPub.publish(Int8(11))
        servStartDiag()
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
        msgSRV.xLont1 = 11
        msgSRV.yLat1 = 45
        msgSRV.xLont2 = Long
        msgSRV.yLat2 = Lat
        msgSRV.threshold = 1
        res = is_near_srv(msgSRV).response
        while not res:#wait for GPS for 5 min
             if self.preempt_requested():
                rospy.loginfo("Return Home GPS Ardu is being preempted")
                self.service_preempt()
                return 'preempted'
             try:
                (trans1,rot1) = listener.lookupTransform("local_origin", "fcu", rospy.Time(0))
             except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
             msgSRV.xLont1 = trans1[0]
             msgSRV.yLat1 = trans1[1]
             res = is_near_srv(msgSRV).response
             #if end algo change algo to direct one
        #stop autonomous ardu
        WS.sendCommand(1500,1500)
        servStartDiag()
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData
        statePub.publish(String("ReturnHomeTeleOp"))
        stateInterPub.publish(Int8(12))
        servStartDiag()
        rospy.loginfo("ReturnHomeTeleOp")
        #needed PKG mavros pwm_serial_send 
        # signal to end 
        global teleOpGOend
        s = rospy.Subscriber('/restart_msg',Empty,remoteCallback) 
        teleOpGOend = 1
        while not teleOpGOend:#send by callback
            if self.preempt_requested():
                rospy.loginfo("Go building TeleOp is being preempted")
                self.service_preempt()
                return 'preempted'
            rospy.sleep(1.0/20.0)
        s.unregister()
        servStartDiag()
        return 'endReturnHomeTeleOp'


def retTeleOp_cb(outcome_map):
    if outcome_map['RetTeleOp'] == 'endReturnHomeTeleOp':
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
        global selfErrorPub,stateInterPub,statePub,servStartDiag,is_near_srv,initData
        WS.sendCommand(1500,1500)
        servStartDiag()
        rospy.loginfo("EmergencyStopReturn")
        msg = WS.waitForRemoteGo(60*60)
        if msg.data:
           return 'endEmergEnd'
        else:
          if state==0:
            return 'endEmergGPS'
          elif state==1:
            return 'endEmergGPSArdu'
          else:
            return 'endEmergTeleOp'
           
        return 'endEmergTeleOp'

###################################################################

def changeInitData_cb(req):
    global initData
    initData.heading = req.data
    return ChangeInitDataResponse()
###################################################################

###################################################################
def ShutdownCallback():
    global sis
    sis.stop()
    rospy.loginfo("shutdown state machine")

def monitor_cb(ud, msg):  
    global error_message
    error_message = msg
    return False

##################################################################
#################### Start of node ###############################
##################################################################

rospy.init_node('ai_mapping_robot')
rospy.on_shutdown(ShutdownCallback)

serv = rospy.Service('change_initData', ChangeInitData, changeInitData_cb)
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
    smach.Concurrence.add('GPS_STOP_STUCK', smach_ros.MonitorState("/GPS_stop_stuck",ErrorMessage,monitor_cb))

with gobuildingGPSArdu_concurrence:
    smach.Concurrence.add('GPSA_norm', GoBuildingGPSArdu())
    smach.Concurrence.add('GPSA_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))

with gobuildingTeleOp_concurrence:
    smach.Concurrence.add('TeleOp', GoBuildingTeleOp())
    smach.Concurrence.add('TeleOp_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))

with initEntry_concurrence:
    smach.Concurrence.add('InitEntry', InitEntryBuilding())
    smach.Concurrence.add('Entry_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))


with interiorCartoGraphie_concurrence:
    smach.Concurrence.add('InterCarto', CartographieBuilding())
    smach.Concurrence.add('Carto_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))

with procedural_stop_concurrence:
    smach.Concurrence.add('ProcStop', ProceduralStop())
    smach.Concurrence.add('ProcStop_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))


with exitBuilding_concurrence:
    smach.Concurrence.add('ExitBuild', ExitBuilding())
    smach.Concurrence.add('ExitB_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))

with exitBuildingTeleOp_concurrence:
    smach.Concurrence.add('ExitTeleOp', ExitBuildingTeleOp())
    smach.Concurrence.add('ExitTeleOp_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))

with returnHomeGPS_concurrence:
    smach.Concurrence.add('RetHomeGPS', ReturnHomeGPS())
    smach.Concurrence.add('RetHomeGPS_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))

with returnHomeGPSArdu_concurrence:
    smach.Concurrence.add('RetHomeGPSArdu', ReturnHomeArdu())
    smach.Concurrence.add('RetHomeGPSArdu_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))

with returnHomeTeleOp_concurrence:
    smach.Concurrence.add('RetTeleOp', ReturnHomeTeleOp())
    smach.Concurrence.add('RetTeleOp_STOP', smach_ros.MonitorState("/stop_command",ErrorMessage,monitor_cb))

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
                                                                                 'endEmer_Exit':'ExitTeleOp'})

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

global sis
sis = smach_ros.IntrospectionServer('AI_EURATHLON', sm_cal, '/GENERAL_BEHAVIOUR')
selfErrorPub = rospy.Publisher("/stop_command",ErrorMessage)
stateInterPub = rospy.Publisher("/set_state_proxy",Int8)
statePub = rospy.Publisher("/State_Publisher",String)
servStartDiag = rospy.ServiceProxy('start_diag', sSrv.Empty)
is_near_srv = rospy.ServiceProxy('IsNear', IsNear)
listener = tf.TransformListener()
sis.start()
outcome = sm_cal.execute()
#rospy.spin()
sis.stop()
