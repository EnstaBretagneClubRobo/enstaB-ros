#!/usr/bin/env python
import rospy
import tf
import geometry_msgs
import tf.transformations as trans

class bufferTrans(object):
    def __init__(self,nBufferSize=50):
        self.bufferSize = nBufferSize
        self.TransList = []
        self.rotList = []
        self.IsGoodList = []

    def add(self,trans,rot,IsGood):
        if not IsGood:
            self.addReal(0,0,0)
        else:
            self.addReal(trans,rot,1)
    
    def addReal(self,trans,rot,IsGood):
        if len(self.TransList)== self.bufferSize:
           self.TransList = self.TransLit[1:]
           self.rotList = self.rotList[1:]
           self.IsGoodList = self.IsGoodList[1:]
        self.TransList.append(trans)
        self.rotList.append(rot)
        self.IsGoodList.append(IsGood)

    def getDiff(self,futur=0,past=1):
        if len(self.TransList)-futur < 0 or len(self.TransList)-past < 0:
           ROS_ERROR("trying to see past the buffer size")
           return 0
        else:
           euler = trans.euler_from_quaternion(self.rotList[-futur]*trans.quaternion_inverse(self.rotList[-past]))
           TransVect = (self.TransList[-futur][0]-self.TransList[-past][0],
                        self.TransList[-futur][1]-self.TransList[-past][1],
                        self.TransList[-futur][2]-self.TransList[-past][2])
           return (TransVect,euler)

    def getLastVector(self):
        n = 2
        i = 0
        resultIndex = (0,0);
        while n and i < len(self.TransList):
            if self.IsGoodList[-i]:
                resultIndex[2-n] = i
                n=n-1
            i=i+1
        return resultIndex
        
def getLastVector(listt,cond):
    n = 2
    i = 0
    resultIndex = (0,0);
    while n and i < len(listt):
        if listt[-i]==cond:
            resultIndex[2-n] = i
            n=n-1
        i=i+1                          
    return resultIndex
########################################################
class drift_detect_node(object):
    def __init__(self):
        rospy.init_node('drift_detection')
        rospy.on_shutdown(self.ShutdownCallback)
        self.listener = tf.TransformListener()
        self.bufferHokuyo = bufferTrans()
        self.bufferKinect = bufferTrans()
        self.bufferFakeOdo = bufferTrans()

    def ShutdownCallback(self):
        print 'shutdown' #send stop messages 

    def spin(self):
        hokuyo = 1
        kinect = 1
        fakeOdo = 1
        rate = rospy.Rate(1.0)
        trans1 = (0,0,0)
        rot1 = (0,0,0,0)
        trans2 = (0,0,0)
        rot2 = (0,0,0,0)
        trans3 = (0,0,0)
        rot3 = (0,0,0,0)
        
        while not rospy.is_shutdown():
            try:
                (trans1,rot1) = self.listener.lookupTransform("proxy_frame", "laser", rospy.Time(0))
                hokuyo = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                hokuyo = 0
            try:
                (trans2,rot2) = self.listener.lookupTransform("proxy_frame", "camera_link", rospy.Time(0))
                kinect = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                kinect = 0
            try:
                (trans3,rot3) = self.listener.lookupTransform("proxy_frame", "roue", rospy.Time(0))
                fakeOdo = 1
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                fakeOdo = 0
            self.bufferHokuyo.add(trans1,rot1,hokuyo)
            self.bufferKinect.add(trans1,rot1,kinect)
            self.bufferFakeOdo.add(trans1,rot1,fakeOdo)
            
            self.analyseBuffer()
            rate.sleep()

    def analyseBuffer(self):
        binTime = [(x+y+z)  for (x, y,z) in zip(self.bufferHokuyo.IsGoodList,self.bufferKinect.IsGoodList,self.bufferFakeOdo.IsGoodList)]
        index = getLastVector(binTime,3)
        if not index[0] >= index[1]:
           (Htrans,Hrot) = self.bufferHokuyo.getDiff(index[0],index[1])
           (Ktrans,Krot) = self.bufferKinect.getDiff(index[0],index[1])
           (Ftrans,Frot) = self.bufferFakeOdo.getDiff(index[0],index[1])


   




node = drift_detect_node()
node.spin()

