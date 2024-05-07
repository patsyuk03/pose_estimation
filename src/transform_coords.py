#!/usr/bin/env python

import rospy
import tf
from find_object_2d.msg import ObjectsStamped
from geometry_msgs.msg import PoseStamped

class TfExample:
    def __init__(self):
        self.objFramePrefix = "object"
        self.targetFrameId = rospy.get_param("~target_frame_id", "")
        self.objFramePrefix = rospy.get_param("~object_prefix", self.objFramePrefix)
        self.subs = rospy.Subscriber("objectsStamped", ObjectsStamped, self.objectsDetectedCallback)
        self.pose_pub = rospy.Publisher('/pose_stamped', PoseStamped, queue_size=10)
        self.tf_listener = tf.TransformListener()

    def objectsDetectedCallback(self, msg):
        if msg.objects:
            targetFrameId = self.targetFrameId if self.targetFrameId else msg.header.frame_id
            multiSubId = 'b'
            previousId = -1
            for i in range(0, len(msg.objects.data), 12):
                id = int(msg.objects.data[i])

                multiSuffix = ""
                if id == previousId:
                    multiSuffix = "_" + multiSubId
                    multiSubId = chr(ord(multiSubId) + 1)
                else:
                    multiSubId = 'b'
                previousId = id

                objectFrameId = "{}_{}{}".format(self.objFramePrefix, id, multiSuffix)

                try:
                    pose = self.tf_listener.lookupTransform(targetFrameId, objectFrameId, msg.header.stamp)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as ex:
                    rospy.logwarn(str(ex))
                    continue

                rospy.loginfo("{} [x,y,z] [x,y,z,w] in '{}' frame: [{},{},{}] [{},{},{},{}]".format(
                    objectFrameId, targetFrameId,
                    pose[0][0], pose[0][1], pose[0][2],
                    pose[1][0], pose[1][1], pose[1][2], pose[1][3]))

                pose_stamped_msg = PoseStamped()
                pose_stamped_msg.header.stamp = rospy.Time.now()
                pose_stamped_msg.header.frame_id = targetFrameId
                pose_stamped_msg.pose.position.x = pose[0][0]
                pose_stamped_msg.pose.position.y = pose[0][1]
                pose_stamped_msg.pose.position.z = pose[0][2]
                pose_stamped_msg.pose.orientation.x = pose[1][0]
                pose_stamped_msg.pose.orientation.y = pose[1][1]
                pose_stamped_msg.pose.orientation.z = pose[1][2]
                pose_stamped_msg.pose.orientation.w = pose[1][3]

                self.pose_pub.publish(pose_stamped_msg)

if __name__ == '__main__':
    rospy.init_node('transform_coords')
    sync = TfExample()
    rospy.spin()
