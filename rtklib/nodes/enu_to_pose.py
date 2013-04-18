#!/usr/bin/env python
import roslib
roslib.load_manifest('rtklib')
import rospy
import sys
import socket
import re
import time
import calendar

from sensor_msgs.msg import TimeReference
from geometry_msgs.msg import PoseStamped,TransformStamped
from tf.msg import tfMessage
from tf import transformations
import tf


#Add the tf_prefix to the given frame id
def addTFPrefix(frame_id):
    prefix = ""
    prefix_param = rospy.search_param("tf_prefix")
    if prefix_param:
        prefix = rospy.get_param(prefix_param)
        if prefix[0] != "/":
            prefix = "/%s" % prefix

    return "%s/%s" % (prefix, frame_id)


if __name__ == "__main__":
    # Init publisher
    rospy.init_node('enu_to_pose')
    posepub = rospy.Publisher('enu', PoseStamped)
    timepub = rospy.Publisher('time_reference', TimeReference)
    tfpub = tf.TransformBroadcaster()
    tflisten = tf.TransformListener()

    # Init rtklib port
    host = rospy.get_param('~host', 'localhost')
    port = rospy.get_param('~port', 3333)
    global_frame_id = rospy.get_param('~global_frame_id', '/map')
    odom_frame_id   = rospy.get_param('~odom_frame_id',   '/odom')
    base_frame_id   = rospy.get_param('~base_frame_id',   '/base_link')
    publish_tf      = rospy.get_param('~publish_tf',      True)

    # Quality parameters
    accept_quality  = rospy.get_param('~quality','1,2,4,5,6')
    accept_quality  = [int(x) for x in accept_quality.split(',')]
    accept_num_sats = int(rospy.get_param('~min_sat', 5))
    accept_ratio    = float(rospy.get_param('~min_ratio', 1.0))

    enu = PoseStamped()
    enu.header.frame_id = '/map'
    time_ref_source = rospy.get_param('~time_ref_source', global_frame_id)
    gpstime = TimeReference()
    gpstime.source = time_ref_source
    trans_corr = (0.0, 0.0, 0.0)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((host, port))
        file = sock.makefile()

        while not rospy.is_shutdown():
            data = file.readline()
            time_now = rospy.get_rostime()
            fields = re.split('\s+', data)
            if fields[0] == '%': continue
            assert len(fields) is 16

            quality = int(fields[5])
            nr_sats = int(fields[6])
            ratio   = float(fields[14])

            # Pose
            enu.header.stamp = time_now
            enu.pose.position.x = float(fields[2])
            enu.pose.position.y = float(fields[3])
            enu.pose.position.z = float(fields[4])

            # Timeref
            # Expects time as UTC
            gpstime.header.stamp = time_now
            t = time.strptime(fields[0] + ' ' + fields[1], "%Y/%m/%d %H:%M:%S.%f")
            gpstime.time_ref = rospy.Time.from_sec(calendar.timegm(t))

            posepub.publish(enu)
            timepub.publish(gpstime)

            if not publish_tf: continue

            # Transform
            if quality in accept_quality and nr_sats >= accept_num_sats and ratio >= accept_ratio:
                try:
                    tflisten.waitForTransform(odom_frame_id, base_frame_id, time_now, rospy.Duration(1.0))
                    (trans,rot) = tflisten.lookupTransform(odom_frame_id, base_frame_id, time_now)
                except tf.Exception:
                    print("Catched tf.Exception, i.e. no transform received in time, using zero-values, (just for testing)")
                    (trans, rot) = ((0,0,0), (0,0,0,0.1))

                trans_gps = (float(fields[2]), float(fields[3]), float(fields[4]))
                trans_corr = (trans_gps[0] - trans[0], trans_gps[1] - trans[1], trans_gps[2] - trans[2])
                rot = (0.0, 0.0, 0.0, 1.0)

            tfpub.sendTransform(trans_corr, rot, rospy.get_rostime(), odom_frame_id, global_frame_id)

    except rospy.ROSInterruptException:
        sock.close()

