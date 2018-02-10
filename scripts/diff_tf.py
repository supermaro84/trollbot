#!/usr/bin/env python

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   
   ----------------------------------
   Portions of this code borrowed from the arbotix_python diff_controller.
   
diff_controller.py - controller for a differential drive
  Copyright (c) 2010-2011 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of Vanadium Labs LLC nor the names of its 
        contributors may be used to endorse or promote products derived 
        from this software without specific prior written permission.
  
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""

import rospy
from math import sin, cos

from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from std_msgs.msg import Int64


class DiffTf:

    def __init__(self):
        """Program initialization"""

        """Initialize ROS node"""
        rospy.init_node("diff_tf")
        rospy.loginfo("-I- %s started" % rospy.get_name())
        
        """Parameters"""
        self.rate = rospy.get_param('~rate', 10.0)                           # Rate of tf publishing
        self.ticks_meter = float(rospy.get_param('ticks_meter', 3000))       # Wheel encoder ticks / meter
        self.base_width = float(rospy.get_param('~base_width', 0.27))        # The wheel base width in meters
        
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')  # Name of the base frame of the robot
        self.odom_frame_id = rospy.get_param('~odom_frame_id', 'odom')       # Name of the parent frame of the robot
        
        self.encoder_min = rospy.get_param('encoder_min', -32768)            # Encoder minimum
        self.encoder_max = rospy.get_param('encoder_max', 32767)             # Encoder maximum
        self.encoder_range = self.encoder_max - self.encoder_min             # Encoder range

        """
        Magic numbers?
        
        low_wrap = -13107.5
        high_wrap = 13106.5
        """
        self.encoder_low_wrap = rospy.get_param('wheel_low_wrap', self.encoder_range * 0.3 + self.encoder_min)
        self.encoder_high_wrap = rospy.get_param('wheel_high_wrap', self.encoder_range * 0.7 + self.encoder_min)

        self.t_next = rospy.Time.now() + rospy.Duration(1.0/self.rate)       # Time at next update
        
        """Internal values"""
        self.enc_left = None                                                 # Last known left encoder value
        self.enc_right = None                                                # Last known right encoder value
        self.left = 0                                                        # Left encoder value
        self.right = 0                                                       # Right encoder value
        self.lmult = 0                                                       # Complete revolutions of the left encoder
        self.rmult = 0                                                       # Complete revolutions of the right encoder
        self.prev_lencoder = 0                                               # Previous reading from left wheel encoder
        self.prev_rencoder = 0                                               # Previous reading from right wheel encoder
        self.x = 0                                                           # x coordinate of position
        self.y = 0                                                           # y coordinate of position
        self.th = 0                                                          # Bearing of robot
        self.dx = 0                                                          # Linear velocity
        self.dr = 0                                                          # Angular velocity
        self.then = rospy.Time.now()                                         # Set initial time
        
        """Subscribers"""
        rospy.Subscriber("lwheel", Int64, self.left_wheel_callback)
        rospy.Subscriber("rwheel", Int64, self.right_wheel_callback)

        """Publishers"""
        self.odomPub = rospy.Publisher("odom", Odometry, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

    def spin(self):
        """Program operation"""

        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

    def update(self):
        """Update and publish state"""

        now = rospy.Time.now()
        if now > self.t_next:
            """Calculate time"""
            elapsed = now - self.then                             # Time elapsed since last update
            self.then = now                                       # Reset time of last update to current
            elapsed = elapsed.to_sec()                            # Convert ROS time to seconds
            
            """Calculate odometry"""
            if self.enc_left is None:                             # If initializing, set delta left and delta right to 0
                delta_left = 0
                delta_right = 0
            else:
                delta_left = (self.left - self.enc_left)          # Number of ticks on left encoder since last update
                delta_left = delta_left / self.ticks_meter        # Convert from ticks to meters
                delta_right = (self.right - self.enc_right)       # Number of ticks on right encoder since last update
                delta_right = delta_right / self.ticks_meter      # Convert from ticks to meters
            self.enc_left = self.left                             # Update left encoder record to new value
            self.enc_right = self.right                           # Update right encoder record to new value
            delta = (delta_left + delta_right) / 2                # Distance traveled is the average of the two wheels
            theta = (delta_right - delta_left) / self.base_width  # This approximation works for small angles
            self.dx = delta / elapsed                             # Calculate linear velocity
            self.dr = theta / elapsed                             # Calculate angular velocity

            if delta != 0:
                x = cos(theta) * delta                            # Calculate distance traveled in x direction
                y = -sin(theta) * delta                           # Calculate distance traveled in y direction
                self.x = self.x + (cos(self.th) * x - sin(self.th) * y)  # Increment x position
                self.y = self.y + (sin(self.th) * x + cos(self.th) * y)  # Increment y position

            if theta != 0:
                self.th = self.th + theta                         # Increment bearing

            """Create quaternion to describe robot pose"""
            quaternion = Quaternion()
            quaternion.x = 0.0
            quaternion.y = 0.0
            quaternion.z = sin(self.th / 2)
            quaternion.w = cos(self.th / 2)

            """Publish Transformation to robot frame"""
            self.odomBroadcaster.sendTransform((self.x, self.y, 0), (0.0, 0.0, quaternion.z, quaternion.w),
                                               rospy.Time.now(), self.base_frame_id, self.odom_frame_id)

            """Publish odometry"""
            odom = Odometry()
            odom.header.stamp = now                               # Set current time stamp
            odom.header.frame_id = self.odom_frame_id             # Set parent frame id
            odom.pose.pose.position.x = self.x                    # Set x coordinate
            odom.pose.pose.position.y = self.y                    # Set y coordinate
            odom.pose.pose.position.z = 0                         # Set z coordinate to zero
            odom.pose.pose.orientation = quaternion               # Set pose
            odom.child_frame_id = self.base_frame_id              # Set child frame id
            odom.twist.twist.linear.x = self.dx                   # Set linear velocity
            odom.twist.twist.linear.y = 0                         # No lateral velocity possible
            odom.twist.twist.angular.z = self.dr                  # Set angular velocity
            self.odomPub.publish(odom)                            # Publish odometry message

    def left_wheel_callback(self, msg):
        """Update left encoder position"""

        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap:
            self.lmult += 1
            
        if enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap:
            self.lmult -= 1
            
        self.left = 1.0 * (enc + self.lmult * self.encoder_range)
        self.prev_lencoder = enc

    def right_wheel_callback(self, msg):
        """Update right encoder position"""

        enc = msg.data
        if enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap:
            self.rmult += 1
        
        if enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap:
            self.rmult -= 1
            
        self.right = 1.0 * (enc + self.rmult * self.encoder_range)
        self.prev_rencoder = enc


def main():
    diff_transform = DiffTf()
    diff_transform.spin()


if __name__ == '__main__':
    main()

