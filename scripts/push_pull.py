#! /usr/bin/env python

#####################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2016, Michał Drwięga
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#####################################################################

import rospy, math
from geometry_msgs.msg import Twist, Wrench

class PushPullTeleop(object):
  linear_force_threshold = 0.1
  linear_vel = 0.1
  torque_threshold = 0.1
  angular_vel = 0.1

  def init(self):
    # Initial values
    self.force_msg = Wrench()
    self.update_rate = 50   # Hz
    self.alive = True

    # Setup publishers and subscriber
    self.pub_twist = rospy.Publisher('/man_cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('jc/steering_wheel_force', Wrench, self.cb_steering_wheel_force)

  def cb_steering_wheel_force(data):
    force_msg = data
    print 'Received force: ', force_msg

  def run(self):
    try:
      self.init()
      r = rospy.Rate(self.update_rate) # Hz
      while not rospy.is_shutdown():
        self.update()
        r.sleep()
    except rospy.exceptions.ROSInterruptException:
      pass

  def update(self):
    if rospy.is_shutdown():
      return

    twist = Twist()

    # Linear force to velocity
    if self.force_msg.force.x == 0:
      twist.linear.x = 0
    elif self.force_msg.force.x > self.linear_force_threshold:
      twist.linear.x = self.linear_vel
    else:
      twist.linear.x = -self.linear_vel

    # Torque z axis to angular velocity
    if self.force_msg.torque.z == 0:
      twist.angular.z = 0
    elif self.force_msg.torque.z > self.torque_threshold:
      twist.angular.z = self.angular_vel
    else:
      twist.angular.z = -self.angular_vel

    self.pub_twist.publish(twist)
    print 'Twist:\tlinear %.2f\tangular %.2f' % (twist.linear.x,twist.angular.z)

if __name__ == '__main__':
  rospy.init_node('push_pull_teleop')
  teleop = PushPullTeleop()
  teleop.run()
