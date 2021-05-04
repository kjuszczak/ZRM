#!/usr/bin/env python
from os.path import join
import roslib;
import rospy
from soc.msg import Current
import random
import scipy.io

def load_mat(index = 0):
  mat = scipy.io.loadmat('/home/kamil/catkin_ws/src/soc/src/B0005.mat', squeeze_me=True)
  data = mat['B0005']['cycle'].item()['data']
  current = list(data[index]['Current_measured'].item())
  time = list(data[index]['Time'].item())
  diff_time = [time[i] - time[i-1] for i in range(1, len(time))]
  return diff_time, current

def gen_number():
    pub = rospy.Publisher('actualCurrent', Current, queue_size=10)
    rospy.init_node('current_generator')

    diff_time, current = load_mat(index=0)
    charge_state = {
      'current': current,
      'diff_time': diff_time
    }

    diff_time, current = load_mat(index=1)
    discharge_state = {
      'current': current,
      'diff_time': diff_time
    }

    current = [random.uniform(-10, 10) for i in diff_time]
    random_state = {
      'current': current,
      'diff_time': diff_time
    }

    i = 0
    limit = len(random_state['current'])
    while not rospy.is_shutdown():
        msg = Current()
        if i < limit:
          msg.current = random_state['current'][i]
          msg.time = random_state['diff_time'][i]
          pub.publish(msg)
        else:
          i = 0
          continue
        rospy.sleep(0.05)
        i = i + 1

if __name__ == '__main__':
  try:
    gen_number()
  except Exception, e:
    print(e)