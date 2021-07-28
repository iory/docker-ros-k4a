#!/usr/bin/env python

try:
    import queue
except ImportError:
    import Queue as queue

import audio_common_msgs.msg
import numpy  # Make sure NumPy is loaded before it is used in the callback
import rospy
import sounddevice as sd

assert numpy  # avoid "imported but unused" message (W0611)


def find_device(name):
    devices = sd.query_devices()
    for d in devices:
        if name in d['name'].encode('utf-8'):
            return d
    return None


def find_device_from_device_name(name):
    devices = sd.query_devices()
    for d in devices:
        if d['name'].encode('utf-8').startswith(name):
            return d
    return None


def main():
    rospy.init_node('audio_capture')
    device = rospy.get_param('~device', None)
    if device is None:
        device_name = rospy.get_param('~device_name', None)
        device_info = find_device_from_device_name(device_name)
    else:
        device_info = find_device(device)

    q = queue.Queue()

    def callback(indata, frames, time, status):
        """This is called (from a separate thread) for each audio block."""
        if status:
            print(status)
        q.put(indata.copy())

    pub = rospy.Publisher('/audio', audio_common_msgs.msg.AudioData,
                          queue_size=10)
    sd.default.dtype = 'int16'

    try:
        # soundfile expects an int, sounddevice provides a float:
        samplerate = int(device_info['default_samplerate'])
        channels = int(device_info['max_input_channels'])

        # Make sure the file is opened before recording anything:
        with sd.InputStream(samplerate=samplerate, device=device,
                            channels=channels, callback=callback):
            while not rospy.is_shutdown():
                tmp = q.get()
                msg = audio_common_msgs.msg.AudioData()
                msg.data = tmp.tobytes()
                pub.publish(msg)
    except Exception as e:
        rospy.logerr('{}'.format(e))


if __name__ == '__main__':
    main()
