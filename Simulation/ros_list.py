#!/usr/bin/env python
# license removed for brevity

from __future__ import print_function
import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

radar = roslibpy.Topic(client, '/radar', 'std_msgs/String')
#radar.subscribe(lambda message: print('Heard talking: ' + message['data']))


while client.is_connected:
    obstacle=input("0=pas obstacle 1=obstacle : ")
    try:
        obstacle=int(obstacle)
    except:
        obstacle=0

    if(obstacle==0):
        radar.publish(roslibpy.Message({'data': '0'}))
    elif(obstacle==1):
        radar.publish(roslibpy.Message({'data': '1'}))

    print('Sending message...')

client.terminate()