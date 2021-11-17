import time
import roslibpy

client = roslibpy.Ros(host='localhost', port=9090)
client.run()

class Talker:
    def __init__(self):
        self.talker = roslibpy.Topic(client, '/lgsvl_cmd', 'lgsvl_msgs/VehicleControlData')
        self.seq=0

    def talker_pub(self,acceleration_pct,braking_pct,target_wheel_angle,target_wheel_angular_rate,target_gear):
        self.seq=self.seq+1
        self.talker.publish(
            roslibpy.Message(
            {"header": roslibpy.Header(stamp=roslibpy.Time.now(), frame_id='',seq=self.seq),
            'acceleration_pct': acceleration_pct,
            'braking_pct': braking_pct,
            'target_wheel_angle': target_wheel_angle,
            'target_wheel_angular_rate': target_wheel_angular_rate,
            'target_gear': target_gear
            }))
        #self.seq=self.seq+1
        #print(self.seq)

talker=Talker()
PI=3.14
while client.is_connected:
    obstacle=input("0=pas obstacle 1=obstacle 2=tourner_gauche 3=tourner_droite : ")
    try:
        obstacle=int(obstacle)
    except:
        obstacle=0
    #acceleration_pct,braking_pct,target_wheel_angle,target_wheel_angular_rate,target_gear
    if(obstacle==0):
        talker.talker_pub(1,0,0,0,1)
        #time.sleep(5)
    elif(obstacle==1):
        talker.talker_pub(0,1,0,0,1)
    elif(obstacle==2):
        talker.talker_pub(0,0,-PI/4,0,1)
    elif(obstacle==3):
        talker.talker_pub(0,0,PI/4,0,1)



    
    print('Sending message...')
    #time.sleep(1)

talker.unadvertise()

client.terminate()