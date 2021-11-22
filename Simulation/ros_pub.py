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

class Listener:
    def __init__(self):
        self.listener = roslibpy.Topic(client, '/radar', 'std_msgs/String')
        self.seq=0
        self.obs=0
    def receive_message(self,msg):
        self.obs = msg['data']

    def listener_sub(self):
        #self.listener.subscribe(lambda message: print('Heard talking: ' + message['data']))
        self.listener.subscribe(self.receive_message)



talker=Talker()
listener=Listener()
PI=3.14
already_accelerated=False
while client.is_connected:
    #obstacle=input("Simulation: \n 0: Accelerate \n 1: Obstacle (brake) \n 2: Turn left \n 3: Turn right \n Input: ")
    listener.listener_sub()
    obstacle=listener.obs;
        
    try:
        obstacle=int(obstacle)
    except:
        obstacle=0
    #acceleration_pct,braking_pct,target_wheel_angle,target_wheel_angular_rate,target_gear
    if(obstacle==0):
        print("No obstacle")
        if(already_accelerated==False):
            already_accelerated=True
            talker.talker_pub(1,0,0,0,1)
            time.sleep(5)       
    elif(obstacle==1):
        print("OBSTACLE")
        already_accelerated=False
        
        talker.talker_pub(0,1,0,0,1)
        time.sleep(5)
    elif(obstacle==2):
        talker.talker_pub(0,0,-PI/4,0,1)
    elif(obstacle==3):
        talker.talker_pub(0,0,PI/4,0,1)
    #print('Sending message...')
    #time.sleep(1)

talker.unadvertise()

client.terminate()