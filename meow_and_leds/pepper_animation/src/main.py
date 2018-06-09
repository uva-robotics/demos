#!/usr/bin/python

import rospy
import threading
import time
import sys

from std_msgs.msg import String

from naoqi import ALProxy

# Threads
class LedThread (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.running = False
        self.deamon = True
        try:
            self.leds = ALProxy("ALLeds",_IP,9559)
        except:
            print("Could not connect to Pepper (is her IP correct?)")

    # Init function
    def start(self):
        try:
            x = self.leds
        except NameError:
            print("No connection to Pepper, no use in starting animation :/")
            return
        if not self.running:
            # Init
            # Re-poke the connection
            self.running = True
            print("Animation started")
            threading.Thread.start(self)
        else:
            print("Animation already started")

    # While running, do...
    def run(self):
        # Do a funny lil' animation
        step = 0
        while self.running:
            # Loop through all four adjacent leds and turn 'em on
            for i in range(8):
                mod = (1.0 if (step == i) else 0.0)
                self.leds.setIntensity("FaceLedRight" + str(i), mod)
                self.leds.setIntensity("FaceLedLeft" + str(i), mod)
            # Sleep .1 sec
            then = time.time()
            while time.time() - then < .1:
                pass
            step = (step + 1) % 8

    # Quits the thread
    def stop(self):
        if self.running:
            # Stop it
            print("Stopping animation...")
            self.running = False
            while self.isAlive():
                pass
            # Reset the leds
            for i in range(8):
                self.leds.off("FaceLedRight" + str(i))
                self.leds.off("FaceLedLeft" + str(i))
            print("Animation stopped")
        else:
            print("Animation already stopped")

class Connection (threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.deamon = True
        self.running = False
        self.publisher = rospy.Publisher("/animation", String, queue_size=5)
        self.last_poke = time.time()

    def start (self):
        if not self.running:
            print("Starting Connection thread...")
            self.running = True
            if not self.isAlive():
                threading.Thread.start(self)
            print("Connection thread started.")
        else:
            print("Connection thread already running!")

    # Ping the pepper to let him know there's some connection
    def run (self):
        global led_thread
        self.last_poke = time.time()
        while self.running:
            if time.time() - self.last_poke >= 3 and led_thread.isAlive():
                print("Timeout, stopping animation...")
                led_thread.stop()
                # Overwrite the thread by a new one
                led_thread = None
                led_thread = LedThread()

    # Reset the connection counter
    def poke (self):
        self.last_poke = time.time()

    def stop (self):
        if self.running:
            print("Stopping Connection thread...")
            self.running = False
            while self.isAlive():
                pass
            print("Connection thread stopped.")
        else:
            print("Connection thread already stopped")

# Globals
LedColor = 0X00FFFFFF

def load_ip ():
    global _IP

    print("Loading pepper's IP address...")
    try:
        # Get the path to and including pepper_animation folder:
        path = ""
        for crumble in sys.argv[0].split("/"):
            path += crumble + "/"
            if crumble == "pepper_animation":
                break
        with open(path + "src/data/pepper_ip.txt", "r") as f:
            line = f.readline()
            while line != '':
                if line[:10] == "pepper_ip:":
                    _IP = line[10:]
                    while _IP[-1:] == "\n":
                        _IP = _IP[:-1]
                    break
                line = f.readline()
            try:
                return 0
            except NameError:
                _IP = "127.0.0.1"
                return 1
    except IOError:
        _IP = "127.0.0.1"
        return 1

def animation_listener (data):
    global led_thread
    global LedColor
    global connection

    if data.data == "begin:rotateEyesCustom":
        # Poke the connection thread
        led_thread.start()
        connection.poke()
    elif data.data == "end:rotateEyesCustom":
        led_thread.stop()
        # Overwrite the thread by a new one
        led_thread = None
        led_thread = LedThread()
    elif "new_color:0X00" in data.data:
        LedColor = int(data.data[10:], 16)
    elif data.data == "stillAlive":
        connection.poke()
    elif data.data == "playSound:meow":
        audio.play(meowID)
    elif data.data == "playSound:kggg":
        audio.play(kgggID)
    else:
        print("Message '" + data.data + "' not recognised, skipping...")

if __name__ == '__main__':
    # Setup thread
    global led_thread
    global connection
    global audio
    global meowID
    global kgggID

    # Read arguments
    print(sys.argv[0])
    if len(sys.argv) > 1:
        ip = sys.argv[1]
        if ip == "ip":
            # Load the ip address
            if load_ip() == 1:
                print("No ip address set yet (program will be using '127.0.0.1')")
                sys.exit(0)
            else:
                print("Pepper's IP address: " + _IP)
                sys.exit(0)
        elif ip.count(".") == 3:
            # Probably ip, check if numbers ranging 0-255:
            check = True
            for num in ip.split("."):
                try:
                    if int(num) < 0 or int(num) > 255:
                        check = False
                        break
                except ValueError:
                    check = False
                    break
            if check:
                # It's an ip!
                print("Setting '" + ip + "' as new pepper IP...")
                # Get the path to and including pepper_animation folder:
                path = ""
                for crumble in sys.argv[0].split("/"):
                    path += crumble + "/"
                    if crumble == "pepper_animation":
                        break
                # Save 2 file:
                try:
                    with open(path + "src/data/pepper_ip.txt", "w") as f:
                        f.write("pepper_ip:" + ip)
                        print("Succes")
                except IOError:
                    print("Error while setting IP")
            else:
                print("Given argument is gibberish, ignoring...")
        else:
            print("Given argument is gibberish, ignoring...")

    # Load pepper ip
    if load_ip() == 1:
        print("Something went wrong while loading pepper's IP, using '127.0.0.1' instead...")
    else:
        print("Load succes (" + _IP + ")")

    # Setup sound
    try:
        audio = ALProxy("ALAudioPlayer",_IP,9559)
        meowID = audio.loadFile("/home/nao/audio/cat_meow2.wav")
        audio.setVolume(meowID, 0.25)
        kgggID = audio.loadFile("/home/nao/audio/cat_screech.wav")
        audio.setVolume(kgggID, 0.25)
    except:
        print("Could not connect to Pepper (is her IP correct?)")

    led_thread = LedThread()
    connection = Connection()
    connection.start()

    # Begin a ros subscriber
    rospy.init_node("pepper_animation", anonymous=True)
    # Also start a publisher to push the thing
    #animationPublisher = rospy.Publisher("/animation", String, queue_size=5)
    #animationPublisher.publish("test")
    animationSubscriber = rospy.Subscriber("/animation", String, animation_listener)

    rospy.spin()

    # Stop the threads
    led_thread.stop()

    connection.stop()
    while led_thread.isAlive() or connection.isAlive():
        pass

    # Unload all soundfiles
    try:
        audio.stopAll()
        audio.unloadAllFiles()
    except NameError:
        print("No audio to unload")

    print("Succesfully completed script")
