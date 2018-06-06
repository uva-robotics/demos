# Tim Test
This package is mainly for testing purposes. Should you still want to use it, see the guide below. Unfortunately, this package cannot be used standalone, except to produce some funny messages on the /animation topic.

However, when used in combination with pepper_animation, the following can be done:
- Pepper's eyes will start spinning around as long as the tim_test program is running
- Pepper will meow if head is touched
  - There is a 10% chance this meow will be an angry one
  - In addition, the eyes animation will switch to a random color

## How 2 run:
### First time run:
In order to run this program with all functionalities, please make sure both tim_test/src/main.py and pepper_animation/src/main.py are executable. If they aren't, use:
```
sudo chmod +x src/tim_test/src/main.py
sudo chmod +x src/pepper_animation/src/main.py
```
Once that is done, compile all packages if not done already:
```
catkin_make
```
Congratulations, your packages are now ready for use
### Startup:
#### Launching pepper_animation
First, make sure the ip-address is set correctly:
```
rosrun pepper_animation main.py ip
```
This returns either the IP address currently believed to be Pepper's, or that no address is set. If the address is incorrect or not set, set it using:
```
rosrun pepper_animation main.py x.x.x.x
```
**x.x.x.x** is to be replaced with Pepper's IP.

After the correct IP address is set, launch pepper_animation using:
```
rosrun pepper_animation main.py
```
#### Launching tim_test
Once pepper_animation is up and running, boot up tim_test using:
```
rosrun tim_test main.py
```
If you wish to prohibit the program from playing sound, run tim_test with this command instead:
```
rosrun tim_test main.py mute
```
If everything went according to plan, you should now see that the spinning animation at pepper's eyes is running.

## How 2 quit
Quitting in both tim_test and pepper_animation is fairly easy. While running, simply press Ctrl+C to terminate the program, and the program itself will take care that all threads are stopped. In the slight chance that the program fails in doing this, press Ctrl+Z to hard-terminate it. In theory, no python services should be left running, but this can be checked using:
```
top
```
If there are any, kill them using:
```
killall python
```
It should be noted that pepper_animation is designed to run as a service. This means you can keep it running, and it will resume doing it's thing when tim_test is booted again.
