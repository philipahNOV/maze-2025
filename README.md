# NOV maze 2025

This is the summer project at NOV summer 2024. 
The goal is to balance a ball through a maze without it falling into holes along the way.

## Getting Started

### Prerequisites
- [Python 3.x](https://www.python.org/downloads/)
- Required Python packages (listed in `requirements.txt`)

### Installation
1. Clone the repository:
    ```sh
    git clone https://github.com/yourusername/nov-maze-2024.git
    cd nov-maze-2024
    ```
2. Py Zed:
    The project uses a Zed camera, and its special API. This need to be downloaded manually as it does not work with pip install.
    ```sh
    import pyzed.sl as sl
    ```
### Running the Code
    There are multiple things to consider when running this code. First there is the jetson, this is the main hub. Here all calculations will be done. Then we have the Raspberry PI, here the UI is located.
    so when running the code, start the both jetson and the raspberry. This should start a handshake, and send the jetson into state 0.0. 
    Jetson is started by
    ```
    cd jetson\src
    python3 main.py
    ```
    It is important to note that it has to be python3, as of right now.
    To boot up the raspberry pi, go to the source file of the pi.
    ```
    cd pi\src
    python3 main.py
    ```

# 
For Next Year Students
All text below is information about the system at the moment. I 100% recomend reading this before starting over.
Login for raspberrypi is: login: raspberrypi, password: raspberry
Login for Jetson is: login: student, password: student 

## Good to know consepts
When moving the motors, you call arduino_thread.send_target_position(dir1,dir2,speed1, speend2). dir is the direction of the motor, 1/-1 makes the motor move up. 3/-3 makes them move down, and 2 is stand still. speed goes from 0 to 255, where 255 is max speed. (For the max speed look at data sheet, but i think it is 44mm/s without load.)
The camera has a corinate system that starts on the top left corner. So x = 0 is to the left moving right. y = 0 is top and moving down. 
Camera angle in radians is retrived with camera_thread.orientation, where [0] is x and [1] is y. The angle in degrees is retrived by camera_thread.orientation_deg. Moving the motors in a positive direction drives the angles to a potitive value. 


## Cleaning up the code
Eeeeee for saving time I have not fixed the fact that the actuators have flipped actuators. So when working with arduino its not (x,y,x_speed,y_speed) but (y,x,y_speed,x_speed). This is an easy fix by just flipping pin for x and y at the start, and their selective actuator feedback. But some of the code like elevator has hardcoded what to do, so that needs chaning. Also in the arduino_connection.py the function flipps the command. That is why it is (x,y,x_s,y_s) in python but not in the arduino.

## Arduino to Jetson coms
When our regulator team began working on regulating the system, they encountered a problem with the communication between the Arduino and Jetson. The primary issue was the delay in the Jetson receiving signals from the Arduino, which caused the regulation system to become unstable due to the slow speed. Although I don't have the exact numbers, I recall that the Jetson received a response in approximately 0.2 seconds. This delay made the back-and-forth communication too slow for the system's requirements. Additionally, the actuators cannot process changes in direction and speed faster than 0.2 seconds. Given the Jetson's 0.2-second delay and an additional 0.05-second delay for sending data, it results in a total of 0.25 seconds before the actuators receive information about changes. Here are my recommendations:
1. Set up a script to measure the speed of Arduino to Jetson communication: This will provide a better understanding of the actual communication speed (note that I have not tested this myself). This is important because we have currently eliminated all communication from Arduino to Jetson, relying solely on Jetson to Arduino communication. As a result, we cannot get feedback on the actuators' positions.
2. Stress test the actuators: Determine how quickly the actuators can change direction after receiving a command (within Arduino). 
3. Test direct actuator feedback to Jetson: We attempted to connect the actuator feedback pins directly to the Jetson to achieve zero delay, but this approach did not work as expected. Further testing is needed to determine why this method failed and to explore the possibility of using the output pins on the Jetson.
By addressing these issues, we can improve the communication speed and stability of the regulation system.

# Explaining the code
Code is written by Knut Selstad. If help is needed i answer on mail on knut.selstad@gmail.com

First of all, I would like to emphasize that keeping as much of the existing code as possible will save you a significant amount of time. Since I did not have a physical model at the start, many of the components were created without being tested until the final week. Therefore, I suggest starting at the beginning and checking what works and what does not.

The MQTT can be made much more robust than it is currently. Additionally, the AI model needs thorough testing. I also recommend reviewing each step from start to finish to determine if everything functions as intended. Since only one person developed all the code, a considerable amount of time was spent on isolated tasks, and many concepts and algorithms did not receive the necessary focus.

So i want to try to explain the code. This explenation is created to follow along, so that when seeing the code for the first time, you will have an easier time getting into it. First the architecture communication between the Jetson and the pi is a Sub/Pub, MQTT communication. The communication between the them is created using static IP of 192.168.1.3 (for the jetson), and 192.168.1.2 (for the pi). This connection is created using ethernett connection (eth0). Using this method the connection can stay up without internett. The communication between the arduino and the jetson is USB. 
Lets start at jetson/src/main.py. Here we boot up all the threads that are going to be used. The threads are 'MQTT', 'Camera', 'Arduino' and 'Main'. Lets start at the top and move down. 
# MQTT thread
The MQTT thread, boots up a connection to the broker on 192.168.1.3 on port 1883. It subsctibes to the topics "handshake/request", "jetson/command" and "jetson/state". So i want here to say that the jetson/command means that the pi has send jetson a command to do, and jetson/state is what state it is in. So an example is that the user presses a button on the screen, the pi sends a command to the jetson: "jetson/command:1.0". The jetson does the command, and sends "pi/state:1.0" to set the pi into the state that it commanded the jetson to go into. So command means that the main topic gets a command not sends it. When the MQTT thread is init it starts the handshake sequece. There it waits for a message on "handshake/response" if it retrives a responce it will awnser with "ack".
# Camera thread
For the camera thread it will start jetson/src/camera/cam_loop.py. There we init the reselution, the fps and other init states. I have added code to automaticly detect the board, so that it is the only thing that is beeing showed. This has been commented out bacuse of the lack of testing. When the next summer inters (Think the person reading this) i hope that more and better testing can be done to make it work 100% of the time. The camera thread updates the last frame and the orientation of the camera at the rate of the camera fps. The positive thing of this is that if there is any application that need the data from the camera it will gets the lates one at full fps. I think the way this thread is done is perfect. The latest frame is called: latest_frame and the last orientaion is called: orientation. So if another application is going to be calling the frame it will call: camera_thread.latest_frame or camera_thread.orientation.
# Arduino thread
For the 'Arduino' thread it is booting up the connection to an arduino that it finds at one of the ports to the jetson. The connection is made with a baud rate of 9600. For the Arduino it will sleep its thread until it retrives a command on the function "send_target_positions", this function is what makes the actuators move. The parameters for this fuction is dir1, dir2, speed1, speed2. Where dir is the direction of the actuator, and 1/-1 is increace the actuator heigth. 3/-3 is lowering the actuoator heigth. For speed, this is an integer between 0 and 255. where 255 is max speed. Be cearfull when working with high speeds, they are very high. The command is sent over serial, and the arduino thread is not waiting for a respose from the arduino, that means you can send commands as fast as you want. Not that it will actualy do the command that you send when having this high sending speed. So i recomend that you send a new command only if there is a change in direction or you want a speed change bigger then 20 or 30. Not overloading the arduino with unesessay commands can create a more reliable system.
# Main
## 0.0
This thread is the main thread, it has no name and is the one that runs all the other code. Lets start at the top of jetson\src\main.py, here all the threads are init, and the handshake between the pi and the jetson starts. When the handshake has completed, the main thread will go into either state 1.0 or 2.0 where 1.0 is automatic (automatic is what we have done this summer), and 2.0 is manuel (we have not done anything here this summer). 
## 1.0
When in state 1.0 the jetson will see if it has a path loaded, if it has a path loaded, then it is possible to jump into state 1.2. If not then you will stay on state 1.0. Then the user can press the scan board button. This makes the pi send a command to jetson to go intp state 1.1. 
## 1.1
Now, when in state 1.1 the main thread will start of by finding the goal and start. It will do this by going over the lines_ and detect_shapes code. This algorithm has written by me (Knut Selstad), and it is not perfect. The goal is to find a pentagon and a triangle. The problem is that normaly you would do something called contours with CV2 but, i found that this does not work, because of the imperferctions in the image. So i made my own detection algorithm. Lets go over it:
## Lines_
The lines_ functions create small lies between strait points in the image that is longer then 1. So this will create lines of lenght 1. Then it will sort this list, and start to connect the lines that are closes to together, if they are to far apart they will not connect. Then when the lines are connected, they are passed into a rdp algorithm, this will take points from start to end and find out if you can remove some points to create straiter lines. The goal of this is to go from lines that are not strait, to one long line. Each list then has its max, min in both x and y axis checked. If the dictace between them is more then 25% they are removed. The reason for this is because the triangle and the pentagon is as wide has they are high. So if a line does not have a square feature, then they are incorrect, example of this is just a strait line. Lastly, if the start and end points are to far apart they are removed.
## Dectect_shapes
On this new lines we can then aply the CV2 contours. Where the len of approx for triangle is 3 and perntagon is 5. Start is the triangle and the end is the pentagon.
## Path
When back at the 1.1 state with start and end, path of the board is going to be found. The board of where it can go and not go is desided on where there is darkere color and where there are lighter colors. So holes and walls (dark) will become obsticals. The path is found by the use of A* algorihtm with a twist. Insted of the agent getting reward for getting closer towards the goal it gets reward for staying away from the walls and holes. The punishment of getting close to the wall is very high this creates this "have to stay away from the wall at all costs" mindset for the agent. This algorihtm works perferct. The function will return the path that will take the ball from start to end and the board, (the black and white board where black is places it is not allowed to walk on). When a path is found a rdp algorihtm is run to remove the contiues line, and make the path points instead. This makes it so that the ball can go from point to point. When a path is found it the main thread now moves over to state 1.2
## 1.2
This state is a form of waiting state, when in this state 4 thins can happen. A new board is chosen, and the user needs to scan the board again. Sending it over to state 1.1. The user can select to go back to the menu, sending the state back to 0.0. Then it can do one of two things, either use a PID regulator to solve it, 1.3, or use AI to solve it, 2.3.
## The regulators
The regulators are from this point created by two different people, so when looking at the code for the PID, will have differances to it compared to the AI solution. I have created the AI solution but have not the time to test it as i want. 

# AI
The model can output three commands: increase height, decrease height, and maintain the current height. During training, decisions were made every 0.05 seconds, equivalent to 20 frames per second (fps). The change in angle per decision is approximately 0.2 degrees. This is based on the maximum speed of the actuators, which is around 44 mm per second. Thus, 44 mm/s divided by 20 frames equals 2.2 mm per 0.05 seconds, translating to roughly 0.2 degrees.

If you plan to train the model using AI, I suggest examining my approach first. Additionally, for reinforcement learning (RL), it is crucial to provide rewards and punishments based on the agent's immediate actions, rather than past actions, to avoid confusing the agent during training. My method involves a function that uses mathematical calculations to predict whether the current action is correct based on future outcomes. If you choose to use my learning method, I recommend reviewing the simulation to ensure its accuracy.

Please note that I am not an expert in AI and its use on regulation systems. My recommendation to review my training steps is intended to give you an overview of what has been done, rather than implying that I have created a perfect system.
