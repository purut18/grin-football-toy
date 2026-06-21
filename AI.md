# Goal
- We're making a desk toy. The player shoots a ball towards a goal post. The goal post is gaurded by an AI-controlled goalkeeper that stops the ball.
- The goalkeeper's AI is an reinforcement algorithm that learns as the user plays. It is dumb at first but soon starts stopping goals.
- The pitch has three rows of IR sensors with 5 sensors each. They are layed flat on the pitch facing up. These readings from the sensors are used to train the goalkeeper.
    - The first two sensors are in front of the keeper and one is behind it. The sensor behind the keeper calculates if a goal was scored or not. It also sees where the goal was scored to train the reinforcement algorithm of the goalkeeper.

# Project Structure
This is a mono-repo contianing the code for the Arduino Uno Q (4GB), the RN mobile app, and the 3D print files.

## Folders
/arduino-app: The code for the Arduino Uno Q. It follows the strict format of an 'Arduino App Lab' app that can be run on the Uno Q. https://docs.arduino.cc/tutorials/uno-q/user-manual/

/app: The react native app that works with the Arduino app.

/3d-print: Irrelevant for you. 3D printing STL files.

/ai-learning: Learnings for the AI that can be used to code/think.