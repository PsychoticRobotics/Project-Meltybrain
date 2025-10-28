# melontype
## melontype is an open source meltybrain library, collaboratively written with ChatGPT
(see _Transcript of the coding session_ section for details.)

* THIS IS NOT PRODUCTION QUALITY CODE
  * it has worked in testing
    * it still needs a lot more testing
      * **it might not work properly** all the time.
  * **Use at your own risk!**

* NOT FOR USE IN MEDICAL OR MILITARY APPLICATIONS.
  * it's probably a bad idea for everyone.

## Current status
The main reason for this commit is to make sure there is a "good enough" version to go back to if the next radical code changes break everything. 

Let's pretend the previous commit was perfect: if you get commit 20d46d115944a705967b1c6af9ae70f9de6671d1 you should have a working robot. The description of functionality here is for that version. 

This code _basically_ works. The driving experience is not what it should be. You currently need to rotate the aim around as you move to keep it going straight, but this may be due to other bugs.
It can translate between marks 2 feet apart in my 4' long test box and back again in only a few seconds. I only need to be able to do 10 feet/minute. This is definitely fast enough for the NHRL 2025 Functionality test. It did the pinball thing where it bounces off walls when I missed my actual target and hit the wall.

The robot now uses a PID controller so the throttle is actually a percent of the maximum RPM selector instead of power controller for the ESCs directly.

There are two numbers displayed on the rotating surface. one of these is the RPM and the other is (i think) the current RPM percent?

The heading lock is rock steady. The heading stays fixed (assume it was fixed) and you just have to change your relative position with the right pot... see below for idealized driving instructions.

You can enter a mode where you edit the curve for the current robot by first clearing any existing config:
- plug the USB cable into the teensy
- open the serial monitor
- type 'c' and press enter. any previous config is cleared.
- disconnect the robot from USB and put the top plate back on.

The next time you use the robot, select some rotation speed with the throttle, wait for it to stabilize (should be instantaneous), then use **Channel 5** and **Channel 6** to stop the display from rotating.
Then put the 3-position swiitch in the middle position. once the display is stable, move the switch to the bottom position, then back to middle.
Increase the speed, adjust the dials to stabilize the display, and toggle the 3-position switch again to store the next position. Repeat for lots of speeds.

Once you have enough samples, change the 3-position switch to the top position. now that you have multiple piecewise mapping functions linking data from centrifugal force to RPM, this will keep the display locked at different speeds.

You can dump the current curve data by:
- plugging a USB cable into the computer and teensy.
- open the serial monitor
- type 'p' and press enter. This should dump all captured input mappings.
 - the dumped look up table maps the current centrifugal acceleration to a predicted RPM. The code interpolates between forces.

The reason for this is: after calibrating the mappings you won't need to adjust the "virtual radius" dial manually ever again.

------

# Robot Tuning Instructions

## Power Up and Set Up
- Ensure your transmitter is configured correctly:
  - **Channel 5** is mapped to a potentiometer (dial).
  - **Channel 6** is mapped to another potentiometer (dial).
  - **Channel 7** is mapped to the inner 2-position switch. If this switch is down, the numeric display rotates the other way.
  - **Channel 8** is mapped to a 3-position switch.
    - top is default operation.
      - with no saved config, use **Channel 5** and **Channel 6** to calculate current offset.
      - with saved config, use that config.
    - middle is programming mode.
    - toggling to the botton position and back to the middle saves the current force/RPM value in the config.
      - the more and the more evenly (randomly) spaced, the better.
- Make sure your transmitter is turned off and not within easy reach.
- Make sure the robot is powered on.
  - you should see a moving red, green, and blue pattern. This means it's looking for a radio signal.
- turn on the transmitter. You should now see a flashing pattern of mostly green and blue.
  - make sure the throttle stays very low until the bot is in the test box. (you can carry the whole bot by the sticky-out-bits of the weapon.)

## Start Spinning
- Use **Channel 3** (left stick) to control the spinning speed:
  - Push the stick up slightly from the bottom to start the spin. Gradually increase until you only see blue LEDs near the edges of the robot.
  - If green LEDs are visible, the robot is not spinning fast enough. Continue increasing the throttle until only blue LEDs are visible.

## Fine-Tuning LED Color
- If the robot is spinning fast but you still see green LEDs, adjust **Channel 5** (mapped to a dial):
  - Turn the dial up slowly until only blue LEDs are visible.
  - If you can't eliminate the green LEDs, try turning **Channel 5** all the way up to trigger the blue LEDs earlier, then adjust the throttle to fine-tune the speed.
- If you only see blue LEDs but they are spinning relative to the robot, adjust **Channel 5**. Turning the dial one way should speed them up, and the other will slow them down.

## Check the Blue Arc
- When the robot is spinning fast enough (only blue LEDs visible), observe the blue arc:
  - If the arc is stationary, you are at least partially tuned.
  - The blue arc indicates the **front of the robot**.

## Movement Test
- Push the **right stick (Channel 1)** forward:
  - The robot should move in the direction of the blue arc (forward).
  - If the robot moves correctly, you are tuned. You should also be able to move in other directions using the right stick.
  - Experiment to find the optimal spin speed for smooth translation.

## Adjust for Incorrect Movement
- If the robot moves in the wrong direction:
  - Use **Channel 6** (the other dial) to rotate the blue arc:
    - Adjust the dial to rotate the blue indicator (the front of the robot) to point in the correct direction. The input ranges from **-180° to 180°**.
    - If you need to rotate further, turn the dial all the way around to reset the direction and complete one full revolution.

## Tuning Recap
- **Channel 5** adjusts the distance between the accelerometer and the center of rotation (1mm to 100mm).
- **Channel 6** adjusts the orientation of the blue arc to align the robot’s front with its direction of movement.

## Final Adjustments
- Continue tweaking **Channel 5** for the optimal LED response and **Channel 6** for alignment, as necessary.
- Once properly tuned, the robot should move smoothly and consistently in the direction of the blue arc.



TODO: images would show some ideas better

------
# Transcript of the coding session
Here is the transcript of me interacting with ChatGPT that resulted in the first version
of the code:

  https://chatgpt.com/share/67132632-9b8c-8000-8483-046ebea0f9ee

It wasn't just me saying "write me the code for a meltybrain combat robot." 

This transcript might help you understanding the code. You could just search for the 
function name, for example. Some code has been modified from what the bot wrote, and
I've done some clean up. 

It took on the order of 10 hours to write, which is much faster than it would've taken
me to get to this point. I got frustrated several times with the AI for not being able
to do anything right. 

This is also the 2nd attempt at doing it, and it's only as good (ha!) as it is because
I learned so much from how badly it did with the first attempt.
