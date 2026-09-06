# COMP0225: Robotics and Artificial Intelligence Group Project
##  Efficient Powered Clothing: Twisted String Actuators and Machine Learning Approach in Sit-to-Stand Transition Repository


TSAmodel folder - all the xml files from Igors pHD student, needs to be copied and pasted into models folder on the myosuit github 
init.py - section of code needs to be copied and pased into myosuite/envs/myo/myobase/__init__.py in the github make sure its right at the end
base_TSA.py - copy the file into the myosuite/envs/myo/myobase folder this is just the class definition for registering their environment for now
based_code.py - what you run right now for the sim on mac run like - mjpython test_moving.py


## Part 2 STS:
- put the myotorso_exosuite into myosuite/simhive/myo_sim/torso and put the files in myotorso_exosuite/assets into  myosuite/simhive/myo_sim/torso/assets
- copy the new class into  myosuite/envs/myo/myobase/__init__.py like with the TSA class (make sure to change paths if they are wrong - they are definitely wrong)
- base_sts is for running the sit to stand motion in the end 