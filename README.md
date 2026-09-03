# Rocket 
here comes some stupid data
the diagram you see, its on matlab simulaink.
Rocket simulation

openrocket is a place where people design solid fuel rockets. openrocket is a really powerfull simualtion tool out there to actually make a successfull rocket, but it comes up with its issues, what is that? well solid rockets cannot be controlled by TVC aka thrust vector control. the idea is simple for TVC, you add servos on the rocket engine and tilt it to control the thrust vector. Problem with the solid fuel rocket is that you just want to control the nozzel only but wow there is no nozzel in sold fuel rockets, to tilt it we need to turn the entire booster. this probllem is solved by liquid fuel, but to burn it you need oxidisers, that means extra weight and less thrust. solid fuel rockets are dominant in take offs to exit ionoshpere. rocket enginner use hybrid rockets, solid fuel boosters and liquid fuel for further stages. but we are not going that far. OUr job is really simple compare to all that complex shit out there. The rocket has solid fuel motors, but to control where it will go depends completly on fins now. How do they work, well fins exists cuz they produce drag which tilts the rocket back to its original place and keep it steady throughout the flight. here is the magic now. the oscilation observed is simple damping motion, well if the fin angle is chaged there respective center of oscilation or aka angle of equilibrium will change, meaning the angle of attack (angle of velocity of rocket with its longitudnal axis) can be changed, also meaning rocket will tilt in that direction. Wollaha. you got yourself a rocket. Our job jsut starts at this point, rocket will need to navigate to the targeted position which means how will rocket know its current position? how much rocket shoulldd turn to adjust itself in the correct direction? well this is where Flight Computer comes into the picture. a computer which will calculate its trajectory and decide where it will go next.
Plan
Phase 1:

first we will achive a simple 3DOF moddel for rocket with variable mass an thrust, the data for thrust is imported from the standard solid fuel rocket motor and imported in matlab in csv format. the current rocket is small, it has abality to go roughly 300m, in refrence its height of a 20 story building. we need to match simulation in simulink to be as real as possible, this will include atmospheric parameters, variable mass, and changing gravity with height.
phase 2:

idk man, i havent even achivied phase 1
<img src="./artifacts/rocket_simulink_diag.png">
<img src="./artifacts/rocket_flight.gif">

<img src="./artifacts/PCB_Sch.png">
<img src="./artifacts/PCB_3D.png">
