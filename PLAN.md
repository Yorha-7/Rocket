# Tha Plan
the workspace is matlab workspace for simulatiung the rocket.

Our goal is to achive a full pledegd working simualtion for the rocket.

## what i know about rocket fllight mechanism

till far one thing in know is that aerospace guys use ned for gorund refrence fram aka launch site, and then rocket frame in which z axis is top and also yaw,
y for pitch and x for roll. how is it allign with the base, well initially, rocket z will point apposite to launch site z. rocket x will point to local north and y is west.
so for launch site we have ned (north east down) wrt to local nort, and for the rocket we have nwu (north west up). on this two refrence frames rocket motion will be observed.

## How is the math working

as we now know what are the two frame we are dealing with, now we know the forces too, wrt to launch sit coordinates (ned frame) we have gravity as positive z axis, and yhea
tahts preety all. you see we are not calcullating force in launch site frame, that frame is just to find trajectory, forces will act in the rockets frame.
the onlyy force which we know and which is predictable is the thrust vector, for sollid rocket it has no gimble (it cant be TVC or thrsut vector controlleld). so thrust 
always acts on the z axis, now gravity will try to pulll rocket down, assuming transform of the rocket is the rockets center of mass itself, the gravity vector will be
appllied depending on the rockets orinetation wrt to the launch site coordinates, si we willl lhave gravity responsiblle too, and for the ai drag forces, we have formula 
of dynamic pressure given by NASA scientists, 0.5 * rho * v^2 * constant * now this constant is the deciding factor. originally there are two constant in 3dof for this shit,
drag coefficient and normal coefficient, as they suggest they are not constant in real life, there are methods to calculate these constants but we are not going in that much depth for a protoype rig for now, we will use fact that drag and normal forces are applied on the rocket too, offcourse in the frame of the rocket itslef, since we are 
taking the coefficients as the constant values, the only variablle is the wind speed, since we are taking wind spped to be 0 for starters too, the relative veocity of 
rocket to that of the wind willl also be just vellocity of ricket. 

## Phase 1

first what method are we going to use. welll first we willlll need some data preprocessing, the data on thrust we collectedd from solid rocket engine manufactture is not 
continuos in time also timesteps are uneven, so we willl first begin with cleaning the data to get evenlly distrubuted time steps. in short we will make tim axis even
by findding the missing steps bay averaging the data between the two nearest point mulltiplllied by how much that point is near to that one. in maths it willll 
looks like ( T1 * (t - t1) + T2 * (t2 - t) )/2 * (t2 - t1) where you guessed right, T1 is lleft side data and t2 is right side data and t is the missing time
stamp that folows n*dell(t) time step for our simulation, this way onlly that data will remain whic falls on our desiredd time step time step.

programatically it looks like function thrust_data_cleaning() will be a function which will do this job, it will need input as the data itslef whic is a nx2 dimenision vector,
with n rows and 2 colloumns, you guessed it, one is time and other is thrust in newtons.

we fininshed our first supllementry file jsut like that. 
now the ddescisions to be made is about choosing correct data type for variablles and the vector structure and storage in memory cuz we need fast acces to our data when in need.


## Phase 2 

the next cpp file is about the kinemeatics, we will have a class for kinematics, inside that we will have methods. this is our primary callculation engine, though this is 
not automatic for sure. the job is that this cass methds willll solve some of the basic mechanic equation which are needed to find rocket accll. to calculate rocket 
position we onlly need to know what are the forces are on the center of mass, that forces willl give accl vector direction and magnitude. that frame willll lbe resoved in 
the launch site frame and the point mass next position will be calculated. to run alll those calculation this cllass wil have methods which will run equations like
drag force equation, gravity variation due to rocket altitude, pressure variation, thrust force cacullation and so on, right now since the projectinle is just a 2d motion with 
3dof we dont have much to worry about ,but things will get compicated quicklly in the 6 dof so for now lllest stick to what we are doing. 

## phase 3

our next importable library after our own kinematics cllass willl be for rocket trajectory, now that kinematics library made by us exist, we willl use allll that 
power of solving into finding ocation of rocket for each time stamp. 
we will need a config file too att this point, thsi config file will have something like height of the rocket center of mass for the aunch pad which is absoulte ground. 
initail tillte angle, and stuff like that. using this configuration, rocket calcuations wil lbe made and its position will be estimated.

## phase 4

testing our libraries, for verification we wil use matplotib to visualize data and graphs, this graphs willl linclude height to tiime in z and x axis sepratley, same for
accellaration and same goes for the velocity as well as ange of tilt, we dont know much params but i wil recmommed to use the aerospace guidance to see which params to observe 
in the graphs which willl make debug truly easy.

 
