# Quadrotor Simulator using Matlab/Simulink and Flightgear

The aim of this simulator is to  
1. provide a framework for control and dynamics simulation,   
2. focus on the rigid body attitude control problem,  
3. provide a nice way to visualize what's going on.  

I used it on my thesis to test a global, robust, non-linear controller based on Lyapunov functions. This controller can to track any arbitrary desired orientation of the Quadrotor within a specified error bound, and the results provide a way to translate bounded measurement and navigation errors to final pointing accuracy errors, and also to obtain a bound on the tracking accuracy given the timestep used for the digital implementation of the continuous controller. If you want to find out more please have a look at my [thesis](http://psic.fi.uba.ar/Publicaciones/Tesis/Rosito/Rosito.pdf)!

[![ScreenShot](https://github.com/clausqr/qrsim2/raw/master/clipWUM7wxO1uRY.png)](https://www.youtube.com/watch?v=WUM7wxO1uRY)

##Blocks
The modular approach allows for customization of the different blocks:
![ScreenShot](https://github.com/clausqr/qrsim2/raw/master/qrsim2-block-diagram.png)

a. Command Generation Block / Joystick Input  
b. Controller Block  
c. Rigid Body Dynamics (UAV model) Block  
d. Navigation/Measurement Block  
e. Visualization Block  

Each block is implemented inside a standalone .m file that gets called from within simulink as a level-2 s-file. The name of the .m files is self explanatory (Quadrotor_Navigation.m, Quadrotor_Controller.m, etc.).  

Global initialization is done with a callback hooked up on ModelInit, implemented through modelo2Initialization.m

##Visualization with Flightgear

The last block allows for Flightgear visualization, see [Flightgear-QUADLSE-Model](https://github.com/clausqr/Flightgear-QUADLSE-Model) for a nice UAV model to use with Flightgear. Otherwise it's straightforward. 

##Structure

Almost all functionality is implemented in separate .m files for easy tweaking.

## Launching

1. Load with simulink:  
```bash
load_model('modelo2a.slx')
```
...  
2. Start Flightgear with arguments to listen for network connections:  
```bash
./fgfs --fdm=network,localhost,5501,5502,5503 --fog-fastest --disable-clouds --in-air --enable-freeze --aircraft=QUADLSE  
```
...  
3. Fly  
4. Tweak  
5. goto 3  
