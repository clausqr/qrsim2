# Quadrotor Simulator using Matlab/Simulink and Flightgear

The aim of this simulator is to  
1. provide a framework for control and dynamics simulation,   
2. focus on the rigid body attitude control problem,  
3. provide a nice way to visualize what's going on.

I used it on my [thesis](http://psic.fi.uba.ar/Publicaciones/Tesis/Rosito/Rosito.pdf) to test a global, robust, non-linear controller based on Lyapunov functions. 

[![ScreenShot](https://github.com/clausqr/qrsim2/raw/master/clipWUM7wxO1uRY.png)](https://www.youtube.com/watch?v=WUM7wxO1uRY)

##Blocks
The modular approach allows for customization of the different blocks:
![ScreenShot](https://github.com/clausqr/qrsim2/raw/master/qrsim2-block-diagram.png)

a. Control Reference Source Block
b. Joystick Input
c. Rigid Body Dynamics (UAV model)
d. Navigation/Measurement Block
e. Visualization Block 

##Visualization with Flightgear

The last block allows for Flightgear visualization, see [Flightgear-QUADLSE-Model](https://github.com/clausqr/Flightgear-QUADLSE-Model) for a nice UAV model to use with Flightgear

##Structure

Almost all functionality is implemented in separate .m files for easy tweaking.

## Launching

1. Load with simulink  
```bash
simulink modelo2a.mdl  
```  
2. Start Flighgear with arguments to listen for network connections.  
```bash
./fgfs --fdm=network,localhost,5501,5502,5503 --fog-fastest --disable-clouds --in-air --enable-freeze --aircraft=QUADLSE  
```  
3. Fly  
