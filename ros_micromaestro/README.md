This node drives the MicroMaestro board.  It takes steering/throttle commands from the Vehicle Model  nodes, and converts them to pulses to send to the MicroMaestro.  

The MicroMaestro/USB driver commands are from the following GitHub repository:  https://github.com/jbitoniau/rapapololumaestro  
Currently the source/header files are just copied directly from the repository - will eventually add as a sub-repo on BitBucket

IMPORTANT:  Need CMake 3.0 or higher to compile the driver commands.

A symlink rule has been added to the project in the folder symlink-mm.  Run the install.sh file to add it and the reload.sh file to reload the device rules.  Both need sudo    

Future updates to this node will include the following:  
-Clamping functions to protect the Servos/ESCs  
