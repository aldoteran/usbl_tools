# USBL tools package
This repo is a compilation of useful scripts and tools to interface, communicate,
and visualize data provided by the Evologics SR17 USBL and acoustic communication
modem. Specifically used for research and development using the AUV LoLo from
the Maritime Robotics Laboratory at the KTH Royal Institute of Technology as
part of the Swedish Maritime Robotics Centre (SMaRC) project.

## Dependencies
 - My fork of the [dmac ros driver]()
 - The [sbg_driver](https://github.com/SBG-Systems/sbg_ros_driver.git)

## Nodes

### usbl_correction_relay
This node will take the measured USBLLONG's range and the lat/lon from the SBG AHRS and relay it
to LoLo. Not much to do but `rosrun usbl_tools usbl_correction_relay.py` and Lolo will know what
to do with the message.

### usbl_fix_relay
This node is specifically used for the prox-ops experiments. We relay the corrected USBLLONG position fix
to LoLo in order to track the service boat as a target. Just `rosrun usbl_tool usbl_fix_relay.py` to run.

### bash aliases


