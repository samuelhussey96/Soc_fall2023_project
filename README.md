# Capacitive Touch Controller

This project implements the basic components of a capacitive touch screen. 

## Description

In this project, a proof of concept capacitive touch screen controller is implemented in which a software based DDFS method is used to transmit tones onto an array of conductors. A set of intersecting conductors form a grid and are connected to ADC inputs, this grid of wire forms the "touch screen". Each transmitting conductor couples capacitively to a given receiver, the resulting composite signal is then sampled and processed to extract information about the magnitude of individual tones. The frequency generation and sampling schemes are implemented such that each intersecting node corresponds to a transmitter/receiver pair where the coupled signal level between the two can be mapped to a pixel for display or other application. When another conductive material, such as a finger, comes into close enough proximity with a node, a portion of the transmitted signal couples to that object instead of the receiving conductor resulting in a voltage drop. Coordination of these signal changes forms the basis of touch interface.
