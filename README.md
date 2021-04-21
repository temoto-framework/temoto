# temoto
Setup package for TeMoto

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

**More information about Temoto in https://temoto-telerobotics.github.io**

## Quick Overview

### What is TeMoto?
TeMoto is a ROS-based software framework for building dependable robotic applications for facilitating human-robot collaboration and autonomy.

TeMoto provides a set of software tools that help solving common challenges in applications covering system reliability, human-robot collaboration and multi-robot systems.

***And just like any other toolbox, it's up to the user of the toolbox to decide what tool is right for the application.***

### Key features of TeMoto

* **Dynamic resource management**: Existing components, or resources such as a camera, lidar, network, CPU intensive algorithm, etc.  can be programmatically monitored, started, stopped, and shared.
    
* **Task management**: TeMoto separates mission strategy related code from functional resources, such as sensors and actuators. A task consists of modular actions that implement a specific behaviour, e.g., a sensing or a navigation action. Actions can contain any arbitrary user defined code and can dynamically start, stop and access resources. 
    
* **Minimal development overhead**: TeMoto does not require any resource customization, i.e., existing ROS packages can be used via TeMoto without modification.
    
* **Full system modularity**: While TeMoto contains a number of subsystems (ROS nodes), each subsystem has minimal dependencies. This allows the robotics community to adopt only the tools that matter for their project and keeps code bloat at a minimum.

### Use-case Example
TeMoto framework allows dynamic use of a system's hardware and software resources enabling Long-Term Autonomy (LTA), redundancy and energy efficiency. For example, a planetary rover equipped with a number of sensors and actuators could be deployed in a remote site with a variety of mission(s) that last for long periods of time. Unexpected sensor failures have to be resolved by substituting the failed resource with similar or combined resources. Also, the goals of the mission can change, which requires a modular way to separate mission control logic from the resource management, thus maximizing code reuse and therefore development efficiency. 

As mentioned earlier, not every feature or tool of TeMoto is required for deployment. The developer can choose the tools that matter the most and discard the rest.

<img src="docs/figures/enabling_features.png" alt="Architecture of TeMoto" class="center" width="500"/>

## In-Depth Overview

### Core Concepts and the Architecture

The TeMoto framework stems from three distinct concepts:

* **Action** - A system developer defined and dynamically executable code module that can embed custom logic, such as mission execution strategy.
    
* **Resource** - Generically a *resource* is something allocated or deallocated upon a request, e.g., a camera. A *resource* is not limited to hardware devices, it could also describe an algorithm (image filter, object recognizer, etc) or a combination of *sub-resources*. Resource types are resources described by the same set of parameters.
    
* **Resource Manager** - A subsystem responsible for serving *resource* requests. Different resources types can be handled by the same *resource manager*.

<img src="docs/figures/architecture.png" alt="Architecture of TeMoto" class="center" width="500"/>

### TeMoto Resource Managers


## Installation Instructions

``` bash
# Navigate to your catkin workspace
cd <catkin_workspace>/src

# Clone TeMoto and all it's dependencies
git clone --recursive https://github.com/temoto-telerobotics/temoto

# Build your workspace
catkin build
``` 