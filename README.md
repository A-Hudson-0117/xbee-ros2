# xbee-ros2

This ROS2 project provides functionality to send and receive XBee messages between ROS2 nodes and XBee devices.  It allows for direct communication with XBee networks, enabling ROS2 applications to interact with hardware peripherals and edge devices. The package supports both sending and receiving XBee messages, as well as broadcasting requests.


## Overview

The `xbee-ros2` package facilitates seamless integration between ROS2 and XBee networks. It leverages a custom message definition ([`XBeeSend.msg`](xbee_msgs/msg/XBeeSend.msg)) to encapsulate XBee data and destination addresses, allowing ROS2 nodes to easily transmit XBee commands and receive data from XBee devices.


## Features

*   **XBee Message Transmission:**  Send XBee messages to specific XBee devices or broadcast them to the network.
*   **XBee Message Reception:** Receive XBee messages from XBee devices and process them within ROS2.
*   **XBee Broadcast Requests:**  Send broadcast requests to the XBee network.
*   **Custom Message Definition:** Uses a custom ROS message ([`XBeeSend.msg`](xbee_msgs/msg/XBeeSend.msg)) for standardized XBee communication.


## Installation

1.  **Clone the repository:**
    ```bash
    git clone <repository_url>
    ```
2.  **Build the package:**
    ```bash
    cd xbee-ros2
    colcon build
    ```


## Usage
### contents
*   `xbee_msgs/`: Contains the custom message definition (`XBeeSend.msg`).
*   `xbee_ros/`: Contains the ROS node implementation (`xbee_node.py`).

### Run
1.  **Source the environment:**
    ```bash
    source install/setup.bash
    ```
2.  **Launch the XBee ROS node:**
    ```bash
    ros2 run xbee_ros xbee_node
    ```

### Sending XBee Messages
ROS nodes can publish messages to 2 specified topic defined within the [`xbee_node.py`](xbee_ros/src/xbee_node.py) file, forwarding it to the xbee network.

The default topics are: 
 * `/xbee/send`
 * `/xbee/broadcast`

### Receiving XBee Messages
The [`xbee_node.py`](xbee_ros/src/xbee_node.py) subscribes to ROS topics and processes incoming XBee messages, forwarding them to interested subscribers.

The default topics are: 
 * `/xbee/recv`

### Message Definition
The [`XBeeSend.msg`](xbee_msgs/msg/XBeeSend.msg) file defines the structure of XBee messages used for communication:
> ``` txt
> string destination # Optional 
> string data
> ```

This message includes fields for the destination XBee address and the data to be transmitted. Refer to [`xbee_msgs/msg/XBeeSend.msg`](xbee_msgs/msg/XBeeSend.msg) for complete details.


## Further Information

### Namespaces
The `xbee-ros2` package utilizes the `xbee_ros` namespace to prevent conflicts with other ROS packages.  This means all nodes and topics published by the package are prefixed with `xbee_ros`.


### Changing the Namespace via CLI
You can temporarily change the namespace for the XBee ROS node when running it from the command line using the `-r` & `__ns` args with `ros2 run`.

```bash
ros2 run xbee_ros xbee_node --ros-args -r __ns:=<new_namespace>
```

This will cause the node's name to be `<new_namespace>/xbee_node`. an example entry of `<new_namespace>` is `/test_name`. 


### Remapping Topics via CLI
Topic remapping allows you to change the default topic names used by the package. You can remap topics when running `ros2 run` using the `-r` flag alongside the `--ros-args` argument.  For example, to remap the `send` topic to `my_xbee_send`:

```bash
ros2 run xbee_ros xbee_node --ros-args -r xbee/send:=my_xbee_send
```

This will make the default `/xbee/send` topic be renamed to `/my_xbee_send`. The change applies only to the current process


### Setting ROS Parameters via CLI

The `xbee-ros2` package exposes configurable parameters that influence its behavior. These parameters can be dynamically modified using the `ros2 param set` command. For example, to change the `baud_rate` parameter:

```bash
ros2 run xbee_ros xbee_node --ros-args -p baud_rate:=9600 -p port:='COM1'
```

Refer to the `xbee_node.py` file for a complete list of available parameters and their descriptions.
