---
title: Writing a simple publisher and subscriber (Python)
description: Mirror of the official ROS 2 Humble tutorial, reproduced under CC BY 4.0.
---

!!! info "Mirrored from the official ROS 2 documentation"
    This page reproduces an official ROS 2 Humble tutorial word for word so you
    can read it alongside the lab. It is **reference material, not the
    assignment** — the steps to follow for credit are on the
    [Week 3 lab page](../labs/week-03.md).

    Source: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)  
    Retrieved 15 September 2026. Copyright Open Robotics, licensed under
    [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
    Commands are shown with a `$` prompt, as upstream — do not type the `$`.

# Writing a simple publisher and subscriber (Python)

**Goal:** Create and run a publisher and subscriber node using Python.

**Tutorial level:** Beginner

**Time:** 20 minutes

## Background

In this tutorial, you will create [nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) that pass information in the form of string messages to each other over a [topic](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html).
The example used here is a simple "talker" and "listener" system;
one node publishes data and the other subscribes to the topic so it can receive that data.

The code used in these examples can be found [here](https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclpy/topics).

## Prerequisites

In previous tutorials, you learned how to [create a workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and [create a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).

A basic understanding of Python is recommended, but not entirely necessary.

## Tasks

### 1 Create a package

Open a new terminal and [source your ROS 2 installation](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html) so that `ros2` commands will work.

Navigate into the `ros2_ws` directory created in a previous tutorial.

Recall that packages should be created in the `src` directory, not the root of the workspace.
So, navigate into `ros2_ws/src`, and run the package creation command:

```bash
$ ros2 pkg create --build-type ament_python --license Apache-2.0 py_pubsub
```

Your terminal will return a message verifying the creation of your package `py_pubsub` and all its necessary files and folders.

### 2 Write the publisher node

Navigate into `ros2_ws/src/py_pubsub/py_pubsub`.
Recall that this directory is a [Python package](https://docs.python.org/3/tutorial/modules.html#packages) with the same name as the ROS 2 package it's nested in.

Download the example talker code by entering the following command:

=== "Linux"

    ```bash
    $ wget https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
    ```

=== "macOS"

    ```bash
    $ wget https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py
    ```

=== "Windows"

    In a Windows command line prompt:

    ```bash
    $ curl -sk https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py -o publisher_member_function.py
    ```

    Or in powershell:

    ```bash
    $ curl https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_publisher/examples_rclpy_minimal_publisher/publisher_member_function.py -o publisher_member_function.py
    ```

Now there will be a new file named `publisher_member_function.py` adjacent to `__init__.py`.

Open the file using your preferred text editor.

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 2.1 Examine the code

The first lines of code after the comments import [rclpy](https://docs.ros.org/en/humble/p/rclpy/) so its [Node](https://docs.ros.org/en/humble/p/rclpy/api/node.html) class can be used.

```python
import rclpy
from rclpy.node import Node
```

The next statement imports the built-in [std_msgs/msg/String](https://docs.ros.org/en/humble/p/std_msgs/msg/String.html) message type that the node uses to structure the data that it passes on the topic.

```python
from std_msgs.msg import String
```

These lines represent the node's dependencies.
Recall that dependencies have to be added to `package.xml`, which you'll do in the next section.

Next, the `MinimalPublisher` class is created, which inherits from (or is a subclass of) [Node](https://docs.ros.org/en/humble/p/rclpy/api/node.html).

```python
class MinimalPublisher(Node):
```

Following is the definition of the class's constructor.
`super().__init__` calls the [Node](https://docs.ros.org/en/humble/p/rclpy/api/node.html) class's constructor and gives it your node name, in this case `minimal_publisher`.

[create_publisher](https://docs.ros.org/en/humble/p/rclpy/api/node.html#rclpy.node.Node.create_publisher) declares that the node publishes messages of type [std_msgs/msg/String](https://docs.ros.org/en/humble/p/std_msgs/msg/String.html) (imported from the `std_msgs.msg` module), over a topic named `topic`, and that the "queue size" is 10.
Queue size is a required [Quality of Service](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Quality-of-Service-Settings.html) (QoS) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.

Next, [create_timer](https://docs.ros.org/en/humble/p/rclpy/api/node.html#rclpy.node.Node.create_timer) is used to create a callback that executes every 0.5 seconds.
`self.i` is a counter used in the callback.

```python
def __init__(self):
    super().__init__('minimal_publisher')
    self.publisher_ = self.create_publisher(String, 'topic', 10)
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.i = 0
```

`timer_callback` creates a message with the counter value appended, publishes it, and prints it to the console with [get_logger()](https://docs.ros.org/en/humble/p/rclpy/api/node.html#rclpy.node.Node.get_logger)'s [info()](https://docs.ros.org/en/humble/p/rclpy/rclpy.impl.rcutils_logger.html#rclpy.impl.rcutils_logger.RcutilsLogger.info) function.

```python
def timer_callback(self):
    msg = String()
    msg.data = 'Hello World: %d' % self.i
    self.publisher_.publish(msg)
    self.get_logger().info('Publishing: "%s"' % msg.data)
    self.i += 1
```

Lastly, the main function is defined.

```python
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()
```

First the [rclpy](https://docs.ros.org/en/humble/p/rclpy/) library is initialized, then the node is created, and then it "spins" the node (using [spin()](https://docs.ros.org/en/humble/p/rclpy/api/init_shutdown.html#rclpy.spin)) so its callbacks are called.

#### 2.2 Add dependencies

Navigate one level back to the `ros2_ws/src/py_pubsub` directory, where the `setup.py`, `setup.cfg`, and `package.xml` files have been created for you.

Open `package.xml` with your text editor.

As mentioned in the [previous tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), make sure to fill in the `<description>`, `<maintainer>` and `<license>` tags:

```xml
<description>Examples of minimal publisher/subscriber using rclpy</description>
<maintainer email="you@email.com">Your Name</maintainer>
<license>Apache License 2.0</license>
```

After the lines above, add the following dependencies corresponding to your node's import statements:

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

This declares the package needs [rclpy](https://docs.ros.org/en/humble/p/rclpy/) and  [std_msgs](https://docs.ros.org/en/humble/p/std_msgs/) when its code is executed.

Make sure to save the file.

#### 2.3 Add an entry point

Open the `setup.py` file.
Again, match the `maintainer`, `maintainer_email`, `description` and `license` fields to your `package.xml`:

```python
maintainer='YourName',
maintainer_email='you@email.com',
description='Examples of minimal publisher/subscriber using rclpy',
license='Apache License 2.0',
```

Add the following line within the `console_scripts` brackets of the [entry_points](https://setuptools.pypa.io/en/latest/userguide/entry_point.html) field:

```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
        ],
},
```

Don't forget to save.

#### 2.4 Check setup.cfg

The contents of the `setup.cfg` file should be correctly populated automatically, like so:

```text
[develop]
script_dir=$base/lib/py_pubsub
[install]
install_scripts=$base/lib/py_pubsub
```

This is simply telling [setuptools](https://setuptools.pypa.io/en/latest/userguide) to put your executables in `lib`, because `ros2 run` will look for them there.

You could build your package now, source the local setup files, and run it, but let's create the subscriber node first so you can see the full system at work.

### 3 Write the subscriber node

Return to `ros2_ws/src/py_pubsub/py_pubsub` to create the next node.
Enter the following code in your terminal:

=== "Linux"

    ```bash
    $ wget https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
    ```

=== "macOS"

    ```bash
    $ wget https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py
    ```

=== "Windows"

    In a Windows command line prompt:

    ```bash
    $ curl -sk https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py -o subscriber_member_function.py
    ```

    Or in powershell:

    ```bash
    $ curl https://raw.githubusercontent.com/ros2/examples/{REPOS_FILE_BRANCH}/rclpy/topics/minimal_subscriber/examples_rclpy_minimal_subscriber/subscriber_member_function.py -o subscriber_member_function.py
    ```

Now the directory should have these files:

```bash
__init__.py  publisher_member_function.py  subscriber_member_function.py
```

#### 3.1 Examine the code

Open the `subscriber_member_function.py` with your text editor.

```python
import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

The subscriber node's code is nearly identical to the publisher's.
The constructor creates a subscriber with the same arguments as the publisher using [create_subscription](https://docs.ros.org/en/humble/p/rclpy/api/node.html#rclpy.node.Node.create_subscription).
Recall from the [topics tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.

```python
self.subscription = self.create_subscription(
    String,
    'topic',
    self.listener_callback,
    10)
```

The subscriber's constructor and callback don't include any timer definition, because it doesn't need one.
Its callback gets called as soon as it receives a message.

The callback definition simply prints an info message to the console, along with the data it received.
Recall that the publisher defines `msg.data = 'Hello World: %d' % self.i`

```python
def listener_callback(self, msg):
    self.get_logger().info('I heard: "%s"' % msg.data)
```

The `main` definition is almost exactly the same, replacing the creation and spinning of the publisher with the subscriber.

```python
minimal_subscriber = MinimalSubscriber()

rclpy.spin(minimal_subscriber)
```

Since this node has the same dependencies as the publisher, there's nothing new to add to `package.xml`.
The `setup.cfg` file can also remain untouched.

#### 3.2 Add an entry point

Reopen `setup.py` and add the entry point for the subscriber node below the publisher's entry point.
The [entry_points](https://setuptools.pypa.io/en/latest/userguide/entry_point.html) field should now look like this:

```python
entry_points={
        'console_scripts': [
                'talker = py_pubsub.publisher_member_function:main',
                'listener = py_pubsub.subscriber_member_function:main',
        ],
},
```

Make sure to save the file, and then your pub/sub system should be ready.

### 4 Build and run

You likely already have the [rclpy](https://docs.ros.org/en/humble/p/rclpy/) and [std_msgs](https://docs.ros.org/en/humble/p/std_msgs/) packages installed as part of your ROS 2 system.
It's good practice to run [rosdep](https://docs.ros.org/en/independent/api/rosdep/html/) (check the [rosdep tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html)) in the root of your workspace (`ros2_ws`) to check for missing dependencies before building:

=== "Linux"

    ```bash
    $ rosdep install -i --from-path src --rosdistro humble -y
    ```

=== "macOS"

    rosdep only runs on Linux, so you can skip ahead to next step.

=== "Windows"

    rosdep only runs on Linux, so you can skip ahead to next step.

Still in the root of your workspace, `ros2_ws`, build your new package:

=== "Linux"

    ```bash
    $ colcon build --packages-select py_pubsub
    ```

=== "macOS"

    ```bash
    $ colcon build --packages-select py_pubsub
    ```

=== "Windows"

    ```bash
    $ colcon build --merge-install --packages-select py_pubsub
    ```

Open a new terminal, navigate to `ros2_ws`, and source the setup files:

=== "Linux"

    ```bash
    $ source install/setup.bash
    ```

=== "macOS"

    ```bash
    $ . install/setup.bash
    ```

=== "Windows"

    ```bash
    $ call install/setup.bat
    ```

Now run the talker node.
The terminal should start publishing info messages every 0.5 seconds, like so:

```bash
$ ros2 run py_pubsub talker
[info] [minimal_publisher]: publishing: "hello world: 0"
[info] [minimal_publisher]: publishing: "hello world: 1"
[info] [minimal_publisher]: publishing: "hello world: 2"
[info] [minimal_publisher]: publishing: "hello world: 3"
[info] [minimal_publisher]: publishing: "hello world: 4"
...
```

Open another terminal, source the setup files from inside `ros2_ws` again, and then start the listener node.
The listener will start printing messages to the console, starting at whatever message count the publisher is on at that time, like so:

```bash
$ ros2 run py_pubsub listener
[INFO] [minimal_subscriber]: I heard: "Hello World: 10"
[INFO] [minimal_subscriber]: I heard: "Hello World: 11"
[INFO] [minimal_subscriber]: I heard: "Hello World: 12"
[INFO] [minimal_subscriber]: I heard: "Hello World: 13"
[INFO] [minimal_subscriber]: I heard: "Hello World: 14"
```

Enter `Ctrl+C` in each terminal to stop the nodes from spinning.

## Summary

You created two nodes to publish and subscribe to data over a topic.
Before running them, you added their dependencies and entry points to the package configuration files.

## Next steps

Next you'll create another simple ROS 2 package using the service/client model.
Again, you can choose to write it in either [C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html) or [Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.html).

## Related content

There are several ways you could write a publisher and subscriber in Python; check out the `minimal_publisher` and `minimal_subscriber` packages in the [ros2/examples](https://github.com/ros2/examples/tree/{REPOS_FILE_BRANCH}/rclpy/topics) repo.
