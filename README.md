# rostrace

Launches a ROS system using a provided `.launch` file and generates a Daikon trace
file, recording information regarding the system architecture, messages published
on topics, service calls, and parameter values.
Based on the approach described by Hengle Jiang in his Masters thesis entitled
*Invariant Inferring and Monitoring in Robotic Systems*.

* does not require source code instrumentation
* messages published on topics are recorded using the rosbag utility.
* records service calls by rerouting all services through a recording node, which
  publishes details of the service call to the `service_calls` topic before
  forwarding the request onto the intended node.
