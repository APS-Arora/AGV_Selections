# AGV_Selections
The project is pre-built, no need to build. In case a build is required, build ./src
Note: Source the ./devel/setup.bash to be able to run the project on ROS
Note: Please run the project in ./res directory so the Nodes have access to the problem image

Package: "ps1"
--Node: "Publisher_Node" publishes on topic "/plot" and "/intercom"
--Node: "Subscriber_Node" subscribes to the same topics
