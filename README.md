<h1> vehicle-control </h1>

<h2> Description </h2>

<p> Path Following Control for an Autonomous Golf-Cart. This is a repository containing code used in our capstone project. Objectives of this project are: <ol>
    <li>Estimating the states of the vehicles, i.e. position, orientation, & orientation in 2D plane, using GNSS, Accelerometer, Gyroscope, and EKF algorithm,</li>
    <li>Generating the path used in the experiment by using a pre-taken position (x,y) from GNSS,</li>
    <li>Enabling the golf cart to follow a given path and speed automatically.</li>
</ol> 
</p>

<p>You can find the controller and EKF class in src/Python folder. You can also find the Arduino code in the src folder. You can see custom messages in the msg folder. The codes executed in the previous experiment are located in the nodes folder. You can change the parameter of that code (inside the nodes folder) in the corresponding config file.</p>

<h2> Learning Resources </h2>
<p>To prevent conflicts that may occur, we suggest you use the ROS Melodic. You can learn some basics of ROS used in this experiment here:
    <ol>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment">Installing and Configuring Your ROS Environment</a></li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/NavigatingTheFilesystem">Navigating the ROS Filesystem</a></li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/CreatingPackage">Creating a ROS Package</a> </li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/BuildingPackages">Building a ROS Package</a> </li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/UnderstandingNodes">Understanding ROS Nodes</a> </li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics">Understanding ROS Topics</a> </li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams">Understanding ROS Services and Parameters</a> </li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch">Using rqt_console and roslaunch</a> </li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv">Creating a ROS msg and srv</a>. I didn't use the ROS srv. But, feel free to explore.</li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29">Writing a Simple Publisher and Subscriber (C++)</a>. Read this if you use the C++ language.</li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29">Writing a Simple Publisher and Subscriber (Python)</a>. Read this if you use the Python language.</li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber">Examining the Simple Publisher and Subscriber</a></li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/DefiningCustomMessages">Defining Custom Messages</a> </li>
        <li> <a href="http://wiki.ros.org/ROS/Tutorials/Recording%20and%20playing%20back%20data">Recording and playing back data</a> </li>
    </ol>
</p>

<p>You can learn the basic of ROS-Arduino from <a href="http://wiki.ros.org/rosserial_arduino/Tutorials">here</a>.</p>
