# UDP communication on OBU
  Code on another computer without ROS for udp communication
## Enviroment
  cmake minimum required VERSION 3.0.2
## Function
  Receive locational information including x ,y ,yaw through **my_recv_handler()** from bot.  
  Send upper level information including pedestrian, status for trafficlight through **sendData()** to bot.  
  Edit **main()** if you want to customize the information to send.  
