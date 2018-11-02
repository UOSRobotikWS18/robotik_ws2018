# radio_ros -- Example Ros Package (Python) 

## Radio Example of Ros Nodes, Subscriber, Publisher, Topics

![ex_radio](https://raw.githubusercontent.com/aock/radio_ros/master/docs/RadioRos.png)

<ol>
  <li> Clone this Repository to "your_workspace/src/"</li>
  <li> Run catkin_make </li>
  <li> Run roscore </li>
  <li> Start Radio with:
    <ul> 
      <li> rosrun radio_ros dj.py </li>
      <li> rosrun radio_ros traffic_info.py </li>
      <li> rosrun radio_ros weatherman.py </li>
    </ul>
  </li>
  <li> Start Users with (You can start more than one user in different terminals/sessions):
    <ul> 
      <li> rosrun radio_ros farmer.py </li>
      <li> rosrun radio_ros trucker.py </li>
    </ul>
  </li>
</ol>

To list the existing Topics you can use: 
```bash
rostopic list
```

To show the messages published on a specific Topic e.g. music, run:
```bash
rostopic echo music
```


## Realworld Robot Example

![ex_real](https://raw.githubusercontent.com/aock/radio_ros/master/docs/RadioRos2.png)
