//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () {

	//Initializing the variables for the PID Control

	//Note : Never used the kd and ki because the robot moved to the right location
	//without needing them. Still left the code in place.

    robot.servo = {kp:1, kd:0, ki:0};
    accumulated_error = 0;
    error = 0;
    last_error = 0;
    derivative_error = 0;

    //Setting the timestep using the current world time
    var curdate = new Date();
    dt = curdate.getSeconds()/60*2*Math.PI;
    dt = 0.0001;

    //Performing the PID Controls   
    for (x in robot.joints) {
        robot.joints[x].desired = curdate.getSeconds()/60*2*Math.PI;
        // console.log(robot.joints[x].desired);
    	error = robot.joints[x].desired - robot.joints[x].angle;
	    accumulated_error = accumulated_error + error*dt;
	    derivative_error = (error - last_error)/dt;
        // console.log(error);
        // console.log(derivative_error);
        // console.log(robot.joints[x].angle)
	    last_error = error;

	    robot.joints[x].control = (robot.servo.kp*error) + 
	    (robot.servo.ki*accumulated_error) + 
		(robot.servo.kd*derivative_error);

	}

}

