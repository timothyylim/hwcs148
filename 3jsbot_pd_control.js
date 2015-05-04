//////////////////////////////////////////////////
/////     MOTION CONTROL ROUTINES 
//////////////////////////////////////////////////

// CS148: add PD controller here
function robot_pd_control () {


    kp = 1;
    error = 0;

    var curdate = new Date();
    dt = curdate.getSeconds()/(60*2*Math.PI);
 
    for (x in robot.joints) {
    	error = robot.joints[x].desired - robot.joints[x].angle;
	    robot.joints[x].control = kp*error

	}

}

