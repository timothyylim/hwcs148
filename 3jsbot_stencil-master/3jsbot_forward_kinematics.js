////////////////////////////////////////////////
/////     FORWARD KINEMATICS
//////////////////////////////////////////////////

// CS148: compute and draw robot kinematics (.xform matrix for each link)
// CS148: compute and draw robot heading and lateral vectors for base movement in plane
// matrix_2Darray_to_threejs converts a 2D JavaScript array to a threejs matrix
//   for example: var tempmat = matrix_2Darray_to_threejs(link.xform);
// simpleApplyMatrix transforms a threejs object by a matrix
//   for example: simpleApplyMatrix(link.geom,tempmat);

/*
CS148: reference code has functions for:

robot_forward_kinematics
traverse_forward_kinematics_link
traverse_forward_kinematics_joint
compute_and_draw_heading
*/

 function robot_forward_kinematics () {
 	
 	traverse_link(robot.base, generate_identity());
}

function transform_joint(joint,mstack) {
	//Tranforms the joint, given the origin is contained in mstack
	var x = robot.joints[joint].origin.xyz[0];
    var y = robot.joints[joint].origin.xyz[1];
    var z = robot.joints[joint].origin.xyz[2];
    var r = robot.joints[joint].origin.rpy[0];
    var p = robot.joints[joint].origin.rpy[1];
    var rot_y = robot.joints[joint].origin.rpy[2];

    var result = generate_matrix_transform(x,y,z,r,p,rot_y);
    result = matrix_multiply(mstack,result);

    //Quaternion Control
    var a = generate_quaternion_matrix(robot.joints[joint]);
    var result = matrix_multiply(result,a);
    var tempmat = matrix_2Darray_to_threejs(result);

    robot.joints[joint].xform = result;
    simpleApplyMatrix(robot.joints[joint].geom,tempmat);

}

function transform_link(link,mstack) {
	//Tranforms the joint, given the origin is contained in mstack
	if (link == robot.base) {
		//Base placement
		var x = robot.origin.xyz[0];
	    var y = robot.origin.xyz[1];
	    var z = robot.origin.xyz[2];
	    var r = robot.origin.rpy[0];
	    var p = robot.origin.rpy[1];
	    var rot_y = robot.origin.rpy[2];

	    var result = generate_matrix_transform(x,y,z,r,p,rot_y);
    	var tempmat = matrix_2Darray_to_threejs(result);
    	simpleApplyMatrix(robot.links[robot.base].geom,tempmat);


    	//For Forward and Backward Movement (Robot Heading)
	    a = [0,0,1,1];
	    heading_local = transpose(a);
	    robot_heading = matrix_multiply(result,heading_local);

		//For Sideways movement (Robot Lateral)
		a = [1,0,0,1];
		lateral_local = transpose(a)
	    robot_lateral = matrix_multiply(result,lateral_local);

   		return result;

	}

	var joint = robot.links[link].parent_joint;
	var x = robot.joints[joint].origin.xyz[0];
    var y = robot.joints[joint].origin.xyz[1];
    var z = robot.joints[joint].origin.xyz[2];
    var r = robot.joints[joint].origin.rpy[0];
    var p = robot.joints[joint].origin.rpy[1];
    var rot_y = robot.joints[joint].origin.rpy[2];

    var result = generate_matrix_transform(x,y,z,r,p,rot_y);
    result = matrix_multiply(mstack,result);
    var a = generate_quaternion_matrix(robot.joints[joint]);
    var result = matrix_multiply(result,a);
    var tempmat = matrix_2Darray_to_threejs(result);
    simpleApplyMatrix(robot.links[link].geom,tempmat);

    return result;
}


function traverse_joint(joint,mstack) {
	transform_joint(joint,mstack);
	var link = robot.joints[joint].child;
	traverse2(link, mstack)
}

function traverse2(link, mstack) {

	mstack = transform_link(link,mstack);
	joint = robot.links[link].children[0];
	if (joint != undefined) {
		traverse_joint(joint, mstack);
	}
}

function traverse_link(link,mstack) {
	mstack = transform_link(link,mstack);
	limit = robot.links[link].children.length;
	for (i = (0); i < limit; i++) { 
		joint = robot.links[link].children[i];
		if (joint != undefined) {
			traverse_joint(joint, mstack);
		}
	}

}