//////////////////////////////////////////////////
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
 	
 	mstack = generate_identity();
 	traverse_forward_kinematics_base_link(robot.base, mstack);
 }


function joint_transform(joint,mstack) {
	
	x = robot.joints[joint].origin.xyz[0];
    y = robot.joints[joint].origin.xyz[1];
    z = robot.joints[joint].origin.xyz[2];
    r = robot.joints[joint].origin.rpy[0];
    p = robot.joints[joint].origin.rpy[1];
    rot_y = robot.joints[joint].origin.rpy[2];

    
    temp_matrix = matrix_2Darray_to_threejs(multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y)));

    simpleApplyMatrix(robot.joints[joint].geom, temp_matrix);

    return multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y));
}

function link_transform(link,mstack) {
	
	if (link == robot.base) {

		link = robot.base;
		x = robot.origin.xyz[0];
	    y = robot.origin.xyz[1];
	    z = robot.origin.xyz[2];
	    r = robot.origin.rpy[0];
	    p = robot.origin.rpy[1];
	    rot_y = robot.origin.rpy[2];

	    temp_matrix = matrix_2Darray_to_threejs(multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y)));
	    
	    simpleApplyMatrix(robot.links[link].geom,temp_matrix);
   		
   		return multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y));
	}

	joint = robot.links[link].parent_joint;
	x = robot.joints[joint].origin.xyz[0];
    y = robot.joints[joint].origin.xyz[1];
    z = robot.joints[joint].origin.xyz[2];
    r = robot.joints[joint].origin.rpy[0];
    p = robot.joints[joint].origin.rpy[1];
    rot_y = robot.joints[joint].origin.rpy[2];

    
    temp_matrix = matrix_2Darray_to_threejs(multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y)));
    

    simpleApplyMatrix(robot.links[link].geom,temp_matrix);

    return multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y));
}

function traverse_forward_kinematics_base_link(link,mstack) {

	mstack = link_transform(link,mstack);
	limit = robot.links[link].children.length;
	
	for (i = (0); i < limit; i++) { 
		joint = robot.links[link].children[i];
		
		if (joint != undefined) {
			traverse_forward_kinematics_joint(joint, mstack);
		}
	}

}


function traverse_forward_kinematics_link(link, mstack) {

	mstack = link_transform(link,mstack);

	joint = robot.links[link].children[0];
	
	if (joint != undefined) {
		traverse_forward_kinematics_joint(joint, mstack);
	}
}


function traverse_forward_kinematics_joint(joint,mstack) {

	joint_transform(joint,mstack);
	var link = robot.joints[joint].child;

	traverse_forward_kinematics_link(link, mstack)
}
