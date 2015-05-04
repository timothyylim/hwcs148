//////////////////////////////////////////////////
/////     INVERSE KINEMATICS 
/////     Resolved-rate IK with geometric jacobian
//////////////////////////////////////////////////

// CS148: generate joint controls to move robot to move robot endeffector to target location

/*
CS148: reference code has functions for:

robot_inverse_kinematics
iterate_inverse_kinematics
*/

function robot_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {

    if (update_ik) {
        iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos);
        endeffector_geom.visible = true;
        target_geom.visible = true;
    }
    else {
        endeffector_geom.visible = false;
        target_geom.visible = false;
    }

    //Setting Target Geom
   	output = generate_translation_matrix(target_pos[0], target_pos[1], target_pos[2]);
	tempmat = matrix_2Darray_to_threejs(output);
	simpleApplyMatrix(target_geom,tempmat);

	//Setting Endeffector Geom
	output = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_local_pos);
   	output = generate_translation_matrix(output[0], output[1], output[2]);
   	tempmat = matrix_2Darray_to_threejs(output);
	simpleApplyMatrix(endeffector_geom,tempmat);

    update_ik = false;

}

function iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
	
	p = [];

	arrayDiff = []

	a = robot.joints[endeffector_joint].parent;
	while (a != 'base') {
		b = robot.links[a].parent_joint;
		a = robot.joints[b].parent; 
		p.unshift(b);
	}

	p.push(endeffector_joint);

	var Jacobian = new Array(6);

	for (var i = 0; i < 6; i++) {
		Jacobian[i] = new Array(p.length);
	}

	ik = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_local_pos);


	for (i = 0; i < (p.length); i++) { 
		joint = p[i];
		cross = [];

		a = transpose([0,0,0,1]);
		m = robot.joints[joint].xform;
		worldframe = matrix_multiply(m,a);
		rotationMatrix = [];


		for (k = 0; k < (m.length-1); k++) {
			rotationMatrix[k] = [];
			for (var j = 0; j < m[0].length-1; j++) {
				rotationMatrix[k][j] = m[k][j];
			}
		}

	    axis = matrix_multiply(rotationMatrix,transpose(robot.joints[joint].axis));
	    axis[0] = axis[0][0]; 
	    axis[1] = axis[1][0]; 
	    axis[2] = axis[2][0]; 

	    arrayDiff[0] = ik[0][0] - worldframe[0][0];
	    arrayDiff[1] = ik[1][0] - worldframe[1][0];  
	    arrayDiff[2] = ik[2][0] - worldframe[2][0];

	    cross = vector_cross(axis,arrayDiff);

	    //Assigning values to the Jacobian Matrix
	    Jacobian[0][i] = cross[0];
	    Jacobian[1][i] = cross[1];
	    Jacobian[2][i] = cross[2];
	    Jacobian[3][i] = axis[0]
	    Jacobian[4][i] = axis[1];
	    Jacobian[5][i] = axis[2];
	  	
  	}
  	
	jacobianTranpose = transpose(Jacobian);
	// Uncomment for pseudoInverse case
	// jacobianTranpose = pseudoInverse(Jacobian);

	delta_x = [[0],[0],[0],[0],[0],[0]];

	y = target_pos[1][0] - ik[1][0];
	z = target_pos[2][0] - ik[2][0];
	x = target_pos[0][0] - ik[0][0];

	delta_x[0][0] = x; 
	delta_x[1][0] = y;
	delta_x[2][0] = z;

	joint_controls = [];
	joint_controls = matrix_multiply(jacobianTranpose,delta_x);

	i = 0;
	while (i < p.length){
  		joint = p[i];
  		robot.joints[joint].control += 0.1*joint_controls[i];
  		i = i+1;
	}

}

function pseudoInverse(Jacobian) {
	a = numeric.inv(matrix_multiply(transpose(Jacobian), Jacobian));
	jacobianTranpose = matrix_multiply(a,transpose(Jacobian));
	return jacobianTranpose;
}