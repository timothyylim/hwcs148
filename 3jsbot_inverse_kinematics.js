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
   	result = generate_translation_matrix(target_pos[0], target_pos[1], target_pos[2]);
	tempmat = matrix_2Darray_to_threejs(result);
	simpleApplyMatrix(target_geom,tempmat);

	//Setting Endeffector Geom
	result = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_local_pos);
   	result = generate_translation_matrix(result[0], result[1], result[2]);
   	tempmat = matrix_2Darray_to_threejs(result);
	simpleApplyMatrix(endeffector_geom,tempmat);

    update_ik = false;

}

function iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
	
	//Creating array to hold all the joints in the kinematic hierarchy
	parents = [];
	a = robot.joints[endeffector_joint].parent;
	while (a != 'base') {
		b = robot.links[a].parent_joint;
		a = robot.joints[b].parent; 
		parents.unshift(b);
	}

	parents.push(endeffector_joint);

	//Creating an Empty 6*N Jacobian Matrix
	var Jacobian = new Array(6);

	for (var i = 0; i < 6; i++) {
		Jacobian[i] = new Array(parents.length);
	}

	//Transforming the endeffector_local_pos to world coordinates
	ik = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_local_pos);


	//Setting the column vectors in the Jacobian, Ji
	for (i = 0; i < (parents.length); i++) { 
		joint = parents[i];
		cross = [];

		//Transforming joints coordinate to the world frame
		a = transpose([0,0,0,1]);
		m = robot.joints[joint].xform;
		worldframe = matrix_multiply(m,a);
		rotationMatrix = [];

		//Creating Rotation Matrix
		for (k = 0; k < (m.length-1); k++) {
			rotationMatrix[k] = [];
			for (var j = 0; j < m[0].length-1; j++) {
				rotationMatrix[k][j] = m[k][j];
			}
		}

		//Transforming axis to world frame
	    axis = matrix_multiply(rotationMatrix,transpose(robot.joints[joint].axis));
	    axis[0] = axis[0][0]; 
	    axis[1] = axis[1][0]; 
	    axis[2] = axis[2][0]; 

	    //Calculating the cross the product for the Jacobian Matrix
	    difference_array = []
	    difference_array[0] = ik[0][0] - worldframe[0][0];
	    difference_array[1] = ik[1][0] - worldframe[1][0];  
	    difference_array[2] = ik[2][0] - worldframe[2][0];

	    cross = vector_cross(axis,difference_array);

	    //Assigning values to the Jacobian Matrix
	    Jacobian[0][i] = cross[0];
	    Jacobian[1][i] = cross[1];
	    Jacobian[2][i] = cross[2];
	    Jacobian[3][i] = axis[0]
	    Jacobian[4][i] = axis[1];
	    Jacobian[5][i] = axis[2];
	  	
  	}
  	
	jacobianTranpose = transpose(Jacobian);

	//////FOR THE PSEUDOINVERSE CASE/////
	// jacobianTranpose = pseudoInverse(Jacobian);

	//Calculating delta x
	delta_x = [[0],[0],[0],[0],[0],[0]];

	x = target_pos[0][0] - ik[0][0];
	y = target_pos[1][0] - ik[1][0];
	z = target_pos[2][0] - ik[2][0];

	delta_x[0][0] = x; 
	delta_x[1][0] = y;
	delta_x[2][0] = z;

	//Storing the control values for the angles of the joints
	joint_controls = [];
	joint_controls = matrix_multiply(jacobianTranpose,delta_x);

	//Assigning control values to joints along the kinematic hierarchy
	for (i = 0; i < (parents.length); i++) { 
  		joint = parents[i];
  		robot.joints[joint].control += 0.1*joint_controls[i];
  	}
}

function pseudoInverse(Jacobian) {
	a = numeric.inv(matrix_multiply(transpose(Jacobian), Jacobian));
	jacobianTranpose = matrix_multiply(a,transpose(Jacobian));
	return jacobianTranpose;
}