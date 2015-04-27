//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

quaternion_from_axisangle
quaternion_normalize
quaternion_multiply
quaternion_to_rotation_matrix
*/

function quaternion_from_axisangle(axis,angle) {
	var result = [0, 0, 0, 0];
	result[0] = Math.cos(angle/2);
	result[1] = axis[0]*Math.sin(angle/2);
	result[2] = axis[1]*Math.sin(angle/2);
	result[3] = axis[2]*Math.sin(angle/2);
	return result;
}

function quaternion_to_rotation_matrix(q) {
	var result = generate_identity();
    
    result[0][0] = 1 - 2*(q[2]*q[2]) - 2*(q[3]*q[3]);
    result[0][1] = 2*q[1]*q[2] - 2*q[3]*q[0];
    result[0][2] = 2*q[1]*q[3] + 2*q[2]*q[0];

    result[1][0] = 2*q[1]*q[2] + 2*q[3]*q[0];
    result[1][1] = 1 - 2*(q[1]*q[1]) - 2*(q[3]*q[3]);
    result[1][2] = 2*q[2]*q[3] - 2*q[1]*q[0];

    result[2][0] = 2*q[1]*q[3] - 2*q[2]*q[0];
    result[2][1] = 2*q[2]*q[3] + 2*q[1]*q[0];
    result[2][2] = 1 - 2*(q[1]*q[1]) - 2*(q[2]*q[2]);

    return result;
}

function generate_quaternion_matrix(joint) {
    var result = quaternion_from_axisangle(joint.axis, joint.angle);
    result = quaternion_to_rotation_matrix(result); 
    return result; 

 }
