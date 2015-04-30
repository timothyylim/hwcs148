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



function quaternion_normalize(matrix){
    sum = 0;
    n = 0;
    result = [];

    //1-d Matrix
    if (matrix[0].length == undefined) {
        for (var i = 0; i < matrix.length; i++) {
            a = matrix[i]
            a = a*a;
            sum += a;
            n += 1;

            len = Math.sqrt(sum)
        }

        for (var i = 0; i < matrix.length; i++) {
            result[i] = matrix[i]/len;
        }
        return result
    }

    //Multiple Dimensions
    for (var i = 0; i < matrix.length; i++) {
        for (var j = 0; j < matrix[0].length; j++) {
            a = matrix[i][j]
            a = a*a;
            sum += a;
            n += 1;

            len = Math.sqrt(sum)
        }
    }
    
    for (var i = 0; i < matrix.length; i++) {
        result[i] = [];  
        for (var j = 0; j < matrix[0].length; j++) {
            result[i][j] = matrix[i][j]/len;
        }
    }

    return result;

}
