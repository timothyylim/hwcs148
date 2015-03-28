//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

multiplyMatrices
matrix_transpose
vector_normalize
vector_cross
generate_identity
generate_translation_matrix
generate_rotation_matrix_X
generate_rotation_matrix_Y
generate_rotation_matrix_Z
*/


function multiplyMatrices(first, second) {
    var newMatrix = [],
        newWidth = second[0].length,
        newHeight = first.length;
    //iterating through first matrix rows
    for (var row = 0; row < newHeight; row++) {
        newMatrix[row] = [];
        //iterating through second matrix columns
        for (var column = 0; column < newWidth; column++) { 
            var sum = 0;
            //calculating sum of pairwise products
            for (var index = 0; index < first[0].length; index++) {
                sum += first[row][index] * second[index][column];
            }
            newMatrix[row][column] = sum;
        }
    }
    return newMatrix;
}

function generate_identity() {
	var result = []
	result = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1] 
    ];

    return result;
}

function generate_translation_matrix(x,y,z) {
	var result = generate_identity();
	result[0][3] = x;
	result[1][3] = y;
	result[2][3] = z;

	return result;
} 

function generate_rotation_matrix_X(r) {
	var result = generate_identity();
	result[1][1] = Math.cos(r);
	result[1][2] = -1*(Math.sin(r));
	result[2][1] = Math.sin(r);
	result[2][2] = Math.cos(r);
	return result;
}

function generate_rotation_matrix_Y(p) {
	var result = generate_identity();
	result[0][0] = Math.cos(p);
	result[0][2] = Math.sin(p);
	result[2][0] = -1*(Math.sin(p));
	result[2][2] = Math.cos(p);
	return result;
}

function generate_rotation_matrix_Z(rot_y) {
	var result = generate_identity();
	result[0][0] = Math.cos(rot_y);
	result[0][1] = -1*(Math.sin(rot_y));
	result[1][0] = Math.sin(rot_y);
	result[1][1] = Math.cos(rot_y);
	return result;
}

function generate_matrix_transform(x,y,z,r,p,rot_y) {

	result = multiplyMatrices(generate_translation_matrix(x,y,z),generate_rotation_matrix_X(r));

	result = multiplyMatrices(result,generate_rotation_matrix_Y(p));
	result = multiplyMatrices(result,generate_rotation_matrix_Z(rot_y));
	
	return result;
}

function transform_joint(joint,mstack) {
	
	var x = robot.joints[joint].origin.xyz[0];
    var y = robot.joints[joint].origin.xyz[1];
    var z = robot.joints[joint].origin.xyz[2];
    var r = robot.joints[joint].origin.rpy[0];
    var p = robot.joints[joint].origin.rpy[1];
    var rot_y = robot.joints[joint].origin.rpy[2];

    var result = multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y));
    var tempmat = matrix_2Darray_to_threejs(result);

    simpleApplyMatrix(robot.joints[joint].geom,tempmat);

    return result;
}

function transform_link(link,mstack) {
	
	if (link == "link1") {

		var link = robot.base;
		var x = robot.origin.xyz[0];
	    var y = robot.origin.xyz[1];
	    var z = robot.origin.xyz[2];
	    var r = robot.origin.rpy[0];
	    var p = robot.origin.rpy[1];
	    var rot_y = robot.origin.rpy[2];

	    result = multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y));
	    var tempmat = matrix_2Darray_to_threejs(result);
	    
	    simpleApplyMatrix(robot.links[link].geom,tempmat);
   		
   		return result;
	}

	var joint = robot.links[link].parent_joint;
	var x = robot.joints[joint].origin.xyz[0];
    var y = robot.joints[joint].origin.xyz[1];
    var z = robot.joints[joint].origin.xyz[2];
    var r = robot.joints[joint].origin.rpy[0];
    var p = robot.joints[joint].origin.rpy[1];
    var rot_y = robot.joints[joint].origin.rpy[2];

    result = multiplyMatrices(mstack,generate_matrix_transform(x,y,z,r,p,rot_y));
    var tempmat = matrix_2Darray_to_threejs(result);
    
    simpleApplyMatrix(robot.links[link].geom,tempmat);

    return result;
}

/*
Note:
We traverse through the base link 
then call traverse link > traverse joint > traverse link etc.. 
until we have recursively traversed all 'limbs'
of the given robot 
*/

function traverse_base_link(link,mstack) {
	// Traverse the first link in the robot 
	mstack = transform_link(link,mstack);
	limit = robot.links[link].children.length;
	
	for (i = (0); i < limit; i++) { 
		joint = robot.links[link].children[i];
		
		if (joint != undefined) {
			traverse_joint(joint, mstack);
		}
	}

}

function traverse_child_links(link, mstack) {
	// Traverse through all links which come from joints
	mstack = transform_link(link,mstack);
	joint = robot.links[link].children[0];
	
	if (joint != undefined) {
		traverse_joint(joint, mstack);
	}
}


function traverse_joint(joint,mstack) {
	// Travese through the joints of the robot given the mstack
	transform_joint(joint,mstack);
	var link = robot.joints[joint].child;

	// Traverse to the children links 
	traverse_child_links(link, mstack)
}
