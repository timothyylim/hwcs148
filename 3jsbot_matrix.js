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

    for (var row = 0; row < newHeight; row++) {
        newMatrix[row] = [];

        for (var column = 0; column < newWidth; column++) { 
            var sum = 0;
            
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

	result = multiplyMatrices(multiplyMatrices(generate_translation_matrix(x,y,z),generate_rotation_matrix_X(r)),generate_rotation_matrix_Y(p));
	result = multiplyMatrices(result,generate_rotation_matrix_Z(rot_y));
	
	return result;
}
