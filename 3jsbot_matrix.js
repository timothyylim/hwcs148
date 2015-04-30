//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

/*
CS148: reference code has functions for:

matrix_multiply
matrix_transpose
vector_normalize
vector_cross
generate_identity
generate_translation_matrix
generate_rotation_matrix_X
generate_rotation_matrix_Y
generate_rotation_matrix_Z
*/

function matrix_multiply(m1, m2) {
    var result = [];
    for (var i = 0; i < m1.length; i++) {
        result[i] = [];
        for (var j = 0; j < m2[0].length; j++) {
            var sum = 0;
            for (var k = 0; k < m1[0].length; k++) {
                sum += m1[i][k] * m2[k][j];
            }
            result[i][j] = sum;
        }
    }
    // console.log(result);
    return result;
}

function vector_cross(v1,v2) {
	var result = [];
	result [0] = ( (v1[1] * v2[2]) - (v1[2] * v2[1]) );
	result [1] = ( (v1[2] * v2[0]) - (v1[0] * v2[2]) );
	result [2] = ( (v1[0] * v2[1]) - (v1[1] * v2[0]) );

	return result;
}

function transpose(matrix) {
	var result = [];

	//To handle an edge case when the matrix is a row matrix
	if (matrix[0].length == undefined) {
		for (var i = 0; i < matrix.length; i++) {
			result.push([matrix[i]]);
		}
		return result
	}

	for (var i = 0; i < matrix[0].length; i++) {
		result[i] = [];
	    for (var j = 0; j < matrix.length; j++) {
	        result[i][j] = matrix[j][i];
 	    }
	}
	return result;
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
	var a = generate_translation_matrix(x,y,z);
	var b = generate_rotation_matrix_X(r);
	result = matrix_multiply(a,b);

	var a = result;
	var b = generate_rotation_matrix_Y(p);
	result = matrix_multiply(a,b);

	var a = result;
	var b = generate_rotation_matrix_Z(rot_y);
	result = matrix_multiply(a,b);
	
	return result;
}