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
 	
 	traverse_forward_kinematics_link(robot.base, generate_identity());
}

function transform_joint(joint,mstack) {
	var x = robot.joints[joint].origin.xyz[0];
    var y = robot.joints[joint].origin.xyz[1];
    var z = robot.joints[joint].origin.xyz[2];
    var r = robot.joints[joint].origin.rpy[0];
    var p = robot.joints[joint].origin.rpy[1];
    var rot_y = robot.joints[joint].origin.rpy[2];

    var result = generate_matrix_transform(x,y,z,r,p,rot_y);
    result = matrix_multiply(mstack,result);

    // var a = generate_quaternion_matrix(robot.joints[joint]);
    var b = quaternion_from_axisangle(robot.joints[joint].axis, robot.joints[joint].angle);
    b = quaternion_to_rotation_matrix(b); 
   
    var tempmat = matrix_2Darray_to_threejs(matrix_multiply(result,b));

    robot.joints[joint].xform = result;
    simpleApplyMatrix(robot.joints[joint].geom,tempmat);

}

function transform_link(link,mstack) {
	if (link == robot.base) {
		var x = robot.origin.xyz[0];
	    var y = robot.origin.xyz[1];
	    var z = robot.origin.xyz[2];
	    var r = robot.origin.rpy[0];
	    var p = robot.origin.rpy[1];
	    var rot_y = robot.origin.rpy[2];

	    var result = generate_matrix_transform(x,y,z,r,p,rot_y);
    	var tempmat = matrix_2Darray_to_threejs(result);
    	simpleApplyMatrix(robot.links[robot.base].geom,tempmat);

		lateral_local = [[1],[0],[0],[1]];
	    heading_local = [[0],[0],[1],[1]];
	    
	    robot_lateral = matrix_multiply(result,lateral_local);
	    robot_heading = matrix_multiply(result,heading_local);


	    if (typeof heading_geom === 'undefined') {
            var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);

            var temp_material = new THREE.MeshBasicMaterial( {color: 0x00ffff} )
            heading_geom = new THREE.Mesh(temp_geom, temp_material);
            scene.add(heading_geom);
        }
        a = generate_translation_matrix(robot_heading[0], robot_heading[1], robot_heading[2]);
        tempmat = matrix_2Darray_to_threejs(a);

        
       	simpleApplyMatrix(heading_geom,tempmat);
        if (typeof lateral_geom === 'undefined') {
            var temp_geom = new THREE.CubeGeometry(0.3, 0.3, 0.3);
            var temp_material = new THREE.MeshBasicMaterial( {color: 0x008888} )
            lateral_geom = new THREE.Mesh(temp_geom, temp_material);
            scene.add(lateral_geom);
        }
        b = generate_translation_matrix(robot_lateral[0], robot_lateral[1], robot_lateral[2]);
        tempmat = matrix_2Darray_to_threejs(b);
       	simpleApplyMatrix(lateral_geom,tempmat);

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


    var b = quaternion_from_axisangle(robot.joints[joint].axis, robot.joints[joint].angle);
    b = quaternion_to_rotation_matrix(b); 
    var result = matrix_multiply(result,b);
    var tempmat = matrix_2Darray_to_threejs(result);
  

    simpleApplyMatrix(robot.links[link].geom,tempmat);

    return result;
}


function traverse_forward_kinematics_joint(joint,mstack) {
	transform_joint(joint,mstack);
	traverse_forward_kinematics_link_non_base(robot.joints[joint].child, mstack)
}

function traverse_forward_kinematics_link_non_base(link, mstack) {

	mstack = transform_link(link,mstack);
	joint = robot.links[link].children[0];
	if (joint != undefined) {
		traverse_forward_kinematics_joint(joint, mstack);
	}
}

function traverse_forward_kinematics_link(link,mstack) {
	mstack = transform_link(link,mstack);
	limit = robot.links[link].children.length;
	for (i = (0); i < limit; i++) { 
		joint = robot.links[link].children[i];
		if (joint != undefined) {
			traverse_forward_kinematics_joint(joint, mstack);
		}
	}

}