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

function robot_inverse_kinematics(ik_target, endeffector, ik_local) {
  iterate_inverse_kinematics(ik_target, endeffector, ik_local);
}

function iterate_inverse_kinematics(target_pos, endeffector_joint, endeffector_local_pos) {
  Jacobian = compute_jacobian(target_pos, endeffector_joint, endeffector_local_pos);
  // jacobianTranpose = transpose(Jacobian);
  // invJacobian = numeric.inv(Jacobian);
  // console.log(Jacobian)
  // console.log(jacobianTranpose);
}

function compute_jacobian(target_pos, endeffector_joint, endeffector_local_pos) {
  //Putting the chain of joints in the endeffector's kinematic chain into an array
  parents = [];
  a = robot.joints[endeffector_joint].parent;

  while (a != 'base') {
    b = robot.links[a].parent_joint;
    a = robot.joints[b].parent; 
    parents.unshift(b);
  }

  parents.push(endeffector_joint);
  // console.log(parents)
  //Calculating the Jacobian

  //Creating an Empty 6*N Jacobian Matrix
  var Jacobian = new Array(6);

  for (var i = 0; i < 6; i++) {
    Jacobian[i] = new Array(parents.length);
  }

  //Transforming the endeffector_local_pos to world coordinates
  var ik = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_local_pos);
  // result = generate_translation_matrix(ik[0],ik[1],ik[2])
  // var tempmat = matrix_2Darray_to_threejs(result);
  // simpleApplyMatrix(robot.joints[endeffector_joint].geom,tempmat);

  //Setting the remaining column vectors in the Jacobian
  for (i = 0; i < (parents.length); i++) { 
    joint = parents[i];
    cross = [];

    a = [0,0,0,1];
    a = transpose(a);
    worldframe = matrix_multiply(robot.joints[joint].xform,a);


    cross[0] = worldframe[0];
    cross[1] = worldframe[1];
    cross[2] = worldframe[2];
    result = generate_translation_matrix(cross[0],cross[1],cross[2])
    var tempmat = matrix_2Darray_to_threejs(result);
    simpleApplyMatrix(robot.joints[joint].geom,tempmat);

    axis = robot.joints[joint].axis; 

    difference_array = []
    difference_array[0] = ik[0] - worldframe[0];
    difference_array[1] = ik[1] - worldframe[1];  
    difference_array[2] = ik[2] - worldframe[2];

    cross = vector_cross(axis,difference_array);

    Jacobian[0][i] = cross[0];
    Jacobian[1][i] = cross[1];
    Jacobian[2][i] = cross[2];
    Jacobian[3][i] = axis[0]
    Jacobian[4][i] = axis[1];
    Jacobian[5][i] = axis[2];
  }
    
  // console.log(cross);
  console.log(Jacobian);
  // console.log(transpose(Jacobian))

  return Jacobian;
}