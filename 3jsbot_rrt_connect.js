//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// CS148: 
// implement RRT-Connect by Kuffner and LaValle (2000)
//    paper link: http://msl.cs.uiuc.edu/~lavalle/papers/KufLav00.pdf

// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by robot_collision_test()

/*
CS148: reference code has functions for:

    tree_add_vertex
    tree_add_edge
    random_config
    new_config
    nearest_neighbor
    rrt_extend
    rrt_connect
    find_path
    path_dfs
*/

function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) {
        q_goal_config[i] = 0;
    }

    // CS148: add necessary RRT initialization here

    // flag to continue rrt iterations
    rrt_iterate = true;

    //Setting variables for the algorithm
    x_max = robot_boundary[1][0] + 2;
    x_min = robot_boundary[0][0] - 2;
    z_max = robot_boundary[1][0] + 2;
    z_min = robot_boundary[0][0] - 2;

    epsilon = 0.5;
    distance = 0;
    for (i = 0; i < (q_goal_config.length); i++ ) {
        diff = epsilon*epsilon;
        distance += diff;
    }
    inc_dist = Math.sqrt(distance);
    robot_path = new Array();

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
    init_time = cur_time;
    tree_1 = tree_init(q_goal_config);
    tree_2 = tree_init(q_start_config);
}


function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        // CS148: implement RRT iteration here
        rrt_out = rrt_connect_planner(tree_1, tree_2)
        temp = tree_1;
        tree_1 = tree_2;
        tree_2 = temp;

        // update time marker for last iteration update
        cur_time = Date.now();

        return rrt_out;

    }

    // return path not currently found
    return false;
}

function rrt_connect_planner(tree1, tree2) {
    new_vertex = extend(tree1,random_config());

    if ( new_vertex != "Trapped") {
        isConnected = connect(new_vertex, tree2);
    }
    return isConnected;
}

function connect (q,tree) {
    near_neighbor = nearest_neighbor(q.vertex, tree);
    sum_distance = 0;

    for (i = 0; i < q.vertex.length; i++ ) {
        dist1 = near_neighbor.vertex[i] - q.vertex[i];
        dist1_square = dist1*dist1;
        sum_distance += dist1_square;
    }

    final_distance = Math.sqrt(sum_distance);
    if (final_distance<inc_dist) {
        rrt_iterate = false;
        find_path(q,near_neighbor);
        return "reached"
    }
    return false
}

function find_path(q1,q2) {
    //For the first tree
    while (q1.parent != null) {
        robot_path.unshift(q1);    
        q1 = q1.parent;
    }
    robot_path.unshift(q1);

    //For the second tree
    while (q2.parent != null) {
        robot_path.push(q2);   
        q2 = q2.parent;
    }
    robot_path.push(q2);

    //Drawing out the Path

    for (i=0;i<robot_path.length;i++) {
        robot_path[i].geom.material.color = {r:1,g:0,b:0};
    }

}
function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].parent = null;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function extend(tree,q) {
    q_near = nearest_neighbor(q,tree);
    q_new = new_config(q_near,q, epsilon);
    if (robot_collision_test(q_new) == false) {
        q_new = tree_add_vertex(q_new, tree, q_near);
        return q_new;
    }
    return "Trapped";
}

function random_config() {

    x = Math.random() * (x_max - x_min) + x_min;
    y = 0;
    z = Math.random() * (z_max - z_min) + z_min;
    r = 0;
    p = Math.random() * Math.PI *2;
    rot_y = 0;

    q = [x,y,z,r,p,rot_y];

    for (i in robot.joints) {
        q = q.concat(Math.random() * Math.PI *2);
    }

    return q
}

function nearest_neighbor(q_random,tree) {
    shortest_dist = Number.MAX_VALUE;;
    for (x in tree.vertices) {
        sum_distance = 0;

        vertex = tree.vertices[x].vertex;

        for (i = 0; i < vertex.length; i++ ) {
            dist1 = q_random[i] - vertex[i];
            dist1_square = dist1*dist1;
            sum_distance += dist1_square;
        }

        final_distance = Math.sqrt(sum_distance);
        if (final_distance<shortest_dist) {
            shortest_dist = final_distance;
            near_neighbor = tree.vertices[x];
        }
    }
    return near_neighbor;
}

function new_config (q_near, q_dest, incremental_distance) {
    vector = [];
    output = [];

    for (i=0;i<q_dest.length; i++) {
        vector[i] = q_dest[i] - q_near.vertex[i]; 
    }

    norm_vector = quaternion_normalize(vector);

    for (i=0;i<q_near.vertex.length; i++) {
        output[i] = q_near.vertex[i] + (norm_vector[i]*incremental_distance); 
    }
    return output;
} 

function tree_add_vertex(q,tree,parent) {
    index = tree.newest + 1;

    tree.vertices[index] = {};
    tree.vertices[index].vertex = q;
    tree.vertices[index].parent = parent;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[index]);

    // maintain index of newest vertex added to tree
    tree.newest = index;

    return tree.vertices[tree.newest];  

}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);

    vertex.geom = temp_mesh;
}