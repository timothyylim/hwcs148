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
    rrt_iter_count = 0;
    iter_limit = 0;

    //Setting variables for the algorithm
    x_max = robot_boundary[1][0];
    x_min = robot_boundary[0][0];
    z_max = robot_boundary[1][0];
    z_min = robot_boundary[0][0];

    epsilon = 0.5;
    distance = 0;
    for (i = 0; i < (q_goal_config.length); i++ ) {
        diff = epsilon*epsilon;
        distance += diff;
    }
    inc_dist = Math.sqrt(distance);
    robot_path = new Array();

    cur_time = Date.now();
    init_time = cur_time;
    tree_a = tree_init(q_goal_config);
    tree_b = tree_init(q_start_config);
}

function tree_add_vertex(q,tree,parent) {
    i = tree.newest + 1;

    tree.vertices[i] = {};
    tree.vertices[i].vertex = q;
    tree.vertices[i].parent = parent;

    add_config_origin_indicator_geom(tree.vertices[i]);
    tree.newest = i;

    return tree.vertices[tree.newest];  

}

function random_config() {

    x = (x_max - x_min) * Math.random() + x_min;
    z = (z_max - z_min) * Math.random() + z_min;
    p = Math.random() * Math.PI *2;
    
    y = 0;
    r = 0;
    rot_y = 0;

    q = [x,y,z,r,p,rot_y];

    for (i in robot.joints) {
        q = q.concat(Math.random() * Math.PI *2);
    }

    return q
}


function new_config (qNear, q_dest, delta_q) {

    vect = [];
    q = [];

    for (i=0;i<q_dest.length; i++) {
        vect[i] = q_dest[i] - qNear.vertex[i]; 
    }

    norm_vect = quaternion_normalize(vect);

    for (i=0;i<qNear.vertex.length; i++) {
        q[i] = qNear.vertex[i] + (norm_vect[i]*delta_q); 
    }

    return q;
} 


function nearest_neighbor(rand_q,tree) {
    shortestDistance = Number.MAX_VALUE;;
    for (x in tree.vertices) {
        distance = 0;
        a = tree.vertices[x].vertex;

        for (i = 0; i < a.length; i++ ) {
            difference = (rand_q[i] - a[i])*(rand_q[i] - a[i]);
            distance += difference;
        }

        distance = Math.sqrt(distance);
        if (distance<shortestDistance) {
            shortestDistance = distance;
            NearestNeighbour  = tree.vertices[x];
        }
    }
    return NearestNeighbour ;
}


function rrt_extend(tree,q) {

    qNear = nearest_neighbor(q,tree);
    qNew = new_config(qNear,q, epsilon);

    if (robot_collision_test(qNew) == false) {
        return tree_add_vertex(qNew, tree, qNear);
    }
    return "Trapped";
}


function rrt_connect (q,tree) {

    NearestNeighbour  = nearest_neighbor(q.vertex, tree);
    distance = 0;
    


    for (i = 0; i < q.vertex.length; i++ ) {

        difference = NearestNeighbour .vertex[i] - q.vertex[i];
        difference = difference*difference;
        distance += difference;
    }

    z = Math.sqrt(distance);
    

    if (z<inc_dist) {
        rrt_iterate = false;
        find_path(q, true);
        find_path(NearestNeighbour , false);
        time = (Date.now()- init_time)/1000;
        console.log("The time is " + time);

        for (i=0;i<robot_path.length;i++) {
            robot_path[i].geom.material.color = {r:1,g:0,b:0};
        }

        return "reached"
    }
    return false
}


function find_path(q, bool) {

    if (bool == true){

        while (q.parent != null) {
            robot_path.unshift(q);    
            q = q.parent;
        }
        robot_path.unshift(q);  

    }

    else{

        while (q.parent != null) {
            robot_path.push(q);   
            q = q.parent;
        }
        robot_path.push(q); 
    }
}


function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        var output = rrt_connect_planner(tree_a, tree_b)
        temp = tree_a;
        tree_a = tree_b;
        tree_b = temp;
        rrt_iter_count++;

        cur_time = Date.now();

        return output;

    }

    return false;
}

function rrt_connect_planner(tree_a, sree_b) {
    qNew = rrt_extend(tree_a,random_config());

    if ( qNew != "Trapped") {
        var connected = rrt_connect(qNew, tree_b);
    }
    return connected;
}


function tree_init(q) {

    var tree = {};

    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].parent = null;

    add_config_origin_indicator_geom(tree.vertices[0]);

    tree.newest = 0;

    return tree;
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