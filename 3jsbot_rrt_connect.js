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

    // console.log(x_max);

    // x_max = 100;
    // x_min = -100;

    // z_max = 100;
    // z_min = -100;
    // console.log(x_min);
    // epsilon = 2*(x_max-x_min)/100;
    epsilon = 0.5;
    distance = 0;
    for (i = 0; i < (q_goal_config.length); i++ ) {
        diff = epsilon*epsilon;
        distance += diff;
    }
    inc_dist = Math.sqrt(distance);
    // inc_dist = 3;
    // console.log(inc_dist);
    // inc_dist = Math.sqrt((epsilon*epsilon)+(epsilon*epsilon));
    robot_path = new Array();

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
    init_time = cur_time;
    tree_a = tree_init(q_goal_config);
    tree_b = tree_init(q_start_config);
    // console.log(q_goal_config);
    // console.log(q_start_config);

    // console.log("planner initialized");
}


function robot_rrt_planner_iterate() {

    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

        // CS148: implement RRT iteration here
        // if (rrt_iter_count == iter_limit) {
        //     rrt_iterate = false;
        // }
        var result = rrt_connect_planner(tree_a, tree_b)
        temp = tree_a;
        tree_a = tree_b;
        tree_b = temp;
        rrt_iter_count++;

        // update time marker for last iteration update
        cur_time = Date.now();

        // console.log(time);
        return result;

    }

    // return path not currently found
    return false;
}

function rrt_connect_planner(tree_a, sree_b) {
    q_new = extend(tree_a,random_config());

    if ( q_new != "Trapped") {
        var connected = connect(q_new, tree_b);
    }
    return connected;
}

function connect (q,tree) {
    // console.log(q.vertex);
    n_neighbor = nearest_neighbor(q.vertex, tree);
    // console.log(n_neighbor.vertex.length);
    distance = 0;
    for (i = 0; i < q.vertex.length; i++ ) {
        // console.log(i);
        diff = n_neighbor.vertex[i] - q.vertex[i];
        // console.log(diff);
        diff = diff*diff;
        // console.log(diff);
        distance += diff;
        // console.log(distance);
        // console.log(distance);
    }
    z = Math.sqrt(distance);
    // console.log(z);
    if (z<inc_dist) {
        rrt_iterate = false;
        find_path(q);
        find_path2(n_neighbor);
        time = (Date.now()- init_time)/1000;
        // console.log("Found path")
        console.log("The time is " + time);
        // draw_highlighted_path(path)
        for (i=0;i<robot_path.length;i++) {
            robot_path[i].geom.material.color = {r:1,g:0,b:0};
        }

        return "reached"
    }
    return false
}

function find_path(q) {

    //For the first tree
    while (q.parent != null) {
        robot_path.unshift(q);    
        q = q.parent;
    }
    robot_path.unshift(q);  
}

function find_path2(q) {

    //For the second tree
    while (q.parent != null) {
        robot_path.push(q);   
        q = q.parent;
    }
    robot_path.push(q); 
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
    // console.log(q);
    q_near = nearest_neighbor(q,tree);
    // console.log(q_near);
    q_new = new_config(q_near,q, epsilon);
    // /console.log(q_new);
    // console.log(robot_collision_test(q_new));
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

function nearest_neighbor(rand_q,tree) {
    shortest_dist = Number.MAX_VALUE;;
    for (x in tree.vertices) {
        distance = 0;
        a = tree.vertices[x].vertex;
        // console.log(a.length);
        // console.log(rand_q[1]);
        // console.log(a[1]);
        for (i = 0; i < a.length; i++ ) {
            // console.log(i);
            diff = rand_q[i] - a[i];
            // console.log(diff);
            diff = diff*diff;
            // console.log(diff);
            distance += diff;
            // console.log(distance);
            // console.log(distance);
        }
        distance = Math.sqrt(distance);
        if (distance<shortest_dist) {
            shortest_dist = distance;
            n_neighbor = tree.vertices[x];
        }
    }
    return n_neighbor;
}

function new_config (q_near, q_dest, delta_q) {
    // Calculating gradient

    // console.log(q_near.vertex);
    // console.log(q_dest);
    /*
    z2 = q_dest[2];
    z1 = q_near.vertex[2];

    x2 = q_dest[0];
    x1 = q_near.vertex[0];

    a = z2-z1;
    b = x2-x1;

    len = Math.sqrt((a*a) + (b*b));
    a = a/len;
    b = b/len;
    c = Math.sqrt(a*a + b*b);

    q = new Array(q_near.vertex.length);
    for (i=0;i<q_near.vertex.length;i++) {
        q[i] = q_dest[i];
    }

    // console.log(q);
    q[0] = x1+ (delta_q*b);
    // y = 0;
    q[2] = z1 + (delta_q*a);
    // r = 0;
    // p = 0;
    // rot_y = 
*/
    vect = [];
    q = [];

    for (i=0;i<q_dest.length; i++) {
        vect[i] = q_dest[i] - q_near.vertex[i]; 
    }

    norm_vect = quaternion_normalize(vect);

    for (i=0;i<q_near.vertex.length; i++) {
        q[i] = q_near.vertex[i] + (norm_vect[i]*delta_q); 
    }
    // console.log(q);
    return q;
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