/* ******************************************************************************
Author: Gerik Zatorski

Dependencies:
- two.js
****************************************************************************** */

// these must be defined in a previously loaded file (ex two.js)
var two;
var window_height;
var window_width;

// Set vector weights
var weight_proj = 0.2;
var weight_neighbors = 0.65;
var weight_noise = 0.15;
var weight_goal = 0;

/* Class to model starlings */
class Boid {
    constructor(x,y,heading) {
	this.x = x;
	this.y = y;
	this.heading = new Two.Vector(heading.x,heading.y);
	this.speed = 10;
	this.size = 5.0;

	this.circle = two.makeCircle(this.x, this.y, this.size)
	this.circle.fill = '#000000';
	this.circle.opacity = 0.80;
	this.circle.noStroke();
    }
    
    move() {
	this.x += this.heading.x * this.speed;
	this.y += this.heading.y * this.speed;

	// Bounce off window edges
	if (this.x < 0 || this.x > window_width) {
	    this.x -= 2 * this.heading.x * this.speed;
	    this.heading.x = -this.heading.x;
	}
	if (this.y < 0 || this.y > window_height) {
	    this.y -= 2 * this.heading.y * this.speed;
	    this.heading.y = -this.heading.y;
	}

	this.circle.translation.set(this.x,this.y);
    }
   
    change_heading(v) {
	this.heading.x = v.x;
	this.heading.y = v.y;
    }

}


class Goal {
    constructor(x,y) {
	this.x = x;
	this.y = y;
    }
}

class Predator {
    constructor(x,y) {
	this.x = x;
	this.y = y;
    }

    move(x,y) {
	this.x = x;
	this.y = y;
    }
}

/* Class to represent set of boids in murmuration */
class Murmuration {
    constructor(n) {
	this.numBoids = n;
	this.time = 0;

	// create array of boids
	this.boids = [];
	for (var i = 0; i < n; i++) {
	    this.boids.push(new Boid(window_width/2+getRandomInt(-100,100), window_height/2+getRandomInt(-100,100), getRandomUnitVector()));
	}

/*	// set custom properties for debugging
	this.boids[0].circle.fill = "#000000";
	for (var i = 0; i < this.numBoids; i++) {
	    this.boids[i].x = window_width/2;
	}
	this.boids[0].y = window_height/2+200;
	this.boids[1].y = window_height/2+130;
*/
	// add in predator
	this.predator = new Predator(0,0);

	// goal position initiated on click
	this.goal_exists = false;
    }

    random_moves() {
	for (var i = 0; i < this.numBoids; i++) {
	    this.boids[i].move_random();
        }
    }

    calc_velocities() {

	var diff_vectors = [];

	var last_states = [];
	for (i = 0; i < this.numBoids; i++) {
	    last_states.push(this.boids[i].heading);
	}

	/* Get Movement data for each boid */
	for (var i = 0; i < this.numBoids; i++) {

	    var thetas = [];
	    var distances = [];
	    var neighbors = [];
	    var containing_neighbor = [];
	    var boundaries = [];
	    var status = [];

	    /* Test relationships with other boids */
	    for (var j = 0; j < this.numBoids; j++) {
		// don't test relationship with self
		if (j == i) {
		    distances.push(null);
		} else {
		    var d = Math.sqrt( Math.pow(this.boids[i].x - this.boids[j].x, 2) + Math.pow(this.boids[i].y - this.boids[j].y, 2));
		    distances.push(d);
		    var dtheta;
		    var vision_blocked = false;
		    if (d > this.boids[j].size) {
			dtheta = Math.asin(this.boids[j].size/d);
			var rel_theta = Math.atan2( (this.boids[j].y-this.boids[i].y) , (this.boids[j].x-this.boids[i].x) );
			thetas.push([ rel_theta - dtheta , j , "open"]);
			thetas.push([ rel_theta + dtheta , j , "close"]);
		    } else { // if bird is within this other bird
			vision_blocked = true;
			containing_neighbor = j;
		    }
		}
	    } // end of comparisons to other boids

	    /* setup thetas in eventq for comparison, ie set range of thetas to [0,2pi) */
	    for (var x = 0; x < thetas.length; x++) { if (thetas[x][0] < 0) thetas[x][0] += 2*Math.PI; }

	    var vect_neighbors = new Two.Vector();
	    var vect_proj = new Two.Vector();

	    /* if vision is clear, cycle through event q (thetas) in order to determine boundaries and line-of-sight neighbors */
	    if (!vision_blocked) {
		thetas = thetas.sort(ComparatorThetas);
		// cycle through thetas (event q) for domain boundaries and neighbors
		for (var z = 0; z < thetas.length; z++) {
		    if (thetas[z][2] == "close") { // if end of circle
			if ( status.length == 1 ) boundaries.push(thetas[z][0]);
			var index = status.indexOf(thetas[z][1]);
			status.splice(index,1);
		    } else { // if beginning of circle
			if ( status.length == 0 ) boundaries.push(thetas[z][0]);
			status.push(thetas[z][1]);
		    }
		    // Add neighbors in line-of-sight
		    var closest_boid = null;
		    var smallest_dist = null;
		    if (status.length > 0) {
			for (var a = 0; a < status.length; a++) {
			    if (distances[a] < smallest_dist || smallest_dist == null) {
				closest_boid = status[a];
				smallest_dist = distances[a];
			    }
			}
			if (neighbors.indexOf(closest_boid) == -1) neighbors.push(closest_boid);
		    }
		}

		/* reset range of boundaries: (-pi,pi] */
		for (var x = 0; x < boundaries.length; x++) { if (boundaries[x] > Math.PI) boundaries[x] -= 2*Math.PI; }

		/* Calculate neighbor orientation vector */
		var neighbors_extended = []; 
		for (var x = 0; x < neighbors.length; x++) { neighbors_extended.push([ neighbors[x] , distances[neighbors[x]] ]); }
		neighbors_extended.sort(ComparatorNeighbors);
		for (var x = 0; x < 4 && x < neighbors_extended.length; x++) { vect_neighbors.addSelf(last_states[neighbors_extended[x][0]]); }
		vect_neighbors.divideScalar(Math.min(4, neighbors_extended.length));

		/* Calculate projection vector */
		var x_vectors = [];
		var y_vectors = [];
		for (var x = 0; x < boundaries.length; x++) {
		    if (boundaries[x] > Math.PI) boundaries[x] -= Math.PI;
		    if (boundaries[x] < -Math.PI) boundaries[x] += Math.PI;
		    x_vectors.push(Math.cos(boundaries[x]));
		    y_vectors.push(Math.sin(boundaries[x]));
		}

		vect_proj.set( x_vectors.reduce(add,0)/x_vectors.length , y_vectors.reduce(add,0)/y_vectors.length );
	    }
	    /* but, if vision is blocked, set neighbor and projection vectors */
	    else {
		vect_neighbors = last_states[containing_neighbor];
		vect_proj = last_states[i];
	    }	    

	    /* Noise vector */
	    var vect_noise = getRandomUnitVector();

	    vect_proj.multiplyScalar(weight_proj);
	    vect_neighbors.multiplyScalar(weight_neighbors);
	    vect_noise.multiplyScalar(weight_noise);

	    var diff_vect = new Two.Vector(0,0);
	    diff_vect.addSelf(vect_proj);
	    diff_vect.addSelf(vect_neighbors);
	    diff_vect.addSelf(vect_noise);


	    /* Goal vector */
	    if (this.goal_exists) {
		var goal_theta = Math.atan2( (this.goal.y-this.boids[i].y) , (this.goal.x-this.boids[i].x) );
		var vect_goal = new Two.Vector( Math.cos(goal_theta) , Math.sin(goal_theta) );
		
		vect_goal.multiplyScalar(weight_goal);
		diff_vect.addSelf(vect_goal);
	    }

	    // what about predators
	    var running = false;
	    var dist_predator = Math.sqrt( Math.pow(this.boids[i].x - this.predator.x, 2) + Math.pow(this.boids[i].y - this.predator.y, 2));
	    if (dist_predator < 10 * this.boids[i].speed) { running = true; }
	    
	    if (running) {
		var predator_theta = Math.atan2( (this.predator.y-this.boids[i].y) , (this.predator.x-this.boids[i].x) );
		var vect_run = new Two.Vector( Math.cos(predator_theta) , Math.sin(predator_theta) );
		vect_run.negate();
		diff_vectors.push( vect_run );
	    } else {
		diff_vectors.push(diff_vect);
	    }


	    // Debugging
	    // if(i==0) console.log("vect_proj = " + vect_proj);
	    // if(i==0) console.log("vect_neighbors = " + vect_neighbors);
	    // if(i==0) console.log("vect_noise = " + vect_noise);

        } // end of i bird loop
	return diff_vectors;
}

    UpdateBirds() {
	/*move all boids 1 step in current heading (Eq 2 in paper) */
	for (var i = 0; i < this.numBoids; i++){
	    this.boids[i].move();
	}

	/* set heading for next step (Eq 3 in paper) */
	var new_velocities = this.calc_velocities();
	for (var i = 0; i < this.numBoids; i++){
	    this.boids[i].change_heading( new_velocities[i] );
	}

	this.time++;
    }
}


/******************** Helper functions ********************/

// Returns a random integer between min (included) and max (excluded)
function getRandomInt(min, max) {
    min = Math.ceil(min);
    max = Math.floor(max);
    return Math.floor(Math.random() * (max - min)) + min;
}

/* Returns a random unit vector */
function getRandomUnitVector() {
    var theta =  Math.random() * 2 * Math.PI;
    return new Two.Vector(Math.cos(theta), Math.sin(theta)).normalize();
}

/* For comparing thetas array */
function ComparatorThetas(a, b) {
    if (a[0] < b[0]) return -1;
    if (a[0] > b[0]) return 1;
    return 0;
}

/* For comparing neighbor arrays by distance */
function ComparatorNeighbors(a, b) {
    if (a[1] < b[1]) return 1;
    if (a[1] > b[1]) return -1;
    return 0;
}

/* simple adding function to use with reduce on javascript arrays */
function add(a, b) {
    return a + b;
}
