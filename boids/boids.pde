import java.util.Collections;
import java.util.Comparator;

Flock flock;
int n_boids = 20;
int t = 0;

float predator_radius = 20;

// control parameters (sum to 1)
float phi_p = 0.20;
float phi_a = 0.50;
float phi_n = 0.30;

void setup() {
  size(640, 360);
  flock = new Flock();
  // Add an initial set of boids into the system
  for (int i = 0; i < n_boids; i++) {
    flock.addBoid(new Boid(width/2,height/2,i));
  }

  // debugging sort stuff
  // ArrayList<Event> tEvents = new ArrayList<Event>()
  // tEvents.add(new Event(0, new Boid(0,0,0), true));
  // tEvents.add(new Event(1, new Boid(1,1,1), true));
  // tEvents.add(new Event(2, new Boid(2,2,2), true));
  // tEvents.add(new Event(5, new Boid(5,5,5), true));
  // tEvents.add(new Event(3, new Boid(3,3,3), false));
  // tEvents.add(new Event(4, new Boid(4,4,4), true));

  // println(tEvents);
  // sort_events(tEvents);
  // println(tEvents);

  // debugging sort stuff
  // ArrayList<Boid> tBoids = new ArrayList<Boid>();
  // Boid b = new Boid(0,0,0);
  // tBoids.add(new Boid(5,5,5));
  // tBoids.add(new Boid(1,1,1));
  // tBoids.add(new Boid(2,2,2));
  // tBoids.add(new Boid(3,3,3));
  // tBoids.add(new Boid(4,4,4));

  // println(tBoids);
  // sort_neighbors(tBoids, b);
  // println(tBoids);

}

void draw() {
  // if (!mousePressed) return;
  // only execute stuff below on mouse click
  println("-------------# t = " + t);
  background(50);
  flock.run();
  t++;
}

// The Event Class

// class Event implements Comparable<Event> {
class Event {

  float theta;
  Boid boid;
  boolean start;
  Event (float t, Boid b, boolean s) {
    theta = t;
    boid = b;
    start = s; // first edge?
  }
  int compareTo(Event o) {
    if (theta < o.theta) return -1;
    if (theta > o.theta) return 1;
    return 0;
  }
  String toString() {
    return theta + " " + start;
  }

}

// The Flock (a list of Boid objects)

class Flock {

  ArrayList<Boid> boids; // An ArrayList for all the boids

  Flock() {
    boids = new ArrayList<Boid>(); // Initialize the ArrayList
  }

  void run() {
    for (Boid b : boids) {
      // println("Running boid # " + b.id);
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }
  }

  void addBoid(Boid b) {
    boids.add(b);
  }
}

// The Boid class

class Boid {

  PVector position;
  PVector velocity;
  PVector acceleration;
  float r;
  float maxforce;    // Maximum steering force
  float maxspeed;    // Maximum speed
  int id;

  // PriorityQueue<Boid> close_neighbors = new PriorityQueue<Boid>(4); // closest 4 neighbors
  ArrayList<Boid> close_neighbors = new ArrayList<Boid>(); // closest 4 neighbors
  // Boid pierced_boid = null;
  
  Boid(float x, float y, int num) {
    acceleration = new PVector(0, 0);
    id = num;

    // This is a new PVector method not yet implemented in JS
    // velocity = PVector.random2D();

    // Leaving the code temporarily this way so that this example runs in JS
    float angle = random(TWO_PI);
    velocity = new PVector(cos(angle), sin(angle));
    position = new PVector((float)x, (float)y);
    r = 2.0;
    maxspeed = 2;
    maxforce = 0.03;
  }

  String toString() {
    String s = "Boid #" + id + ": " + position.x + " " + position.y;
    return s;
  }

  void run(ArrayList<Boid> boids) {
    flock(boids);
    update();
    borders();
    render();
  }

  void applyForce(PVector force) {
    // We could add mass here if we want A = F / M
    acceleration.add(force);
  }

  
  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {

    ArrayList<Event> eventq = new ArrayList<Event>();
    ArrayList<Boid> status = new ArrayList<Boid>();
    ArrayList<Float> boundaries = new ArrayList<Float>();
    ArrayList<Boid> in_sight = new ArrayList<Boid>();       // line-of-sight neighbors
    boolean overlapping = false;
    
    // Create event queue based on other relationship with other boids
    for (Boid other : boids) {
      if (this == other) { continue; } // don't add self to queue
      float d = PVector.dist(position, other.position);
      if (d < r) {
        overlapping = true;
        break;
      }
      
      float dtheta = asin( (float) other.r/d );
      float rel_theta = atan2( other.position.y-position.y , other.position.x-position.x );
      eventq.add(new Event(rel_theta-dtheta, other, true));
      eventq.add(new Event(rel_theta+dtheta, other, false));
    }

    // ensure thetas are in range [0,2*PI)
    for (Event e : eventq) {
      if (e.theta < 0) {
        e.theta += 2*PI;
      }
    }

    // sort event queue by thetas
    // Collections.sort(eventq);
    eventq = sort_events(eventq);

    // cycle through eventq in order to determine boundaries and line-of-sight neighbors
    for (Event e : eventq) {
      if (!e.start) {
        if (status.size() == 1) {
          boundaries.add(e.theta);
          status.remove(e.boid);
        }
      } else {
        if (status.size() == 0) {
          boundaries.add(e.theta);
          status.add(e.boid);
        }
      }

      Boid pierced_boid = null;
      float smallest_distance = width + height;
      if (status.size() > 0) {
        for (Boid other : status) {
          float d = PVector.dist(position, other.position);          
          if (d < smallest_distance) {
            pierced_boid = other;
            smallest_distance = d;
          }
        }
        if (in_sight.indexOf(pierced_boid) == -1) {
          in_sight.add(pierced_boid);
        }
      }
    }
    
    // todo: reset range of boundaries?
    ArrayList<PVector> vect_boundaries = new ArrayList<PVector>();
    ArrayList<PVector> vect_neighbors = new ArrayList<PVector>();

    for (float theta : boundaries) {
      vect_boundaries.add(new PVector(cos(theta), sin(theta)));
    }

    // only use 4 closest line-of-sight neighbors
    // k closest neighbor
    // Collections.sort(in_sight, createComparator(this));
    in_sight = sort_neighbors(in_sight, this);
    
    close_neighbors.clear();
    for (int i = 0; i < min(4,in_sight.size()); i++) {
      close_neighbors.add(in_sight.get(i));
    }
    
    for (Boid b : close_neighbors) {
      vect_neighbors.add(new PVector(b.velocity.x, b.velocity.y));
    }

    PVector proj = average_direction(vect_boundaries);
    PVector alig = average_direction(vect_neighbors);;
    PVector nois = PVector.random2D();


    PVector predator = new PVector(mouseX, mouseY);
    float d_pred = PVector.dist(position, predator);

    if (d_pred < predator_radius) {
      PVector vect_escape = new PVector(position.x-predator.x, position.y-predator.y);
      vect_escape.mult(0.9);
      applyForce(vect_escape);
    } else if (!overlapping) {
      // Arbitrarily weight these forces
      proj.mult(phi_p);
      alig.mult(phi_a);
      nois.mult(phi_n);
      // Add the force vectors to acceleration
      applyForce(proj);
      applyForce(alig);
      applyForce(nois);
    } else {
      applyForce(nois); // all noise if vision is blocked (ie overlapped = true)
    }
  }

  // Method to update position
  void update() {
    velocity.add(acceleration); // Update velocity
    velocity.limit(maxspeed);   // Limit speed
    position.add(velocity);     // Update position
    acceleration.mult(0);       // Reset acceleration to 0 each cycle
  }

  void render() {
    // Draw a triangle rotated in the direction of velocity
    float theta = velocity.heading2D() + radians(90);
    // heading2D() above is now heading() but leaving old syntax until Processing.js catches up

    // text(str(id), position.x, position.y);
    
    fill(200, 100);
    stroke(255);
    pushMatrix();
    translate(position.x, position.y);
    rotate(theta);
    beginShape(TRIANGLES);
    vertex(0, -r*2);
    vertex(-r, r*2);
    vertex(r, r*2);
    endShape();
    popMatrix();
  }

  // Wraparound
  void borders() {
    if (position.x < -r) position.x = width+r;
    if (position.y < -r) position.y = height+r;
    if (position.x > width+r) position.x = -r;
    if (position.y > height+r) position.y = -r;
  }

}

PVector average_direction(ArrayList<PVector> vectors) {
  int n = vectors.size();
  float x = 0;
  float y = 0;
  for (PVector v : vectors) {
    x += v.x;
    y += v.y;
  }
  return new PVector(x/n , y/n);
}

// cannot use this with processing.js
// Comparator<Boid> createComparator(Boid b) {
//   final Boid finalB = new Boid(b.position.x, b.position.y, b.id);
//   return new Comparator<Boid>() {

//     public int compare(Boid b0, Boid b1) {
//       float ds0 = PVector.dist(finalB.position, b0.position);
//       float ds1 = PVector.dist(finalB.position, b1.position);
//       return Float.compare(ds0, ds1);
//     }
//   };
// }

ArrayList<Event> sort_events(ArrayList<Event> events) {
  int min;
  for (int i = 0; i < events.size(); ++i) {
    min = i;
    for (int j = events.size()-1; j > i; --j) {
      if (events.get(j).theta < events.get(min).theta) {
        min = j;
      }
    }
    Event tmp = events.get(i);
    events.set(i, events.get(min));
    events.set(min, tmp);
  }
  return events;
}

ArrayList<Boid> sort_neighbors(ArrayList<Boid> neighbors, Boid b) {
  int min;
  for (int i = 0; i < neighbors.size(); ++i) {
    min = i;
    for (int j = neighbors.size()-1; j > i; --j) {
      
      Boid b0 = neighbors.get(min);
      Boid b1 = neighbors.get(j);
      float ds0 = PVector.dist(b.position, b0.position);      
      float ds1 = PVector.dist(b.position, b1.position);
      
      if (ds1 < ds0) {
        min = j;
      }
    }
    Boid tmp = neighbors.get(i);
    neighbors.set(i, neighbors.get(min));
    neighbors.set(min, tmp);
  }
  return neighbors;

}

