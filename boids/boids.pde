// Objects
Flock flock;
Predator predator;

// Simulation Parameters
int n_boids = 4;
boolean debugging = true;
boolean physics = false;
float predator_radius = 20;

// debugging stuff
int step = 0;

// control parameters (sum to 1)
float phi_p = 0.30;
float phi_a = 0.60;
float phi_n = 0.10;

void setup() {
  size(640, 360);
  flock = new Flock();
  predator = new Predator();
  // Add an initial set of boids into the system
  for (int i = 0; i < n_boids; i++) {
    // flock.addBoid(new Boid(width/2,height/2,i));
    flock.addBoid(new Boid(random(width), random(height), i));
  }
}

void draw() {
  if (debugging) println("-------------# steps = " + step);
  // if (!mousePressed) return; // only execute stuff below on mouse click
  background(50);
  predator.run();
  flock.run();
  step++;
}

// The Event Class
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
      b.run(boids);  // Passing the entire list of boids to each boid individually
    }
  }
  void addBoid(Boid b) {
    boids.add(b);
  }
}

// The Predator class
class Predator {
  PVector position = new PVector();
  Predator() {
  }
  void run() {
    position.x = mouseX;
    position.y = mouseY;
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

  ArrayList<Boid> close_neighbors = new ArrayList<Boid>(); // closest 4 neighbors
  
  Boid(float x, float y, int num) {
    acceleration = new PVector(0, 0);
    id = num;
    // velocity = PVector.random2D(); // not implementing in processing.js
    float angle = random(TWO_PI);
    velocity = new PVector(cos(angle), sin(angle));
    position = new PVector((float)x, (float)y);
    r = 2.0;
    maxspeed = 2;
    // maxforce = 0.03;
    maxforce = 0.09;
  }

  String toString() {
    // String s = "Boid #" + id + ": " + "(" + position.x + "," + position.y + ")" + ;
    String s = "Boid #" + id + ": " + "(" + position + ")";
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


  void radial_scan() {
    // create event queue
    
  }

  // We accumulate a new acceleration each time based on three rules
  void flock(ArrayList<Boid> boids) {
    ArrayList<Event> eventq = new ArrayList<Event>();
    ArrayList<Boid> status = new ArrayList<Boid>();
    ArrayList<Float> boundaries = new ArrayList<Float>();
    ArrayList<Boid> in_sight = new ArrayList<Boid>();       // line-of-sight neighbors
    boolean overlapping = false;

    radial_scan(); // todo
    
    // Create event queue based on other relationship with other boids
    for (Boid other : boids) {
      if (this == other) { continue; } // don't add self to queue

      float d = PVector.dist(position, other.position);
      float dx = other.position.x - position.x;
      float dy = other.position.y - position.y;

      float dtheta = asin( other.r/d );
      float rel_theta = atan2( dy , dx );

      if (d < r || Float.isNaN(dtheta) || Float.isNaN(rel_theta)) {
        overlapping = true;
        break;
      }
      
      eventq.add(new Event(rel_theta-dtheta, other, true));
      eventq.add(new Event(rel_theta+dtheta, other, false));
    }

    // sort event queue by thetas
    if (!overlapping) {
    // if (eventq.size() > 0) {
      // Sorting.sort_events(eventq);
      Sorting.sort_events_merge(eventq, 0, eventq.size()-1);
    }

    // cycle through eventq in order to determine boundaries and line-of-sight neighbors
    for (Event e : eventq) {
      if (!e.start) {
        if (status.size() == 1) {
          boundaries.add(e.theta);
          status.remove(e.boid);
        }
      } else {
        if (status.isEmpty()) {
        // if (status.size() == 0) {
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
    // Collections.sort(in_sight, createComparator(this));
    in_sight = Sorting.sort_neighbors(in_sight, this);
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

    PVector predator_vectory = new PVector(mouseX, mouseY);
    float d_pred = PVector.dist(position, predator_vectory);

    // determine desired direction
    PVector steer = new PVector(0,0);
    if (d_pred < predator_radius) {
      PVector vect_escape = new PVector(position.x-predator_vectory.x, position.y-predator_vectory.y);
      steer.add(vect_escape);
      // applyForce(vect_escape);
      // return;
    } else if (!overlapping) {
      // Arbitrarily weight these forces
      proj.mult(phi_p);
      alig.mult(phi_a);
      nois.mult(phi_n);

      steer.add(proj);
      steer.add(alig);
      steer.add(nois);
    } else {
      steer.add(nois); // all noise if vision is blocked (ie overlapped = true)
    }

    if (!physics)  {
      applyForce(steer);
      return;
    }
    
    // Implement Reynolds: Steering = Desired - Velocity
    steer.normalize();
    steer.mult(maxspeed);
    steer.sub(velocity);
    steer.limit(maxforce);
    applyForce(steer);
  }

  PVector flee(PVector target) {
    PVector desired = PVector.sub(position, target); // A vector pointing from the target to the position
    // Scale to maximum speed
    desired.normalize();
    desired.mult(maxspeed); // desired.setMag(maxspeed); // not implemented in Processing.js

    // Steering = Desired minus Velocity
    PVector steer = PVector.sub(desired, velocity);
    steer.limit(maxforce);
    return steer;
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
    float theta = velocity.heading2D() + radians(90); // works with processing.js (now heading() in processing)

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
  PVector result = new PVector(0,0);
  for (PVector v : vectors) {
    result.x += v.x;
    result.y += v.y;
  }
  // return result.div(vectors.size());
  return result.normalize();
}

// Java Collections not available in Processing.js
static class Sorting {

  /* Brute force sorting for Events ArrayList */
  static void sort_events(ArrayList<Event> events) {
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
  }

  /* Brute force sorting for Boids ArrayList */
  static ArrayList<Boid> sort_neighbors(ArrayList<Boid> neighbors, Boid b) {
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

  // #################################################
  // Merge Sort
  // #################################################

  static void sort_events_merge(ArrayList<Event> events, int l, int r) {
    if (l < r) {
      int m = (l+r)/2;
      sort_events_merge(events, l, m);
      sort_events_merge(events, m+1, r);
      merge(events, l, m, r);
    }
  }

  static void merge(ArrayList<Event> events, int l, int m, int r) {
    ArrayList<Event> temp = new ArrayList<Event>();
    temp = events;
    int i = l;
    int j = m + 1;
    int k = l;
    while (i <= m && j <= r) {
      if (temp.get(i).theta <= temp.get(j).theta) {
        events.set(k, temp.get(i));
        i++;
      } else {
        events.set(k, temp.get(j));
        j++;
      }
      k++;
    }
    while (i <= m) {
      events.set(k, temp.get(i));
      k++;
      i++;
    }
  }
}

