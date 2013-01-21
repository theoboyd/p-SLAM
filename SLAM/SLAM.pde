/*
    EKFslam - A simulation of the Simultaneous Localisation and Mapping (SLAM)
              technique, as applied to an autonomous vehicle navigating through
              waypoints across a feature-abundant terrain.

    The simulator configuration, along with various parameters of the vehicle
    and sensors are set in at the top of this file. Also defined is a set of
    landmarks and vehicle waypoints for the desired vehicle path.

    New waypoints can be added (in nearest-distance order) by clicking the left
    mouse button. Existing waypoints can be moved around by holding the left
    mouse button down in their vicinity and then dragging them, or deleted by
    clicking with the right mouse button in their vicinity.

    Author: Roger Stuckey, 2010.
    Original Matlab version: Tim Bailey & Juan Nieto, 2004.
*/

float[][] landmarks;
float[][] waypoints;
float x_max, y_min, y_max, x_range, y_range;
int nf, nnf, nef;
ArrayList WP, LM;
Vehicle vehicle;

// Control parameters
float FWD_SPEED = 6.0;       // vehicle forward speed (m/s)
float MAXG = 30.0*PI/180.0;  // maximum steering angle (-MAXG < g < MAXG) (rad)
float RATEG = 20.0*PI/180.0; // maximum rate of change in steer angle (rad/s)
float WHEELBASE = 4.0;       // vehicle wheel-base (m)
float DT_CONTROLS = 0.025;   // time interval between control signals (s)

// Control noises
float sigmaV = 0.3;          // standard deviation of noise in speed (m/s)
float sigmaG = 3.0*PI/180.0; // standard deviation of noise in steering angle (rad)
Matrix Q = new Matrix({{ pow(sigmaV, 2.0), 0.0 }, { 0.0, pow(sigmaG, 2.0) }});
Matrix QE = Q.newcopy();

// Observation parameters
float MAX_RANGE = 30.0;             // maximum sensor range (m)
float DT_OBSERVE = 8.0*DT_CONTROLS; // time interval between observations (s)

// Observation noises
float sigmaR = 0.1;          // standard deviation in range (m)
float sigmaB = 1.0*PI/180.0; // standard deviation in bearing (rad)
Matrix R = new Matrix({{ pow(sigmaR, 2.0), 0.0 }, { 0.0, pow(sigmaB, 2.0) }});
Matrix RE = R.newcopy();

// Waypoint proximity
float AT_WAYPOINT = 1.0; // distance from current waypoint at which to switch to next waypoint (m)
int NUMBER_LOOPS = 2;    // number of loops through the waypoint list

// Drawing parameters
float M = 2.0; // landmark/waypoint/vehicle marker-size scale
float S = 2.0; // number of standard deviations for covariance ellipses


void setup() {
  /*
    set the window size & colormode.
  */
  size(600, 338);
  colorMode(RGB, 255, 255, 255, 100);

  // create the landmarks
  landmarks = {
    {    4.3542, -25.7009 },
    {   47.8735, -33.1776 },
    {   36.0870, -68.3801 },
    {  109.5258, -65.5763 },
    {  107.2592, -35.6698 },
    {  143.5253,   3.8941 },
    {   71.8998,  26.9470 },
    {  116.7790,  59.9688 },
    {   79.6063,  88.6293 },
    {   19.3140,  80.8411 },
    {  -23.7520,  49.0654 },
    {  -95.8308,  83.6449 },
    { -140.2568,  60.5919 },
    { -110.7906,  36.2928 },
    { -127.1103, -21.3396 },
    { -150.6833, -32.2430 },
    { -135.2702, -77.7259 },
    {  -81.3244, -55.9190 },
    {  -51.4049, -19.1589 },
    {  -81.7777,  16.3551 },
    {  -10.6056,  19.7819 },
    {   37.4470,   3.2710 },
    {  -28.4647,  80.1444 },
    {  -60.7545,  46.2325 },
    {   36.9646,  26.8543 },
    {  132.9842,  28.2385 },
    {  -19.9674, -79.0338 },
    {  -76.8993, -92.1833 },
    { -125.3340,  15.0890 },
    { -184.8152,  22.7018 },
    {  -37.8117,   4.7078 },
    {   82.0003, -17.4388 },
    {   75.2025, -83.1863 },
    {   31.0165, -96.3358 },
    {   70.1041,  57.9979 }
  };
  LM = new ArrayList();
  for (int i = 0; i < landmarks.length; i++) {
    LM.add(new Landmark(landmarks[i]));
  }
  nf = 0; // new feature index

  // create the waypoints
  waypoints = {
    {   18.4073, -41.5888 },
    {   65.0999, -54.9844 },
    {  124.4856, -45.0156 },
    {  136.2720, -17.2897 },
    {   94.5661,   5.7632 },
    {  104.5392,  31.6199 },
    {  103.6326,  70.5607 },
    {   48.7801,  76.4798 },
    {   18.8606,  51.5576 },
    {  -47.3250,  67.4455 },
    { -108.9773,  69.0031 },
    { -141.6168,  41.9003 },
    { -156.5765,   6.3863 },
    { -125.7503, -25.7009 },
    { -122.1237, -64.0187 },
    {  -57.2981, -81.4642 },
    {  -25.5653, -51.5576 }
  };
  WP = new ArrayList();
  for (int i = 0; i < waypoints.length; i++) {
    WP.add(waypoints[i]);
  }

  // create the vehicle
  vehicle = new Vehicle(0.0, 0.0, 0.0);

  // determine the data limits & offsets
  x_min = 0.0;
  x_max = 0.0;
  y_min = 0.0;
  y_max = 0.0;
  for (int i = 0; i < LM.size(); i++) {
    Landmark lm = (Landmark) LM.get(i);
    if (lm.x < x_min) { x_min = lm.x; }
    if (lm.x > x_max) { x_max = lm.x; }
    if (lm.y < y_min) { y_min = lm.y; }
    if (lm.y > y_max) { y_max = lm.y; }
  }
  x_range = x_max - x_min;
  y_range = y_max - y_min;
  x_min -= 0.1*x_range;
  x_max += 0.1*x_range;
  y_min -= 0.1*y_range;
  y_max += 0.1*y_range;
  x_range *= 1.2;
  y_range *= 1.2;

  // zero the seed for repeatability
  randomSeed(0);

  smooth();
}

void draw() {
  /*
    Draw the background.
  */
  background(0, 48, 160);
  // run the vehicle sim
  vehicle.run();
}

void mouseDragged() 
{
  /*
    Drag existing waypoints around.
  */
  for (int i = 0; i < WP.size(); i++) {
    float wp = (float) WP.get(i);
    float p_x = (wp[0] - x_min)*width/x_range;
    float p_y = height - (wp[1] - y_min)*height/y_range;
    if (sqrt(pow(p_x - mouseX, 2.0) + pow(p_y - mouseY, 2.0)) < 10.0) {
      wp[0] = mouseX*x_range/width + x_min;
      wp[1] = (height - mouseY)*y_range/height + y_min;
      WP.set(i, wp);
      break;
    }
  }
}

void mousePressed() {
  /*
    Respond to a button press.
  */
  if (mouseButton == RIGHT) {
    // delete the first, nearest waypoint
    for (int i = 0; i < WP.size(); i++) {
      float wp = (float) WP.get(i);
      float p_x = (wp[0] - x_min)*width/x_range;
      float p_y = height - (wp[1] - y_min)*height/y_range;
      if (sqrt(pow(p_x - mouseX, 2.0) + pow(p_y - mouseY, 2.0)) < 10.0) {
        if (i < vehicle.iwp) {
          vehicle.iwp -= 1;
        } else if (i == vehicle.iwp) {
          if (vehicle.iwp > WP.size() - 1) { // reached final waypoint
            vehicle.iwp = -1;
          }
        }
        WP.remove(i);
        break;
      }
    }
  } else if (mouseButton == LEFT) {
    // add a waypoint to the list
    int i_0, i_1, i_max;
    float wp, p_x, p_y, d, wp_x, wp_y;
    float d_0 = -1.0;
    float d_1 = -1.0;
    // determine the closest existing waypoint
    for (int i = 0; i < WP.size(); i++) {
      wp = (float) WP.get(i);
      p_x = (wp[0] - x_min)*width/x_range;
      p_y = height - (wp[1] - y_min)*height/y_range;
      d = sqrt(pow(p_x - mouseX, 2.0) + pow(p_y - mouseY, 2.0));
      if (d < 10.0) {
        return;
      }
      if ((d_0 < 0.0) || (d < d_0)) {
        i_0 = i;
        d_0 = d;
      }
    }
    // determine the next-closest existing waypoint
    for (int i = 0; i < WP.size(); i++) {
      if (i == i_0) {
        continue;
      }
      wp = (float) WP.get(i);
      p_x = (wp[0] - x_min)*width/x_range;
      p_y = height - (wp[1] - y_min)*height/y_range;
      d = sqrt(pow(p_x - mouseX, 2.0) + pow(p_y - mouseY, 2.0));
      if ((d_1 < 0.0) || (d < d_1)) {
        i_1 = i;
        d_1 = d;
      }
    }
    // add the new waypoint in order between the two existing
    i_max = max(i_0, i_1);
    wp_x = mouseX*x_range/width + x_min;
    wp_y = (height - mouseY)*y_range/height + y_min;
    WP.add(i_max, { wp_x, wp_y });
  }
}

class Landmark {
  /*
    Landmark class.
  */
  float x, y; // x, y coordinates
  float r, b; // range, bearing
  int[] ftag; // { invisible/visible, unseen/new/existing } feature tag
  int ida;    // data association index
  Landmark(float[] lm) {
    // Landmark constructor
    x = lm[0];
    y = lm[1];
    r = 0.0;
    b = 0.0;
    rn = 0.0;
    bn = 0.0;
    ftag = { 0, 0 };
    ida = -1;
  }
}

class Vehicle {
  /*
    Vehicle class.
  */
  float pos_x, pos_y, pos_phi;

  Matrix x, P, xtrue;       // estimated position, covariance & true position (x, y, phi)
  int iwp = 0;              // index to current waypoint
  int iloop = NUMBER_LOOPS; // index to current loop number
  float V = FWD_SPEED;      // vehicle forward speed
  float G = 0.0;            // current steering angle
  float dt = DT_CONTROLS;   // timestep
  float dtsum = 0.0;        // time since last EKF update
  float Vn, Gn;

  Matrix Gv;
  Matrix Gu;
  Matrix temp_1, temp_2, temp_3, temp_4, temp_5, temp_6, temp_7, temp_8;
  Matrix temp_9, temp_10, temp_11;
  Matrix vv, RR, H;
  Matrix temp_12, temp_13, temp_14, temp_15, temp_16, temp_17;
  Matrix SChol, SCholInv, W1, W;
  Matrix temp_18, temp_19, temp_20, temp_21;
  Matrix gv;
  Matrix gz;
  Matrix temp_22, temp_23, temp_24, temp_25, temp_26, temp_27, temp_28, temp_29;
  Matrix temp_30, temp_31, temp_32, temp_33, temp_34;
  Matrix pp;
  Matrix rr;
  Matrix path;

  int pathlen;

  Vehicle(float px, float py, float pphi) {
  /*
    Vehicle constructor.
  */
    pos_x = px;
    pos_y = px;
    pos_phi = pphi;

    int nlm = LM.size();

    x = new Matrix(nlm + 3, 1);
    x.resize(3, 1);
    P = new Matrix(nlm + 3, nlm + 3);
    P.resize(3, 3);

    Vn = V;
    Gn = G;

    // Create a bunch of static matrices for manipulation
    Gv = new Matrix({{ 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 }});
    Gu = new Matrix({{ 0.0, 0.0 }, { 0.0, 0.0 }, { 0.0, 0.0 }});

    temp_1 = new Matrix(3, 3);
    temp_2 = new Matrix(3, 3);
    temp_3 = new Matrix(3, 3);
    temp_4 = new Matrix(3, 3);
    temp_5 = new Matrix(3, 3);
    temp_6 = new Matrix(3, 3);
    temp_7 = new Matrix(3, 3);
    temp_8 = new Matrix(3, 3);

    temp_9 = new Matrix(3, nlm);
    temp_10 = new Matrix(3, nlm);
    temp_11 = new Matrix(nlm, 3);

    vv = new Matrix(10, 1);
    RR = new Matrix(10, 10);
    H = new Matrix(10, nlm + 3);

    temp_12 = new Matrix(nlm + 3, 10);
    temp_13 = new Matrix(nlm + 3, nlm + 3);
    temp_14 = new Matrix(10, 10);
    temp_15 = new Matrix(10, 10);
    temp_16 = new Matrix(10, 10);
    temp_17 = new Matrix(10, 10);
    
    SChol = new Matrix(10, 10);
    SCholInv = new Matrix(10, 10);

    W1 = new Matrix(nlm + 3, 10);
    temp_18 = new Matrix(10, nlm + 3);
    W = new Matrix(nlm + 3, 10);

    temp_19 = new Matrix(nlm + 3, 1);
    temp_20 = new Matrix(10, nlm + 3);
    temp_21 = new Matrix(nlm + 3, nlm + 3);

    gv = new Matrix({{ 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }});
    gz = new Matrix({{ 0.0, 0.0 }, { 0.0, 0.0 }});

    temp_22 = new Matrix(3, 3);
    temp_23 = new Matrix(2, 3);
    temp_24 = new Matrix(3, 2);
    temp_25 = new Matrix(2, 2);
    temp_26 = new Matrix(2, 2);
    temp_27 = new Matrix(2, 2);
    temp_28 = new Matrix(2, 2);
    temp_29 = new Matrix(2, 2);

    temp_30 = new Matrix(2, 3);
    temp_31 = new Matrix(3, 2);
    temp_32 = new Matrix(3, nlm);
    temp_33 = new Matrix(2, nlm);
    temp_34 = new Matrix(nlm, 2);

    pp = new Matrix(2, 2);
    rr = new Matrix(2, 2);

    path = new Matrix(1000, 3);
    pathlen = 0;
  }

  void run() {
  /*
    Update the vehicle and render to screen.
  */
    update();
    render();
  }

  void update() {
  /*
    Update the vehicle true & estimated pose and covariance.
  */
    // compute the steering control
    compute_steering();

    // perform loops: if final waypoint reached, go back to first
    if (iwp == -1) {
      iwp = 0;
      if (iloop > 1) {
        iloop -= 1;
      } else {
        noLoop();
      }
    }
    // compute the vehicle response
    vehicle_model();

    // add random noise to nominal control values
    add_control_noise();

    // EKF predict step
    predict();

    dtsum += dt;
    if (dtsum < DT_OBSERVE)
      return;
    dtsum = 0.0;

    // compute exact observation for each visible landmark
    get_observations();

    // add random noise to observations
    add_observation_noise();

    // maintain the feature/observation lookup table
    data_associate_known();

    // EKF update step
    batch_update();

    // augment the state and covariance with any new observations
    augment();

    // add the estimated pose to the vehicle's path history
    store_data();
  }

  void render() {
  /*
    Draw the landmarks, waypoints, estimated and true vehicle pose.
  */
    float wp, p_x, p_y, theta;

    // draw all landmarks
    for (int i = 0; i < LM.size(); i++) {
      Landmark lm = (Landmark) LM.get(i);
      p_x = (lm.x - x_min)*width/x_range;
      p_y = height - (lm.y - y_min)*height/y_range;
      stroke(32, 192, 32);
      ellipse(p_x, p_y, M, M);
    }

    // draw all waypoints and outline the next waypoint
    for (int i = 0; i < WP.size(); i++) {
      wp = (float) WP.get(i);
      p_x = (wp[0] - x_min)*width/x_range;
      p_y = height - (wp[1] - y_min)*height/y_range;
      if (i == iwp) {
        stroke(255, 0, 0);
      } else {
        noStroke();
      }
      fill(192, 96, 0);
      triangle(p_x - M*2.0, p_y + M*1.3, p_x, p_y - M*2.7, p_x + M*2.0, p_y + M*1.3);
    }

    // draw markers & covariance ellipses for each observation
    noFill();
    for (int i = 0; i < LM.size(); i++) {
      Landmark lm = (Landmark) LM.get(i);
      if (lm.ida < 0) {
          continue;
      }
      int fpos = 3 + lm.ida*2; // feature position in the state vector
      p_x = (x.data[fpos][0] - x_min)*width/x_range;
      p_y = height - (x.data[fpos + 1][0] - y_min)*height/y_range;
      if (lm.ftag[0] == 1) {
        stroke(255);
      } else {
        stroke(224);
      }
      // draw the observation markers
      beginShape(LINES);
      vertex(p_x - M, p_y);
      vertex(p_x + M, p_y);
      vertex(p_x, p_y - M);
      vertex(p_x, p_y + M);
      endShape();

      // draw the covariance ellipses
//      pp = P.newpcopy(fpos, fpos + 1, fpos, fpos + 1);
//      rr = pp.newsqrtm22();
      pp.thispcopy(P, fpos, fpos + 1, fpos, fpos + 1);
      rr.thissqrtm22(pp);
      drawEllipse(fpos);
    }

    // draw the vehicle path
    stroke(255);
    noFill();
    beginShape(LINES);
    for (int i = 0; i < pathlen; i++) {
      p_x = (path.data[i][0] - x_min)*width/x_range;
      p_y = height - (path.data[i][1] - y_min)*height/y_range;
      vertex(p_x, p_y);
    }
    endShape();

    // draw the estimated vehicle pose
    p_x = (x.data[0][0] - x_min)*width/x_range;
    p_y = height - (x.data[1][0] - y_min)*height/y_range;
    theta = -(x.data[2][0] + radians(90.0));

    pushMatrix();
    translate(p_x, p_y);
    rotate(theta);
    stroke(192);
    noFill();
    drawVehicle_bow();
    drawVehicle_mid();
    drawVehicle_fins();
    drawVehicle_stern();
    popMatrix();

    // draw confidence ellipse of the vehicle
    stroke(224);
    noFill();
//    pp = P.newpcopy(0, 1, 0, 1);
//    rr = pp.newsqrtm22();
    pp.thispcopy(P, 0, 1, 0, 1);
    rr.thissqrtm22(pp);
    drawEllipse(0);

    // draw the actual vehicle pose
    p_x = (pos_x - x_min)*width/x_range;
    p_y = height - (pos_y - y_min)*height/y_range;
    theta = -(pos_phi + radians(90.0));

    pushMatrix();
    translate(p_x, p_y);
    rotate(theta);
    noStroke();
    fill(0);
    drawVehicle_bow();
    fill(224, 224, 0);
    drawVehicle_mid();
    fill(192, 96, 0);
    drawVehicle_fins();
    fill(0);
    drawVehicle_stern();

    // draw the sensor detection area
    noStroke();
    fill(224, 35);
    arc(0, 0, MAX_RANGE*2.0*width/x_range, MAX_RANGE*2.0*height/y_range, 0.0, PI);
    popMatrix();

    // display the current frame rate
    fill(224);
    text(int(frameRate) + " FPS", 560, 332);
  }

  void drawEllipse(int idx) {
    /*
      Draw an ellipse using the matrix rr.
    */
    beginShape();
    for (int k = 0; k < 10; k++) {
      float phi = k/10.0*2.0*PI;
      float c = cos(phi);
      float s = sin(phi);
      p_x = (x.data[idx][0] + S*(rr.data[0][0]*c + rr.data[0][1]*s) - x_min)*width/x_range;
      p_y = height - (x.data[idx + 1][0] + S*(rr.data[1][0]*c + rr.data[1][1]*s) - y_min)*height/y_range;
      vertex(p_x, p_y);
    }
    endShape(CLOSE);
  }

  void drawVehicle_bow() {
    /*
      Draw the vehicle's bow.
    */
    beginShape();
    vertex(M*0.5, M*4.0);
    vertex(M, M*3.0);
    vertex(M, M*2.0);
    vertex(-M, M*2.0);
    vertex(-M, M*3.0);
    vertex(-M*0.5, M*4.0);
    endShape(CLOSE);
  }

  void drawVehicle_mid() {
    /*
      Draw the vehicle's mid-section.
    */
    beginShape();
    vertex(-M, M*2.0);
    vertex(M, M*2.0);
    vertex(M, -M*1.0);
    vertex(-M, -M*1.0);
    endShape(CLOSE);
  }

  void drawVehicle_fins() {
    /*
      Draw the vehicle's fins.
    */
    beginShape();
    vertex(0, -M*2.0);
    vertex(M*1.5, -M*2.5);
    vertex(M*1.5, -M*3.0);
    vertex(-M*1.5, -M*3.0);
    vertex(-M*1.5, -M*2.5);
    endShape(CLOSE);
  }

  void drawVehicle_stern() {
    /*
      Draw the vehicle's stern.
    */
    beginShape();
    vertex(-M, -M*1.0);
    vertex(M, -M*1.0);
    vertex(0, -M*4.0);
    endShape(CLOSE);
  }

  void compute_steering() {
  /* 
    Compute the steering angle required to direct the vehicle toward the next
    waypoint.
  */
    float wp = (float) WP.get(iwp);

    // determine if current waypoint reached
    float d2 = pow(wp[0] - pos_x, 2.0) + pow(wp[1] - pos_y, 2.0);

    if (d2 < pow(AT_WAYPOINT, 2.0)) {
      iwp += 1; // switch to next
      if (iwp > WP.size() - 1) { // reached final waypoint, flag and return
        iwp = -1;
        return;
      }
    }

    // compute change in G to point towards current waypoint
    float deltaG = pi_to_pi(atan2(wp[1] - pos_y, wp[0] - pos_x) - pos_phi - G);

    // limit steering rate
    float maxDelta = RATEG*dt;
    if (abs(deltaG) > maxDelta) {
      float deltaG = sign(deltaG)*maxDelta;
    }

    // limit steering angle
    G = G + deltaG;
    if (abs(G) > MAXG) {
      G = sign(G)*MAXG;
    }
  }

  void vehicle_model() {
  /*
    Calculate the new vehicle pose based on a constant velocity model.
  */
    pos_x += V*dt*cos(G + pos_phi);
    pos_y += V*dt*sin(G + pos_phi);
    pos_phi = pi_to_pi(pos_phi + V*dt*sin(G)/WHEELBASE);
  }

  void add_control_noise() {
  /*
    Add random noise to nominal control values. We assume Q is diagonal.
  */
    Vn = V + randn()*sqrt(Q.data[0][0]);
    Gn = G + randn()*sqrt(Q.data[1][1]);
  }

  void predict() {
  /*
    Predict the vehicle state and covariance.
  */
    float s = sin(Gn + x.data[2][0]);
    float c = cos(Gn + x.data[2][0]);
    float vts = Vn*dt*s;
    float vtc = Vn*dt*c;

    // construct the Jacobians
//    Matrix Gv = new Matrix({{ 1, 0, -vts }, { 0, 1, vtc }, { 0, 0, 1 }});
//    Matrix Gu = new Matrix({{ dt*c, -vts }, { dt*s, vtc }, { dt*sin(Gn)/WHEELBASE, Vn*dt*cos(Gn)/WHEELBASE }});

    Gv.data[0][2] = -vts;
    Gv.data[1][2] = vtc;

    Gu.data[0][0] = dt*c;
    Gu.data[0][1] = -vts;
    Gu.data[1][0] = dt*s;
    Gu.data[1][1] = vtc;
    Gu.data[2][0] = dt*sin(Gn)/WHEELBASE;
    Gu.data[2][1] = Vn*dt*cos(Gn)/WHEELBASE;

    // predict covariance
//    Matrix P1313 = Gv.newprod(P.newpcopy(0, 2, 0, 2)).newprod(Gv.newtrans()).newsum( Gu.newprod(QE).newprod(Gu.newtrans()) );
//    P.replace(P1313, 0, 0);

    temp_1.thispcopy(P, 0, 2, 0, 2);
    temp_2.thisprod(Gv, temp_1);
    temp_3.thistrans(Gv);
    temp_4.thisprod(temp_2, temp_3);

    temp_5.thisprod(Gu, QE);
    temp_6.thistrans(Gu);
    temp_7.thisprod(temp_5, temp_6);

    temp_8.thissum(temp_4, temp_7);
    P.replace(temp_8, 0, 0);

    if (P.rows > 3) {
//      Matrix P134e = Gv.newprod(P.newpcopy(0, 2, 3, P.cols - 1));
//      P.replace(P134e, 0, 3);
//      P.replace(P134e.newtrans(), 3, 0);
      temp_9.thispcopy(P, 0, 2, 3, P.cols - 1);
      temp_10.thisprod(Gv, temp_9);
      P.replace(temp_10, 0, 3);
      temp_11.thistrans(temp_10);
      P.replace(temp_11, 3, 0);
    }

    // predict state
    x.data[0][0] += vtc;
    x.data[1][0] += vts;
    x.data[2][0] = pi_to_pi(x.data[2][0] + Vn*dt*sin(Gn)/WHEELBASE);
  }

  void get_observations() {
  /*
    Select the set of landmarks that are visible within vehicle's field-of-view
    and compute exact observation for each.
  */
    for (int i = 0; i < LM.size(); i++) {
      Landmark lm = (Landmark) LM.get(i);
      float dx = lm.x - pos_x;
      float dy = lm.y - pos_y;
      float phi = pos_phi;

      // incremental tests for bounding semi-circle
      if ( (abs(dx) < MAX_RANGE) && (abs(dy) < MAX_RANGE) && ((dx*cos(phi) + dy*sin(phi)) > 0) && ((pow(dx, 2.0) + pow(dy, 2.0)) < pow(MAX_RANGE, 2.0)) ) {
        // Note: the bounding box test is unnecessary but illustrates a possible
        // speedup technique as it quickly eliminates distant points. Ordering
        // the landmark set would make this operation O(logN) rather that O(N)
        lm.ftag[0] = 1;
        lm.r = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
        lm.b = atan2(dy, dx) - phi;
        lm.rn = lm.r;
        lm.bn = lm.b;
      } else {
        lm.ftag[0] = 0;
      }
      LM.set(i, lm);
    }
  }

  void add_observation_noise() {
  /*
    Add random measurement noise. We assume R is diagonal.
  */
    for (int i = 0; i < LM.size(); i++) {
      Landmark lm = (Landmark) LM.get(i);
      if (lm.ftag[0] > 0) { // visible feature
        lm.rn = lm.r + randn()*sqrt(R.data[0][0]);
        lm.bn = lm.b + randn()*sqrt(R.data[1][1]);
        LM.set(i, lm);
      }
    }
  }

  void data_associate_known() {
  /*
    For simulations with known data-associations, this function maintains a
    feature/observation lookup table. It updates the data-association table,
    the set of associated observations and the set of observations to new
    features.
  */
    nnf = 0;
    nef = 0;
    for (int i = 0; i < LM.size(); i++) {
      Landmark lm = (Landmark) LM.get(i);
      if (lm.ftag[0] > 0) { // visible feature
        if (lm.ida > -1) { // existing association
          lm.ftag[1] = 2;
          nef += 1;
        } else { // new feature
          lm.ftag[1] = 1;
          nnf += 1;
          lm.ida = nf;
          LM.set(i, lm);
          nf += 1;
        }
        LM.set(i, lm);
      }
    }
  }

  void batch_update() {
  /*
    Given a feature index (ie, the order of the feature in the state vector),
    predict the expected range-bearing observation of this feature and its
    Jacobian.
  */
//    Matrix vv = new Matrix(2*nef, 1);
//    Matrix RR = new Matrix(2*nef, 2*nef);
//    Matrix H = new Matrix(2*nef, x.rows);

    if (nef > 0) {
      vv.resize(2*nef, 1);
      RR.resize(2*nef, 2*nef);
      H.resize(2*nef, x.rows);
    }
    for (int i = 0; i < 2*nef; i++) {
      for (int j = 0; j < x.rows; j++) {
        H.data[i][j] = 0.0;
      }
    }

    int j = 0;
    for (int i = 0; i < LM.size(); i++) {
      Landmark lm = (Landmark) LM.get(i);
      if ((lm.ftag[0] == 1) && (lm.ftag[1] == 2)) { // visible, existing association
        int fpos = 3 + lm.ida*2;
        // auxiliary values
        float dx = x.data[fpos][0] - x.data[0][0];
        float dy = x.data[fpos + 1][0] - x.data[1][0];
        float d2 = dx*dx + dy*dy;
        float d = sqrt(d2);
        float xd = dx/d;
        float yd = dy/d;
        float xd2 = dx/d2;
        float yd2 = dy/d2;

        // predict z
        float zp1 = d;
        float zp2 = atan2(dy, dx) - x.data[2][0];

        // innovations
        vv.data[2*j][0] = lm.rn - zp1;
        vv.data[2*j + 1][0] = pi_to_pi(lm.bn - zp2);

        RR.replace(RE, 2*j, 2*j);

        // Jacobian sub-matrix
        H.data[2*j][0] = -xd;
        H.data[2*j][1] = -yd;
        H.data[2*j][2] = 0.0;
        H.data[2*j + 1][0] = yd2;
        H.data[2*j + 1][1] = -xd2;
        H.data[2*j + 1][2] = -1.0;

        H.data[2*j][fpos] = xd;
        H.data[2*j][fpos + 1] = yd;
        H.data[2*j + 1][fpos] = -yd2;
        H.data[2*j + 1][fpos + 1] = xd2;

        j++;
      }
    }
    if (nef > 0) {
      kf_cholesky_update(vv, RR, H);
    }
  }

  void kf_cholesky_update(vv, RR, H) {
  /*
    Calculate the KF (or EKF) update given the prior state [x, P] the
    innovation [v, R] and the (linearised) observation model H. The result is
    calculated using Cholesky factorisation, which is more numerically stable
    than a naive implementation.
  */

//    Matrix PHt = P.newprod(H.newtrans()); // (x.rows, x.rows)*(x.rows, 2*nef) = (x.rows, 2*nef)
    temp_12.thistrans(H);
    temp_13.thisprod(P, temp_12);
//    Matrix S = H.newprod(PHt).newsum(RR); // (2*nef, x.rows)*(x.rows, 2*nef) + (2*nef, 2*nef) = (2*nef, 2*nef)
    temp_14.thisprod(H, temp_13);
    temp_15.thissum(temp_14, RR);

//    Matrix SS = S.newsum(S.newtrans()); // make symmetric (2*nef, 2*nef)
//    SS.multf(0.5);
    temp_16.thistrans(temp_15);
    temp_17.thissum(temp_15, temp_16);
    temp_17.multf(0.5);

//    Matrix SChol = SS.newchol("upper"); // (2*nef, 2*nef)
    SChol.thischol(temp_17, "upper"); // (2*nef, 2*nef)
//    Matrix SCholInv = SChol.newcholinv("upper"); // triangular matrix (2*nef, 2*nef)
    SCholInv.thischolinv(SChol, "upper"); // triangular matrix (2*nef, 2*nef)

//    Matrix W1 = PHt.newprod(SCholInv); // (x.rows, 2*nef)*(2*nef, 2*nef) = (x.rows, 2*nef)
    W1.thisprod(temp_13, SCholInv); // (x.rows, 2*nef)*(2*nef, 2*nef) = (x.rows, 2*nef)
//    Matrix W = W1.newprod(SCholInv.newtrans()); // (x.rows, 2*nef)*(2*nef, 2*nef) = (x.rows, 2*nef)
    temp_18.thistrans(SCholInv);
    W.thisprod(W1, temp_18); // (x.rows, 2*nef)*(2*nef, 2*nef) = (x.rows, 2*nef)

//    x.add(W.newprod(vv)); // update
    temp_19.thisprod(W, vv);
    x.add(temp_19);
//    Matrix Wtemp = W1.newprod(W1.newtrans()); // (x.rows, 2*nef)*(2*nef, x.rows) = (x.rows, x.rows)
//    Wtemp.multf(-1.0);
    temp_20.thistrans(W1);
    temp_21.thisprod(W1, temp_20);
    temp_21.multf(-1.0);
//    P.add(Wtemp);
    P.add(temp_21);
  }

  void augment() {
  /*
    Augment the state and covariance. Note: We assume the number of vehicle
    pose states is three and; Only one value for R is used, as all measurements
    are assumed to have same noise properties.
  */
//    Matrix gv = new Matrix(2, 3);
//    Matrix gz = new Matrix(2, 2);

//    Matrix p1313 = P.newpcopy(0, 2, 0, 2);
    temp_22.thispcopy(P, 0, 2, 0, 2);

    // add new features to state
    for (int i = 0; i < LM.size(); i++) {
      Landmark lm = (Landmark) LM.get(i);
      if (lm.ftag[1] == 1) { // new feature
        float s = sin(x.data[2][0] + lm.bn);
        float c = cos(x.data[2][0] + lm.bn);

        // augment x
        int xrows = x.rows;
        x.resize(xrows + 2, 1);
        x.data[xrows][0] = x.data[0][0] + lm.rn*c;
        x.data[xrows + 1][0] = x.data[1][0] + lm.rn*s;

        // Jacobians
        gv.data[0][0] = 1.0;
        gv.data[0][2] = -lm.rn*s;
        gv.data[1][1] = 1.0;
        gv.data[1][2] = lm.rn*c;

        gz.data[0][0] = c;
        gz.data[0][1] = -lm.rn*s;
        gz.data[1][0] = s;
        gz.data[1][1] = lm.rn*c;

        // augment P
        P.resize(xrows + 2, xrows + 2);
//        Matrix Plmlm = gv.newprod(p1313).newprod(gv.newtrans()).newsum( gz.newprod(R).newprod(gz.newtrans()) );
//        Matrix Plmlm = gv.newprod(temp_22).newprod(gv.newtrans()).newsum( gz.newprod(R).newprod(gz.newtrans()) );
//        P.replace(Plmlm, xrows, xrows);

        temp_23.thisprod(gv, temp_22);
        temp_24.thistrans(gv);
        temp_25.thisprod(temp_23, temp_24);

        temp_26.thisprod(gz, R);
        temp_27.thistrans(gz);
        temp_28.thisprod(temp_26, temp_27);

        temp_29.thissum(temp_25, temp_28);
        P.replace(temp_29, xrows, xrows);

//        Matrix Plm13 = gv.newprod(p1313);
//        P.replace(Plm13, xrows, 0);
//        P.replace(Plm13.newtrans(), 0, xrows);
        temp_30.thisprod(gv, temp_22);
        P.replace(temp_30, xrows, 0);
        temp_31.thistrans(temp_30);
        P.replace(temp_31, 0, xrows);
        
        if (xrows > 3) {
//          Matrix Plm3e = gv.newprod(P.newpcopy(0, 2, 3, xrows - 1));
//          P.replace(Plm3e, xrows, 3);
//          P.replace(Plm3e.newtrans(), 3, xrows);
          temp_32.thispcopy(P, 0, 2, 3, xrows - 1);
          temp_33.thisprod(gv, temp_32);
          P.replace(temp_33, xrows, 3);
          temp_34.thistrans(temp_33);
          P.replace(temp_34, 3, xrows);
        }
      }
    }
  }

  void store_data() {
  /*
    Add the estimated pose to the vehicle's path history.
  */
    if (pathlen >= path.rows) {
      // increase the path length, if necessary
      path.resize(path.rows + 1000, 3);
    }
    // update the estimated vehicle path
    path.data[pathlen][0] = x.data[0][0];
    path.data[pathlen][1] = x.data[1][0];
    path.data[pathlen][2] = x.data[2][0];
    pathlen++;
  }
}

float sign(float x_) {
/*
  Calculate the signum (sign) function.
*/
  return (x_ < 0.0) ? -1.0 : 1.0;
}

float pi_to_pi(float angle_) {
/*
  Constrain an angle to +/-90 deg.
*/
  angle_ = angle_ % (2.0*PI);

  if (angle_ > PI) {
    angle_ -= 2.0*PI;
  } else if (angle_ < -PI) {
    angle_ += 2.0*PI;
  }

  return angle_;
}

float randn() {
/*
  Generate a random variable with normal distribution.

  This function uses the first form of the Box-Muller transform to generate
  a random variable with nornmal distribution from one with a uniform
  distribution.
*/
  float u1_ = random(1.0);
  float u2_ = random(1.0);

  float R_ = sqrt(-2.0*log(u1_));
  float theta_ = 2.0*PI*u2_;

  float z0_ = R_*cos(theta_);
//  float z1_ = R_*sin(theta_);

  return(z0_);
}
