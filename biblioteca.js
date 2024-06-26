/*
Repo: https://github.com/bmoren/p5.collide2D/
Created by http://benmoren.com
Some functions and code modified version from http://www.jeffreythompson.org/collision-detection
Version v0.7.3 | June 22, 2020
CC BY-NC-SA 4.0
*/

p5.prototype._collideDebug = false;

// Function to enable debug mode
p5.prototype.collideDebug = function(debugMode) {
  this._collideDebug = debugMode;
};

/*~++~+~+~++~+~++~++~+~+~ 2D ~+~+~++~+~++~+~+~+~+~+~+~+~+~+~+~*/

// Check collision between two rectangles
p5.prototype.collideRectRect = function(x, y, w, h, x2, y2, w2, h2) {
  return (x + w >= x2 && x <= x2 + w2 && y + h >= y2 && y <= y2 + h2);
};

// Check collision between two rectangles using p5.Vector
p5.prototype.collideRectRectVector = function(p1, sz1, p2, sz2) {
  return this.collideRectRect(p1.x, p1.y, sz1.x, sz1.y, p2.x, p2.y, sz2.x, sz2.y);
};

// Check collision between rectangle and circle
p5.prototype.collideRectCircle = function(rx, ry, rw, rh, cx, cy, diameter) {
  let testX = cx;
  let testY = cy;

  if (cx < rx) testX = rx; 
  else if (cx > rx + rw) testX = rx + rw;
  if (cy < ry) testY = ry;
  else if (cy > ry + rh) testY = ry + rh;

  const distance = this.dist(cx, cy, testX, testY);
  return (distance <= diameter / 2);
};

// Check collision between rectangle and circle using p5.Vector
p5.prototype.collideRectCircleVector = function(r, sz, c, diameter) {
  return this.collideRectCircle(r.x, r.y, sz.x, sz.y, c.x, c.y, diameter);
};

// Check collision between two circles
p5.prototype.collideCircleCircle = function(x, y, d, x2, y2, d2) {
  return (this.dist(x, y, x2, y2) <= (d / 2) + (d2 / 2));
};

// Check collision between two circles using p5.Vector
p5.prototype.collideCircleCircleVector = function(p1, d1, p2, d2) {
  return this.collideCircleCircle(p1.x, p1.y, d1, p2.x, p2.y, d2);
};

// Check if a point is inside a circle
p5.prototype.collidePointCircle = function(x, y, cx, cy, d) {
  return (this.dist(x, y, cx, cy) <= d / 2);
};

// Check if a point is inside a circle using p5.Vector
p5.prototype.collidePointCircleVector = function(p, c, d) {
  return this.collidePointCircle(p.x, p.y, c.x, c.y, d);
};

// Check if a point is inside an ellipse
p5.prototype.collidePointEllipse = function(x, y, cx, cy, dx, dy) {
  const rx = dx / 2;
  const ry = dy / 2;
  if (x > cx + rx || x < cx - rx || y > cy + ry || y < cy - ry) return false;

  const xx = x - cx;
  const yy = y - cy;
  const eyy = ry * this.sqrt(this.abs(rx * rx - xx * xx)) / rx;
  return (yy <= eyy && yy >= -eyy);
};

// Check if a point is inside an ellipse using p5.Vector
p5.prototype.collidePointEllipseVector = function(p, c, d) {
  return this.collidePointEllipse(p.x, p.y, c.x, c.y, d.x, d.y);
};

// Check if a point is inside a rectangle
p5.prototype.collidePointRect = function(px, py, x, y, w, h) {
  return (px >= x && px <= x + w && py >= y && py <= y + h);
};

// Check if a point is inside a rectangle using p5.Vector
p5.prototype.collidePointRectVector = function(p, r, sz) {
  return this.collidePointRect(p.x, p.y, r.x, r.y, sz.x, sz.y);
};

// Check if a point is on a line
p5.prototype.collidePointLine = function(px, py, x1, y1, x2, y2, buffer = 0.1) {
  const d1 = this.dist(px, py, x1, y1);
  const d2 = this.dist(px, py, x2, y2);
  const lineLen = this.dist(x1, y1, x2, y2);
  return (d1 + d2 >= lineLen - buffer && d1 + d2 <= lineLen + buffer);
};

// Check if a point is on a line using p5.Vector
p5.prototype.collidePointLineVector = function(p, p1, p2, buffer) {
  return this.collidePointLine(p.x, p.y, p1.x, p1.y, p2.x, p2.y, buffer);
};

// Check if a line collides with a circle
p5.prototype.collideLineCircle = function(x1, y1, x2, y2, cx, cy, diameter) {
  if (this.collidePointCircle(x1, y1, cx, cy, diameter) || this.collidePointCircle(x2, y2, cx, cy, diameter)) return true;

  const distX = x1 - x2;
  const distY = y1 - y2;
  const len = this.sqrt(distX * distX + distY * distY);

  const dot = ((cx - x1) * (x2 - x1) + (cy - y1) * (y2 - y1)) / this.pow(len, 2);
  const closestX = x1 + dot * (x2 - x1);
  const closestY = y1 + dot * (y2 - y1);

  if (!this.collidePointLine(closestX, closestY, x1, y1, x2, y2)) return false;

  const distance = this.dist(closestX, closestY, cx, cy);
  return (distance <= diameter / 2);
};

// Check if a line collides with a circle using p5.Vector
p5.prototype.collideLineCircleVector = function(p1, p2, c, diameter) {
  return this.collideLineCircle(p1.x, p1.y, p2.x, p2.y, c.x, c.y, diameter);
};

// Check if two lines collide
p5.prototype.collideLineLine = function(x1, y1, x2, y2, x3, y3, x4, y4, calcIntersection = false) {
  const uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
  const uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

  if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {
    if (this._collideDebug || calcIntersection) {
      const intersectionX = x1 + uA * (x2 - x1);
      const intersectionY = y1 + uA * (y2 - y1);
      if (this._collideDebug) this.ellipse(intersectionX, intersectionY, 10, 10);
      if (calcIntersection) return { x: intersectionX, y: intersectionY };
    }
    return true;
  }
  if (calcIntersection) return { x: false, y: false };
  return false;
};

// Check if two lines collide using p5.Vector
p5.prototype.collideLineLineVector = function(p1, p2, p3, p4, calcIntersection) {
  return this.collideLineLine(p1.x, p1.y, p2.x, p2.y, p3.x, p3.y, p4.x, p4.y, calcIntersection);
};

// Check if a line collides with a rectangle
p5.prototype.collideLineRect = function(x1, y1, x2, y2, rx, ry, rw, rh, calcIntersection = false) {
  const left = this.collideLineLine(x1, y1, x2, y2, rx, ry, rx, ry + rh, calcIntersection);
  const right = this.collideLineLine(x1, y1, x2, y2, rx + rw, ry, rx + rw, ry + rh, calcIntersection);
  const top = this.collideLineLine(x1, y1, x2, y2, rx, ry, rx + rw, ry, calcIntersection);
  const bottom = this.collideLineLine(x1, y1, x2, y2, rx, ry + rh, rx + rw, ry + rh, calcIntersection);

  if (calcIntersection) {
    return [left, right, top, bottom].find(result => result && result.x !== false && result.y !== false) || { x: false, y: false };
  }
  return (left || right || top || bottom);
};

// Check if a line collides with a rectangle using p5.Vector
p5.prototype.collideLineRectVector = function(p1, p2, r, sz, calcIntersection) {
  return this.collideLineRect(p1.x, p1.y, p2.x, p2.y, r.x, r.y, sz.x, sz.y, calcIntersection);
};

// Optimized version of collideCircleCircle function
p5.prototype.collideCircleCircleOptimized = function(x1, y1, d1, x2, y2, d2) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const distanceSq = dx * dx + dy * dy;
  const radiiSum = (d1 / 2) + (d2 / 2);
  return distanceSq <= radiiSum * radiiSum;
};

// Optimized version of collideCircleCircle function using p5.Vector
p5.prototype.collideCircleCircleOptimizedVector = function(p1, d1, p2, d2) {
  return this.collideCircleCircleOptimized(p1.x, p1.y, d1, p2.x, p2.y, d2);
};

// Enable debug mode for testing
p5.prototype.collideDebug = function(debugMode) {
  this._collideDebug = debugMode;
};
