#![allow(dead_code)]

extern crate funlist;

use funlist::*;
//use std::num;

#[derive(Clone, PartialEq)]
struct Point {
	x: f32,
	y: f32,
}

#[derive(Clone)]
struct Line {
	u: Point,
	v: Point,
}


// Point operation functions
fn point_subtract<'a>(u: &'a Point, v: &'a Point) -> Point {
	// Finds the difference between u and v
	Point { x: u.x - v.x, y: u.y - v.y}
}

fn magnitude(pt: &Point) -> f32 {
	// Finds the magnitude of position vector for pt
	((pt.x * pt.x) + (pt.y * pt.y)).sqrt()
}

fn cross_prod(u: &Point, v: &Point) -> f32 {
	// The corss product of points u and v
	(u.x * v.y) - (u.y * v.x)
}

fn point_dist(u: &Point, v: &Point) -> f32 {
	// Distance between points u and v
	((u.x - v.x) * (u.x - v.x) + (u.y - v.y) * (u.y - v.y)).sqrt()
}

fn line_point_dist(l: &Line, p: &Point) -> f32 {
	let d1 = point_subtract(&l.v, &l.u);
	let d2 = point_subtract(&l.u, &p);
	let d3 = point_subtract(&l.v, &l.u);

	((cross_prod(&d1, &d2)) / magnitude(&d3)).abs()
}

fn line_side_test(l: &Line, p: &Point) -> bool {
	// Tests which side of the line a point is on
	if (l.u == *p) || (l.v == *p) {
		false
	} else {
		let d1 = point_subtract(&l.v, &l.u);
		let d2 = point_subtract(&l.u, &p);
		let c = cross_prod(&d1, &d2);
		if c <= 0.0 {
			false
		} else {
			true
		}
	}
}

fn max_point_from_line<'a>(l: &'a Line, u: &'a Point, v: &'a Point) -> (bool, &'a Point) {
	let d1 = line_point_dist(&l, &u);
	let d2 = line_point_dist(&l, &v);
	if d1 > d2 {
		(true, u)
	} else {
		(false, v)
	}
}

fn furthest_point_from_line<'a>(l: &'a Line, points: &'a List<Point>) -> (Point, f32) {
  match *points {
    List::Nil =>
      panic!{"Can't do quickhull on no points! (furthest_point_from_line received empty point list)"},
    List::Cons(_) => {
      fold(&points, (l.u.clone(), 0.), |(q, max_d), p| {
      	let d = line_point_dist(&l, &p);
      	if d > max_d {
      	  (p.clone(), d)
      	} else {
      	  (q, max_d)
      	}
      })
    }
  }
}


fn quickhull_rec<'a>(l: &'a Line, points: &'a List<Point>, hull_accum: List<Point>) -> List<Point> {
  match *points {
    List::Nil     => hull_accum,
    List::Cons(_) => {
      // Find pivot and set up new lines on right and left
      let (pivot_point, _) = furthest_point_from_line(&l, &points);
      let l_line = Line { u: l.u.clone(), v: pivot_point.clone() };
      let r_line = Line { u: pivot_point.clone(), v: l.v.clone() };

      // Find points outside l_line and r_line
      let l_points = filter(&points, |p|{ line_side_test(&l_line, &p) });
      let r_points = filter(&points, |p|{ line_side_test(&r_line, &p) });

      let hull_accum = quickhull_rec(&r_line, &r_points, hull_accum);
      quickhull_rec(&l_line, &l_points, push(hull_accum, pivot_point))
    }
  }
}

fn quickhull<'a>(points: &'a List<Point>) -> List<Point> {
  let most_left = fold(&points, Point { x: 0., y: 0. }, |p, q| {
    if p.x < q.x { p } else { q.clone() }
  });
  let most_right = fold(&points, Point { x: 0., y: 0. }, |p, q| {
    if p.x > q.x { p } else { q.clone() }
  });

  let t_line = Line { u: most_left.clone(), v: most_right.clone() };
  let b_line = Line { u: most_right.clone(), v: most_left.clone() };

  let t_points = filter(&points, |p|{ line_side_test(&t_line, &p) });
  let b_points = filter(&points, |p|{ line_side_test(&b_line, &p) });

  let hull = push(List::Nil, most_left);
  let hull = push(hull, most_right);
  let hull = quickhull_rec(&t_line, &t_points, hull);
  let hull = quickhull_rec(&b_line, &b_points, hull);

  hull
}

#[test]
fn test() {
  // Testing items:
  let p = Point { x: 3., y: 4. };
  let q = Point { x: 0., y: 0. };
  let l = Line { u: p.clone(), v: q.clone() };
  
  let t = point_subtract(&p, &q);
  println!("Difference between p and q is ({}, {})", t.x, t.y);
  println!("Distance between p and q is {}", point_dist(&l.u, &l.v));
  println!("Magnitude of vector p is {}", magnitude(&p));
  println!("Cross product of vecors p and q is {}", cross_prod(&p, &q));
  
  let r = Point { x: 12., y: -3. };
  println!("Distance from line to r is {}", line_point_dist(&l, &r));
  
  println!("Point r is on {} side of line", line_side_test(&l, &r));
  
  let points: List<Point> = List::Nil;
  let points = push(points, p);
  let points = push(points, q);
  let points = push(points, r);
  let (pt, dist) = furthest_point_from_line(&l, &points);
  
  println!("Furthest point from line l is ({}, {}), at dist = {}.", pt.x, pt.y, dist)
}

#[test]
fn test_qh() {
  let points = List::Nil;
  let points = push(points, Point { x: 0., y: 0. });
  let points = push(points, Point { x: 1., y: 0. });
  let points = push(points, Point { x: 0., y: 1. });
  let points = push(points, Point { x: -1., y: 0. });
  let points = push(points, Point { x: 0., y: -1. });

  let ans = List::Nil;
  let ans = push(ans, Point { x: 1., y: 0. });
  let ans = push(ans, Point { x: 0., y: 1. });
  let ans = push(ans, Point { x: -1., y: 0. });
  let ans = push(ans, Point { x: 0., y: -1. });

  let points = quickhull(&points);

  assert!(compare(&points, &ans));
}


fn main() {
}
