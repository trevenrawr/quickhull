#![allow(dead_code)]

extern crate funlist;

use funlist::*;
use std::num;


#[derive(Copy, Clone)]
struct Point {
	x: f32,
	y: f32,
}

#[derive(Copy, Clone)]
struct Line {
	u: Point,
	v: Point,
}


// Point comparison functions
fn x_max(u: Point, v: Point) -> Point {
	if u.x > v.x { return u; } else { return v;	}
}
fn y_max(u: Point, v: Point) -> Point {
	if u.y > v.y { return u; } else { return v;	}
}
fn x_min(u: Point, v: Point) -> Point {
	if u.x < v.x { return u; } else { return v;	}
}
fn y_min(u: Point, v: Point) -> Point {
	if u.y < v.y { return u; } else { return v;	}
}

// Point operation functions
fn point_subtract(u: Point, v: Point) -> Point {
	// Finds the difference between u and v
	Point { x: u.x - v.x, y: u.y - v.y}
}

fn magnitude(pt: Point) -> f32 {
	// Finds the magnitude of position vector for pt
	((pt.x * pt.x) + (pt.y * pt.y)).sqrt()
}

fn cross_prod(u: Point, v: Point) -> f32 {
	// The corss product of points u and v
	(u.x * v.y) - (u.y * v.x)
}

fn point_dist(u: Point, v: Point) -> f32 {
	// Distance between points u and v
	((u.x - v.x) * (u.x - v.x) + (u.y - v.y) * (u.y - v.y)).sqrt()
}

fn line_point_dist(l: Line, p: Point) -> f32 {
	let d1 = point_subtract(l.v, l.u);
	let d2 = point_subtract(l.u, p);
	let d3 = point_subtract(l.v, l.u);

	((cross_prod(d1, d2)) / magnitude(d3)).abs()
}

fn line_side_test(l: Line, p: Point) -> bool {
	// Tests which side of the line a point is on
	if (l.u.x == p.x && l.u.y == p.y) || (l.v.x == p.x && l.v.y == p.y) {
		return false
	} else {
		let d1 = point_subtract(l.v, l.u);
		let d2 = point_subtract(l.u, p);
		let c = cross_prod(d1, d2);
		if c <= 0.0 {
			return false
		} else {
			return true
		}
	}
}

fn max_point_from_line(l: Line, u: Point, v: Point) -> (bool, Point) {
	let d1 = line_point_dist(l, u);
	let d2 = line_point_dist(l, v);
	if d1 > d2 { return (true, u) } else { return (false, v) }
}


fn furthest_point_from_line(l: Line, points: List<Point>) -> (Point, f32) {
	(Point { x: 0., y: 0. }, 0.)
}


fn main() {
	let mut points: List<Point> = List::Nil;

	let p = Point { x: 3., y: 4. };
	let q = Point { x: 0., y: 0. };
	let l = Line { u: p, v: q };

	let t = point_subtract(p, q);
	println!("Difference between p and q is ({}, {})", t.x, t.y);
	println!("Distance between p and q is {}", point_dist(p, q));
	println!("Magnitude of vector p is {}", magnitude(p));
	println!("Cross product of vecors p and q is {}", cross_prod(p, q));

	let r = Point { x: 12., y: -3. };
	println!("Distance from line to r is {}", line_point_dist(l, r));

	println!("Point r is on {} side of line", line_side_test(l, r));


	push();
	fold();
}
