#![allow(dead_code)]

extern crate funlist;

use funlist::*;
use std::num;

struct Point {
	x: f32,
	y: f32,
}

struct Line<'a> {
	u: &'a Point,
	v: &'a Point,
}


// Point comparison functions
fn x_max<'a>(u: &'a Point, v: &'a Point) -> &'a Point {
	if u.x > v.x { return u; } else { return v;	}
}
fn y_max<'a>(u: &'a Point, v: &'a Point) -> &'a Point {
	if u.y > v.y { return u; } else { return v;	}
}
fn x_min<'a>(u: &'a Point, v: &'a Point) -> &'a Point {
	if u.x < v.x { return u; } else { return v;	}
}
fn y_min<'a>(u: &'a Point, v: &'a Point) -> &'a Point {
	if u.y < v.y { return u; } else { return v;	}
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
	if (l.u.x == p.x && l.u.y == p.y) || (l.v.x == p.x && l.v.y == p.y) {
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


fn furthest_point_from_line<'a>(l: &'a Line, points: &'a List<&'a Point>) -> (&'a Point, f32) {
	match *points {
		List::Nil =>
			panic!{"Can't do quickhull on no points! (furthest_point_from_line received empty point list)"},
		List::Cons(ref cons) => {
			fold(&points, (&l.u, 0.), |(q, max_d), p| {
				let d = line_point_dist(&l, &p);
				if d > max_d {
					(p, d)
				} else {
					(q, max_d)
				}
			})
		}
	}
}


fn quickhull_rec<'a>(l: &'a Line, points: &'a List<&'a Point>, hull_accum: List<&'a Point>) -> List<&'a Point> {
	match *points {
		List::Nil =>
			hull_accum,
		List::Cons(ref cons) => {
			// Find pivot and set up new lines on right and left
			let (pivot_point, _) = furthest_point_from_line(&l, &points);
			let l_line = Line { u: l.u, v: pivot_point };
			let r_line = Line { u: pivot_point, v: l.v };

			// Find points outside l_line
			let l_points = fold(&points, List::Nil, |outside, p| {
				if line_side_test(&l_line, &p) {
					let outside = push(outside, *p);
					outside
				} else {
					outside
				}
			});
			// Find points outside r_line
			let r_points = fold(&points, List::Nil, |outside, p| {
				if line_side_test(&r_line, &p) {
					let outside = push(outside, *p);
					outside
				} else {
					outside
				}
			});

			let hull_accum = quickhull_rec(&r_line, &r_points, hull_accum);
			quickhull_rec(&l_line, &l_points, push(hull_accum, pivot_point))
		}
	}
}





fn main() {
	// Testing items:
	let p = Point { x: 3., y: 4. };
	let q = Point { x: 0., y: 0. };
	let l = Line { u: &p, v: &q };

	let t = point_subtract(&p, &q);
	println!("Difference between p and q is ({}, {})", t.x, t.y);
	println!("Distance between p and q is {}", point_dist(&l.u, &l.v));
	println!("Magnitude of vector p is {}", magnitude(&p));
	println!("Cross product of vecors p and q is {}", cross_prod(&p, &q));

	let r = Point { x: 12., y: -3. };
	println!("Distance from line to r is {}", line_point_dist(&l, &r));

	println!("Point r is on {} side of line", line_side_test(&l, &r));


	let points: List<&Point> = List::Nil;
	let points = push(points, &p);
	let points = push(points, &q);
	let points = push(points, &r);

	let (pt, dist) = furthest_point_from_line(&l, &points);
	println!("Furthest point from line l is ({}, {}), at dist = {}.", pt.x, pt.y, dist)
}
