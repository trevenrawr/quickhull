#![allow(dead_code)]

use std::rc::Rc;  

#[macro_use]
extern crate adapton;

use adapton::collections::*;
use adapton::engine::*;
use adapton::macros::*;

#[derive(Clone, Debug, Hash, Eq, PartialEq)]
struct Point {
	x: isize,
	y: isize,
}

#[derive(Clone, Debug, Hash, Eq, PartialEq)]
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
	(((pt.x * pt.x) + (pt.y * pt.y)) as f32).sqrt()
}

fn cross_prod(u: &Point, v: &Point) -> isize {
	// The corss product of points u and v
	(u.x * v.y) - (u.y * v.x)
}

fn point_dist(u: &Point, v: &Point) -> f32 {
	// Distance between points u and v
	(((u.x - v.x) * (u.x - v.x) + (u.y - v.y) * (u.y - v.y)) as f32).sqrt()
}

fn line_point_dist(l: &Line, p: &Point) -> f32 {
	let d1 = point_subtract(&l.v, &l.u);
	let d2 = point_subtract(&l.u, &p);
	let d3 = point_subtract(&l.v, &l.u);

	((cross_prod(&d1, &d2) as f32) / magnitude(&d3)).abs()
}

fn line_side_test(l: &Line, p: &Point) -> bool {
	// Tests which side of the line a point is on
	if (l.u == *p) || (l.v == *p) {
		false
	} else {
		let d1 = point_subtract(&l.v, &l.u);
		let d2 = point_subtract(&l.u, &p);
		let c = cross_prod(&d1, &d2);
		if c <= 0 {
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


fn furthest_point_from_line<Lev:Level, T:TreeElim<Lev,Point>+'static>
  (l: Line, points: T) -> Point
{
  ns(name_of_str("furthest_point"),
     || monoid_of_tree
     (points, l.u.clone(), Rc::new(
       move |q, p| {
       	 let dp = line_point_dist(&l, &p);
       	 let dq = line_point_dist(&l, &q);
         if dp > dq { p } else { q }
       })))
}

// fn furthest_point_from_line<'a>(l: &'a Line, points: &'a List<Point>) -> (Point, f32) {
//   match *points {
//     List::Nil =>
//       panic!{"Can't do quickhull on no points! (furthest_point_from_line received empty point list)"},
//     List::Cons(_) => {
//       fold(&points, (l.u.clone(), 0.), |(q, max_d), p| {
//       	let d = line_point_dist(&l, &p);
//       	if d > max_d {
//       	  (p.clone(), d)
//       	} else {
//       	  (q, max_d)
//       	}
//       })
//     }
//   }
// }

#[derive(Clone,Hash,Eq,PartialEq,Debug)]
enum Shape {
  Point(Point),
  Line(Line),
}

fn quickhull_rec
  < Lev:Level+'static
  , T:TreeIntro<Lev,Point>+TreeElim<Lev,Point>+'static
  , L:ListIntro<Shape>+'static
  >
  (n:Name, l: Line, points:T, hull:L) -> L
{
  if T::is_empty(&points) { hull }
  else {
    let pivot = furthest_point_from_line(l.clone(), points.clone());
    let l_line = Line { u: l.u.clone(), v: pivot.clone() };
    let r_line = Line { u: pivot.clone(), v: l.v.clone() };
    
    let l_line2 = l_line.clone();
    let l_points =
      ns(name_of_str("filter-l"),
         ||filter_tree_of_tree::<_,_,_,T>
         (points.clone(), Box::new(move |p| line_side_test(&l_line2, &p) )));
    
    let r_line2 = r_line.clone();
    let r_points =
      ns(name_of_str("filter-r"),
         ||filter_tree_of_tree::<_,_,_,T>
         (points.clone(), Box::new(move |p| line_side_test(&r_line2, &p) )));

    let (nr, nl) = name_fork(n);    
    let hull = L::cons(Shape::Line(r_line.clone()), hull);
    let hull = ns(nr.clone(), ||(quickhull_rec_2(nr, r_line, r_points, hull)));
    let hull = L::cons(Shape::Point(pivot.clone()), hull);
    let hull = L::cons(Shape::Line(l_line.clone()), hull);
    let hull = ns(nl.clone(), ||quickhull_rec_2(nl, l_line, l_points, hull));
    hull
  }
}

fn quickhull_rec_2
  < Lev:Level+'static
  , T:TreeIntro<Lev,Point>+TreeElim<Lev,Point>+'static
  , L:ListIntro<Shape>+'static
  >
  (n:Name, l: Line, points:T, hull:L) -> L
{
  let (c, _) = eager!(n.clone() =>>
                      quickhull_rec, n:n, l:l, points:points, hull:hull);
  L::art(c)
}

// fn quickhull_rec<'a>(l: &'a Line, points: &'a List<Point>, hull_accum: List<Shape>) -> List<Shape> {
//   match *points {
//     List::Nil     => hull_accum,
//     List::Cons(_) => {
//       // Find pivot and set up new lines on right and left
//       let (pivot_point, _) = furthest_point_from_line(&l, &points);
//       let l_line = Line { u: l.u.clone(), v: pivot_point.clone() };
//       let r_line = Line { u: pivot_point.clone(), v: l.v.clone() };

//       // Find points outside l_line and r_line
//       let l_points = filter(&points, |p|{ line_side_test(&l_line, &p) });
//       let r_points = filter(&points, |p|{ line_side_test(&r_line, &p) });

//       let hull_accum = push(hull_accum,  Shape::Line(r_line.clone()));
//       let hull_accum = quickhull_rec(&r_line, &r_points, hull_accum);
//       let hull_accum = push(hull_accum, Shape::Point(pivot_point.clone()));
//       let hull_accum = push(hull_accum,  Shape::Line(l_line.clone()));
//       quickhull_rec(&l_line, &l_points, hull_accum)
//     }
//   }
// }

// // TODO: Use this to time quickhull:
// // pub fn measure_ns<F:FnOnce()>(f: F) -> u64 {
// //   let start = time::precise_time_ns();
// //   f();
// //   let end = time::precise_time_ns();
// //   end - start
// // }

fn quickhull
  < Lev:Level+'static
  , T:TreeIntro<Lev,Point>+TreeElim<Lev,Point>+'static
  , L:ListIntro<Shape>+'static
  >
  (points:T) -> L
{
  let most_left =
    ns(name_of_str("most-left"),
       ||monoid_of_tree
       (points.clone(),
        Point {x:isize::max_value(), y:0},
        Rc::new(
          |p:Point, q:Point| { if p.x < q.x { p } else { q.clone() } })));

  let most_right =
    ns(name_of_str("most-right"),
       ||monoid_of_tree
       (points.clone(),
        Point {x:-isize::max_value(), y:0},
        Rc::new(
          |p:Point, q:Point| { if p.x > q.x { p } else { q.clone() } })));
       
 
  let t_line = Line { u: most_left.clone(), v: most_right.clone() };
  let b_line = Line { u: most_right.clone(), v: most_left.clone() };

  let t_line2 = t_line.clone();
  let t_points =
    ns(name_of_str("t-points"),
       || filter_tree_of_tree::<_,_,_,T>
       (points.clone(), Box::new(move |p| line_side_test(&t_line2, &p) )));
  
  let b_line2 = b_line.clone();
  let b_points =
    ns(name_of_str("b-points"),
       || filter_tree_of_tree::<_,_,_,T>
       (points.clone(), Box::new(move |p| line_side_test(&b_line2, &p) )));

  let (nt,nb) = name_fork(name_of_str("rec"));
  
  let hull = L::cons(Shape::Point(most_left), L::nil());  
  let hull = ns(nb.clone(),||quickhull_rec_2(nb, b_line, b_points, hull));

  let hull = L::cons(Shape::Point(most_right), hull);
  let hull = ns(nt.clone(),||quickhull_rec_2(nt, t_line, t_points, hull));
  hull
}

// #[test]
// fn test() {
//   // Testing items:
//   let p = Point { x: 3., y: 4. };
//   let q = Point { x: 0., y: 0. };
//   let l = Line { u: p.clone(), v: q.clone() };
  
//   let t = point_subtract(&p, &q);
//   println!("Difference between p and q is ({}, {})", t.x, t.y);
//   println!("Distance between p and q is {}", point_dist(&l.u, &l.v));
//   println!("Magnitude of vector p is {}", magnitude(&p));
//   println!("Cross product of vecors p and q is {}", cross_prod(&p, &q));
  
//   let r = Point { x: 12., y: -3. };
//   println!("Distance from line to r is {}", line_point_dist(&l, &r));
  
//   println!("Point r is on {} side of line", line_side_test(&l, &r));
  
//   let points: List<Point> = List::Nil;
//   let points = push(points, p);
//   let points = push(points, q);
//   let points = push(points, r);
//   let (pt, dist) = furthest_point_from_line(&l, &points);
  
//   println!("Furthest point from line l is ({}, {}), at dist = {}.", pt.x, pt.y, dist)
// }

// #[test]
// fn test_qh() {
//   let points = List::Nil;
//   let points = push(points, Point { x: 0., y: 0. });
//   let points = push(points, Point { x: 1., y: 0. });
//   let points = push(points, Point { x: 0., y: 1. });
//   let points = push(points, Point { x: -1., y: 0. });
//   let points = push(points, Point { x: 0., y: -1. });

//   let ans = List::Nil;
//   let ans = push(ans, Point { x: 1., y: 0. });
//   let ans = push(ans, Point { x: 0., y: 1. });
//   let ans = push(ans, Point { x: -1., y: 0. });
//   let ans = push(ans, Point { x: 0., y: -1. });

//   let points = quickhull(&points);

//   //assert!(compare(&points, &ans));
// }

#[test]
pub fn test_qh () {
  fn doit() -> Vec<NameElse<Shape>> {
    let l = list_of_vec::<Point,List<_>>(
      &vec![
        NameElse::Name(name_of_usize(0)),  NameElse::Else(Point{x: 2,y: 2}), // inside hull
        NameElse::Name(name_of_usize(1)),  NameElse::Else(Point{x: 2,y:-2}), // inside hull
        NameElse::Name(name_of_usize(2)),  NameElse::Else(Point{x:-2,y:-2}), // inside hull
        NameElse::Name(name_of_usize(3)),  NameElse::Else(Point{x:-2,y: 2}), // inside hull
        
        NameElse::Name(name_of_usize(4)),  NameElse::Else(Point{x: 6,y: 6}), // on hull
        NameElse::Name(name_of_usize(5)),  NameElse::Else(Point{x: 6,y:-6}), // on hull
        NameElse::Name(name_of_usize(6)),  NameElse::Else(Point{x:-6,y:-6}), // on hull
        NameElse::Name(name_of_usize(7)),  NameElse::Else(Point{x:-6,y: 6}), // on hull

        NameElse::Name(name_of_usize(8)),  NameElse::Else(Point{x:1,y:1}), // inside hull
        NameElse::Name(name_of_usize(9)),  NameElse::Else(Point{x:3,y:0}), // inside hull
        NameElse::Name(name_of_usize(10)), NameElse::Else(Point{x:0,y:3}), // inside hull
        NameElse::Name(name_of_usize(11)), NameElse::Else(Point{x:5,y:3}), // inside hull
        NameElse::Name(name_of_usize(12)), NameElse::Else(Point{x:5,y:5}), // inside hull
        
        NameElse::Name(name_of_usize(13)), NameElse::Else(Point{x:10,y:0}),  // on hull
        NameElse::Name(name_of_usize(14)), NameElse::Else(Point{x:0,y:10}),  // on hull
        NameElse::Name(name_of_usize(15)), NameElse::Else(Point{x:-10,y:0}), // on hull
        NameElse::Name(name_of_usize(16)), NameElse::Else(Point{x:0,y:-10}), // on hull

        NameElse::Name(name_of_usize(17)), NameElse::Else(Point{x:10,y:2}),  // on hull ?
        NameElse::Name(name_of_usize(18)), NameElse::Else(Point{x:2,y:10}),  // on hull ?
        NameElse::Name(name_of_usize(19)), NameElse::Else(Point{x:-10,y:2}), // on hull ?
        NameElse::Name(name_of_usize(20)), NameElse::Else(Point{x:2,y:-10}), // on hull ?

        NameElse::Name(name_of_usize(21)),
        ]);
    let t = ns(name_of_str("tree_of_list"),
               ||tree_of_list::<_,_,Tree<_>,_>(Dir2::Right, l));
    let h = ns(name_of_str("quickhull"),
               ||quickhull::<_,_,List<_>>(t.clone()));
    let o = vec_of_list(h, None);
    println!("{:?}", &o);
    o
  }
  init_naive();
  println!("Naive run");
  let o1 = doit();
  init_dcg();
  println!("DCG run");
  let o2 = doit();
  assert_eq!(o1, o2);
}


fn main() {
}
