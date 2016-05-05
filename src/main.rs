#![allow(dead_code)]

extern crate funlist;
extern crate time;
extern crate rand;
extern crate csv;
#[macro_use]
extern crate clap;

use funlist::*;
use rand::*;
use csv::*;
//use std::num;

#[derive(Clone, PartialEq)]
struct Point {
  x: f64,
  y: f64,
}

#[derive(Clone, PartialEq)]
struct Line {
  u: Point,
  v: Point,
}

#[derive(Clone, PartialEq)]
enum Shape {
  Point(Point),
  Line(Line),
}


// Point operation functions
fn point_subtract<'a>(u: &'a Point, v: &'a Point) -> Point {
  // Finds the vector from v to u
  Point { x: u.x - v.x, y: u.y - v.y }
}

fn magnitude(pt: &Point) -> f64 {
  // Finds the magnitude of position vector for pt
  ((pt.x * pt.x) + (pt.y * pt.y)).sqrt()
}

fn cross_prod(u: &Point, v: &Point) -> f64 {
  // The corss product of points u and v
  (u.x * v.y) - (u.y * v.x)
}

fn point_dist(u: &Point, v: &Point) -> f64 {
  // Distance between points u and v
  ((u.x - v.x) * (u.x - v.x) + (u.y - v.y) * (u.y - v.y)).sqrt()
}

fn line_point_dist(l: &Line, p: &Point) -> f64 {
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

fn furthest_point_from_line<'a>(l: &'a Line, points: &'a List<Point>) -> (Point, f64) {
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

fn quickhull_rec<'a>(l: &'a Line, points: &'a List<Point>, hull_accum: List<Shape>) -> List<Shape> {
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

      let hull_accum = push(hull_accum, Shape::Line(r_line.clone()));
      let hull_accum = quickhull_rec(&r_line, &r_points, hull_accum);
      let hull_accum = push(hull_accum, Shape::Point(pivot_point.clone()));
      let hull_accum = push(hull_accum, Shape::Line(l_line.clone()));
      quickhull_rec(&l_line, &l_points, hull_accum)
    }
  }
}

fn quickhull<'a>(points: &'a List<Point>) -> List<Shape> {
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

  let hull = push(List::Nil, Shape::Point(most_left));
  let hull = push(hull, Shape::Point(most_right));
  let hull = push(hull, Shape::Line(t_line.clone()));
  let hull = quickhull_rec(&t_line, &t_points, hull);
  let hull = quickhull_rec(&b_line, &b_points, hull);

  hull
}

#[test]
fn test_fns() {
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
fn test_qh_fails() {
  let points = List::Nil;
  let points = push(points, Point { x: 0., y: 0. });
  let points = push(points, Point { x: 1., y: 0. });
  let points = push(points, Point { x: 0., y: 1. });
  let points = push(points, Point { x: -1., y: 0. });
  let points = push(points, Point { x: 0., y: -1. });

  let ans: List<Shape> = List::Nil;
  let ans = push(ans, Shape::Point(Point { x: 1., y: 0. }));
  let ans = push(ans, Shape::Point(Point { x: 0., y: 1. }));
  let ans = push(ans, Shape::Point(Point { x: -1., y: 0. }));
  let ans = push(ans, Shape::Point(Point { x: 0., y: -1. }));

  let points = quickhull(&points);

  assert!(compare(&points, &ans));
}

// Runtime investigation
fn random_points(n: usize, limit: f64, accum: List<Point>) -> List<Point> {
  if n == 0 {
    accum
  } else {
    let x_pt = rand::thread_rng().gen_range(-limit, limit);
    let y_pt = rand::thread_rng().gen_range(-limit, limit);
    let accum = push(accum, Point { x: x_pt, y: y_pt });

    return random_points(n-1, limit, accum);
  }
}


fn measure_runtimes(max_n: usize, step_size: usize, iters: usize) -> Vec<(usize, u64)> {
  // Measures quickhull's performance at different input sizes.
  // The domain size for generating random points grows with n such that the
  // average density of points remains constant

  // max_n: largest size of input
  // step_size: how far to step n between runs
  // iters: how many times to repeat the measurement at each step, returned runtimes are averaged

  let mut runtimes: Vec<(usize, u64)> = vec![];
  let mut points = random_points(10, 10., List::Nil);

  let the_end: usize = (max_n - 10) / step_size + 1;
  for ii in 1..the_end {
    let n: usize = (ii * step_size) + 9;
    if n % 100 == 0 {
      println!("Running n = {}", n);
    }

    let mut runtime: u64 = 0;
    let limit: f64 = 1000.;

    // Add an additional point!
    points = random_points(1, limit, points);

    for _ in 0..iters {
      let start = time::precise_time_ns();
      quickhull(&points);
      let end = time::precise_time_ns();

      runtime = runtime + (end - start);
    }

    runtimes.push((n, runtime / iters as u64));
  }

  runtimes
}


fn main2(max_n: usize, step_size: usize, iters: usize) {
  let b_start = time::precise_time_ns();

  let runtimes = measure_runtimes(max_n, step_size, iters);

  let b_end = time::precise_time_ns();

  let path = format!("qhmin_runtime_n{}_s{}_i{}.csv", max_n, step_size, iters);
  let mut writer = Writer::from_file(path).unwrap();
  for r in runtimes.into_iter() {
    writer.encode(r).ok().expect("CSV writer error");
  }

  println!("Quickhull runtimes analysis ran in {} seconds", (b_end - b_start) / 1000000000);
}


fn main() {
  // Command line arguments
  let args = clap::App::new("Quickhull")
    .version("0.1")
    .author("Trevor DiMartino")
    .about("Quickhull, both traditional and incremental")
    .args_from_usage("\
      -n --maxn=[max_n]            'maximum points to run'
      -s --stepsize=[step_size]    'points to add between runs'
      -i --iterations=[iters]      'number of runs at each step to average runtimes over' ")
    .get_matches();

  let max_n = value_t!(args.value_of("maxn"), usize).unwrap_or(1000);
  let step_size = value_t!(args.value_of("stepsize"), usize).unwrap_or(1);
  let iters = value_t!(args.value_of("iterations"), usize).unwrap_or(1);
  use std::thread;
  //use std::thread::JoinHandle;
  let child =
    thread::Builder::new().stack_size(64 * 1024 * 1024).spawn(move || { main2(max_n, step_size, iters) });
  let _ = child.unwrap().join();
}