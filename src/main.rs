#![allow(dead_code)]

// $env:ADAPTON_STRUCTURAL=1

use std::rc::Rc;  

#[macro_use]
extern crate adapton;
extern crate time;
extern crate rand;
extern crate csv;
#[macro_use]
extern crate clap;

use adapton::collections::*;
use adapton::engine::*;
use adapton::macros::*;
use rand::*;
use csv::Writer;

#[derive(Clone, Debug, Hash, Eq, PartialEq)]
pub struct Point {
  x: isize,
  y: isize,
}

#[derive(Clone, Debug, Hash, Eq, PartialEq)]
pub struct Line {
  u: Point,
  v: Point,
}


// Point operation functions
pub fn point_subtract<'a>(u: &'a Point, v: &'a Point) -> Point {
  // Finds the difference between u and v
  Point { x: u.x - v.x, y: u.y - v.y}
}

pub fn magnitude(pt: &Point) -> f32 {
  // Finds the magnitude of position vector for pt
  (((pt.x * pt.x) + (pt.y * pt.y)) as f32).sqrt()
}

pub fn cross_prod(u: &Point, v: &Point) -> isize {
  // The corss product of points u and v
  (u.x * v.y) - (u.y * v.x)
}

pub fn point_dist(u: &Point, v: &Point) -> f32 {
  // Distance between points u and v
  (((u.x - v.x) * (u.x - v.x) + (u.y - v.y) * (u.y - v.y)) as f32).sqrt()
}

pub fn line_point_dist(l: &Line, p: &Point) -> f32 {
  let d1 = point_subtract(&l.v, &l.u);
  let d2 = point_subtract(&l.u, &p);
  let d3 = point_subtract(&l.v, &l.u);  
  ((cross_prod(&d1, &d2) as f32) / magnitude(&d3)).abs()
}

pub fn line_side_test(l: &Line, p: &Point) -> bool {
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

pub fn max_point_from_line<'a>(l: &'a Line, u: &'a Point, v: &'a Point) -> (bool, &'a Point) {
  let d1 = line_point_dist(&l, &u);
  let d2 = line_point_dist(&l, &v);
  if d1 > d2 {
    (true, u)
  } else {
    (false, v)
  }
}

pub fn furthest_point_from_line<Lev:Level, T:TreeElim<Lev,Point>+'static>
  (l: Line, points: T) -> Point
{
  monoid_of_tree
    (points, l.u.clone(), Rc::new(
      move |q, p| {
       	let dp = line_point_dist(&l, &p);
       	let dq = line_point_dist(&l, &q);
        if dp > dq { p } else { q }
      }))
}

#[derive(Clone,Hash,Eq,PartialEq,Debug)]
pub enum Shape {
  Point(Point),
  Line(Line),
}

pub fn name_of_point (p:&Point) -> Name { name_pair(name_of_isize( p.x), name_of_isize( p.y)) }
pub fn name_of_line  (l:&Line)  -> Name { name_pair(name_of_point(&l.u), name_of_point(&l.v)) }

fn quickhull_rec
  < Lev:Level+'static
  , T:TreeIntro<Lev,Point>+TreeElim<Lev,Point>+'static
  , L:ListIntro<Shape>+'static
  >
  (l: Line, points:T, hull:L) -> L
{
  if T::is_empty(&points) { hull }
  else {
    let mid =
      ns(name_pair(name_of_str("mid"),name_of_line(&l)),
         || furthest_point_from_line(l.clone(), points.clone()));
    
      let l_line = Line { u: l.u.clone(), v: mid.clone() };
      let r_line = Line { u: mid.clone(), v: l.v.clone() };
    
    let l_line2 = l_line.clone();
    let l_points =
      ns(name_of_line(&l_line2),
         ||filter_tree_of_tree::<_,_,_,T>
         (points.clone(), Box::new(move |p| line_side_test(&l_line2, &p) )));
    
    let r_line2 = r_line.clone();
    let r_points =
      ns(name_of_line(&r_line2),
         ||filter_tree_of_tree::<_,_,_,T>
         (points.clone(), Box::new(move |p| line_side_test(&r_line2, &p) )));
    
    let (nr, nl) = name_fork(name_unit());
    //let hull = L::cons(Shape::Line(r_line.clone()), hull);
    let hull = ns(nr.clone(), ||(quickhull_rec_2(r_line, r_points, hull)));
    let hull = L::cons(Shape::Point(mid.clone()), hull);
    let hull = L::name(nl.clone(), hull);
    //let hull = L::cons(Shape::Line(l_line.clone()), hull);
    let hull = ns(nl.clone(), ||quickhull_rec_2(l_line, l_points, hull));
    hull
  }
}

fn quickhull_rec_2
  < Lev:Level+'static
  , T:TreeIntro<Lev,Point>+TreeElim<Lev,Point>+'static
  , L:ListIntro<Shape>+'static
  >
  (l: Line, points:T, hull:L) -> L
{
  // This macro `eager!` memo-izes the call to quickhull_rec.
  // It places the call and its result in an articulation named `n`.
  let (c, _) = eager!(name_unit() =>>
                      quickhull_rec, l:l, points:points, hull:hull);
  L::art(c)
}

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
    ns(name_of_line(&t_line2),
       || filter_tree_of_tree::<_,_,_,T>
       (points.clone(), Box::new(move |p| line_side_test(&t_line2, &p) )));
  
  let b_line2 = b_line.clone();
  let b_points =
    ns(name_of_line(&b_line2),
       || filter_tree_of_tree::<_,_,_,T>
       (points.clone(), Box::new(move |p| line_side_test(&b_line2, &p) )));

  let (nt,nb) = name_fork(name_of_str("rec"));
  
  let hull = L::cons(Shape::Point(most_left), L::nil());
  let hull = L::art(cell(nb.clone(), hull));
  let hull = L::name(nb.clone(), hull);
  let hull = ns(nb.clone(),||quickhull_rec_2(b_line, b_points, hull));

  let hull = L::cons(Shape::Point(most_right), hull);
  let hull = L::art(cell(nt.clone(), hull));
  let hull = L::name(nt.clone(), hull);
  let hull = ns(nt.clone(),||quickhull_rec_2(t_line, t_points, hull));
  hull
}


fn test_input() -> List<Point> {
  list_of_vec::<Point,List<_>>( &vec![
    NameElse::Name(name_of_usize(4)),  NameElse::Else(Point{x: 6,y: 6}), // on hull
    NameElse::Name(name_of_usize(5)),  NameElse::Else(Point{x: 6,y:-6}), // on hull
    NameElse::Name(name_of_usize(6)),  NameElse::Else(Point{x:-6,y:-6}), // on hull
    NameElse::Name(name_of_usize(7)),  NameElse::Else(Point{x:-6,y: 6}), // on hull
    
    NameElse::Name(name_of_usize(14)), NameElse::Else(Point{x:6,y:6}),  // on hull
    NameElse::Name(name_of_usize(15)), NameElse::Else(Point{x:-6,y:-6}),  // on hull
    NameElse::Name(name_of_usize(16)), NameElse::Else(Point{x:6,y:6}), // on hull
    NameElse::Name(name_of_usize(17)), NameElse::Else(Point{x:6,y:-6}), // on hull
    
    // NameElse::Name(name_of_usize(33)),
    ])
}

#[test]
pub fn test_qh () {
  fn doit() -> Vec<NameElse<Shape>> {
    let l = test_input();
    let t = ns(name_of_str("tree_of_list"),
               ||tree_of_list::<_,_,Tree<_>,_>(Dir2::Right, l));
    let h = ns(name_of_str("quickhull"),
               ||quickhull::<_,_,List<_>>(t.clone()));
    let o = vec_of_list(h, None);
    // println!("{:?}", &o);
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

#[test]
pub fn test_qh_inc () {
  // Generate a random point (in a square from -limit to limit)
  fn gen_point(limit: isize) -> Point {
    let x_pt = rand::thread_rng().gen_range(-limit, limit);
    let y_pt = rand::thread_rng().gen_range(-limit, limit);

    Point{ x: x_pt, y: y_pt }
  };

  fn push_point(name: usize, pt: Point, l: List<Point>) -> List<Point> {
    let l = <List<Point> as ListIntro<Point>>::art(cell(name_of_usize(name), l));
    let l = <List<Point> as ListIntro<Point>>::name(name_of_usize(name), l);
    let l = <List<Point> as ListIntro<Point>>::cons(pt,  l);
    l
  };
  
  fn doit(l:List<Point>) -> Vec<NameElse<Shape>> {
    let t = ns(name_of_str("tree_of_list"),
               ||tree_of_list::<_,_,Tree<_>,_>(Dir2::Right, l));
    let h = ns(name_of_str("quickhull"),
               ||quickhull::<_,_,List<_>>(t.clone()));
    let o = vec_of_list(h, None);
    // println!("hull = {:?}\n", &o);
    o
  };

  let new_point = gen_point(10);

  println!("Naive run");
  init_naive();
  let inp      = test_input();
  let naive_o1 = doit(inp.clone());
  let inp      = push_point(333, new_point.clone(), inp);
  let naive_o2 = doit(inp);

  println!("DCG run");
  init_dcg();
  let inp     = test_input();
  let dcg_o1  = doit(inp.clone());
  let inp     = push_point(333, new_point.clone(), inp);
  let dcg_o2  = doit(inp);

  assert_eq!(naive_o1, dcg_o1);
  assert_eq!(naive_o2, dcg_o2);  
}

#[test]
pub fn rh_test() {
  runtime_harness(100, 5);
}


fn runtime_harness(max_n: usize, pts_per_step: usize) -> Vec<(usize, u64, u64)> {
  // We start with 10 random points, so max size has to be bigger than that!
  assert!(max_n > 11);

  let name_step = 2;
  
  // Generate a random point (in a square from -limit to limit)
  fn gen_point(limit: isize) -> Point {
    let x_pt = rand::thread_rng().gen_range(-limit, limit);
    let y_pt = rand::thread_rng().gen_range(-limit, limit);

    Point{ x: x_pt, y: y_pt }
  };

  fn push_point(name_step:usize, name: usize, pt: Point, l: List<Point>) -> List<Point> {
    let l = if name % name_step == 0 {
      let l = <List<Point> as ListIntro<Point>>::art(cell(name_of_usize(name), l));
      let l = <List<Point> as ListIntro<Point>>::name(name_of_usize(name), l);
      l
    } else { l } ;
    let l = <List<Point> as ListIntro<Point>>::cons(pt,  l);
    l
  };
  
  // The code that we want to compare/measure under naive versus DCG engines:
  fn doit(l:List<Point>) -> Vec<NameElse<Shape>> {
    let t = ns(name_of_str("tree_of_list"),
               ||tree_of_list::<_,_,Tree<_>,_>(Dir2::Right, l));
    let h = ns(name_of_str("quickhull"),
               ||quickhull::<_,_,List<_>>(t));
    vec_of_list(h, None)
  };

  // Initialize a vector to store (n, naive_time, DCG_time) for each iteration
  let mut runtimes: Vec<(usize, u64, u64)> = vec![];
  
  // Initialize input lists
  let mut dcg_input: List<Point> = List::nil();
  let mut naive_input: List<Point> = List::nil();

  // Generate a vector of points to push to each input
  let mut new_points: Vec<Point> = vec![];
  for _ in 0..10 { new_points.push(gen_point(10)); }

  // Initialize the current engine with an empty DCG instance and push points
  init_dcg();
  assert!(engine_is_dcg()); // Sanity check
  for (ii, pt) in new_points.iter().enumerate() {
    dcg_input = push_point(name_step, ii, pt.clone(), dcg_input);
  }

  // Current engine is naive; save DCG for later, and push naive points
  let mut dcg = init_naive();
  assert!(engine_is_naive()); // Sanity check
  for (ii, pt) in new_points.iter().enumerate() {
    naive_input = push_point(name_step, ii, pt.clone(), naive_input);
  }
  
  let the_end: usize = (max_n - 10) / pts_per_step + 1;
  for ii in 1..the_end {
    let n: usize = (ii * pts_per_step) + 10;
    if n % 100 < pts_per_step {
      println!("Running n = {}", n);
    }

    let limit = 1000; // (n as f64 / 0.01).sqrt() as isize;
    // println!("limit is {}.", limit);

    // Generate more points
    let mut new_points: Vec<Point> = vec![];
    for _ in 0..pts_per_step { new_points.push(gen_point(limit)); }

    assert!(engine_is_naive()); // Sanity check
    // Push the new points to the naive list
    for (ii, pt) in new_points.iter().enumerate() {
      naive_input = push_point(name_step, ii + n, pt.clone(), naive_input);
    }

    let naive_start = time::precise_time_ns();
    let naive_out = doit(naive_input.clone()); // MEASURE ME!
    let naive_end = time::precise_time_ns();

    init_engine(dcg); // Switch to (saved) DCG engine


    assert!(engine_is_dcg()); // Sanity check
    // Push the new points to the DCG list
    for (ii, pt) in new_points.iter().enumerate() {
      dcg_input = push_point(name_step, ii + n, pt.clone(), dcg_input);
    }

    let dcg_start = time::precise_time_ns();
    let dcg_out = doit(dcg_input.clone()); // MEASURE ME!
    let dcg_end = time::precise_time_ns();

    dcg = init_naive(); // Switch back to naive; save DCG engine for later

    // Log the times for this run
    runtimes.push((n, naive_end - naive_start, dcg_end - dcg_start));

    assert_eq!(naive_out, dcg_out); // (A very important) sanity check
  }

  runtimes
}

fn main2(max_n: usize, step_size: usize) {
  let b_start = time::precise_time_ns();

  // Run the testing harness with (max_n, step_size)
  let runtimes = runtime_harness(max_n, step_size);

  let b_end = time::precise_time_ns();

  // Write the runtimes to a csv for plottage.
  let path = format!("qh_runtime_n{}_s{}.csv", max_n, step_size);
  let mut writer = Writer::from_file(path).unwrap();
  for r in runtimes.into_iter() {
    writer.encode(r).ok().expect("CSV writer error");
  }

  // So I know how long my computer was busy, in case I fall asleep while it's running.
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
      -s --stepsize=[step_size]    'points to add between runs' ")
    .get_matches();

  let max_n = value_t!(args.value_of("maxn"), usize).unwrap_or(1000);
  let step_size = value_t!(args.value_of("stepsize"), usize).unwrap_or(1);

  use std::thread;
  //use std::thread::JoinHandle;
  let child =
    thread::Builder::new().stack_size(64 * 1024 * 1024).spawn(move || { main2(max_n, step_size) });
  let _ = child.unwrap().join();
}
