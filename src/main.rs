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
  monoid_of_tree
    (points, l.u.clone(), Rc::new(
      move |q, p| {
       	let dp = line_point_dist(&l, &p);
       	let dq = line_point_dist(&l, &q);
        if dp > dq { p } else { q }
      }))
}

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
    // TODO name of line l
    let (l_line, l_points, mid, r_line, r_points) = ns(name_of_usize(0), || {
      let mid =
        ns(name_of_str("find-mid"),
           || furthest_point_from_line(l.clone(), points.clone()));
      
      let l_line = Line { u: l.u.clone(), v: mid.clone() };
      let r_line = Line { u: mid.clone(), v: l.v.clone() };
      
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

      (l_line, l_points,
       mid,
       r_line, r_points)
    });

    let (nr, nl) = name_fork(n);
    //let hull = L::cons(Shape::Line(r_line.clone()), hull);
    let hull = ns(nr.clone(), ||(quickhull_rec_2(nr, r_line, r_points, hull)));
    let hull = L::cons(Shape::Point(mid.clone()), hull);
    let hull = L::name(nl.clone(), hull);
    //let hull = L::cons(Shape::Line(l_line.clone()), hull);
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
  // This macro `eager!` memo-izes the call to quickhull_rec.
  // It places the call and its result in an articulation named `n`.
  let (c, _) = eager!(n.clone() =>>
                      quickhull_rec, n:n.clone(), l:l, points:points, hull:hull);
  L::art(c)
}

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

  println!("t_line = {:?}", t_line);
  
  let t_line2 = t_line.clone();
  let t_points =
    ns(name_pair(name_of_str("t-points"), name_of_usize(0)), // TODO name of t_line2
       || filter_tree_of_tree::<_,_,_,T>
       (points.clone(), Box::new(move |p| line_side_test(&t_line2, &p) )));
  
  let b_line2 = b_line.clone();
  let b_points =
    ns(name_pair(name_of_str("b-points"), name_of_usize(0)), // TODO name of b_line2
       || filter_tree_of_tree::<_,_,_,T>
       (points.clone(), Box::new(move |p| line_side_test(&b_line2, &p) )));

  let (nt,nb) = name_fork(name_of_str("rec"));
  
  let hull = L::cons(Shape::Point(most_left), L::nil());
  let hull = L::art(cell(nb.clone(), hull));
  let hull = L::name(nb.clone(), hull);
  let hull = ns(nb.clone(),||quickhull_rec_2(nb, b_line, b_points, hull));

  let hull = L::cons(Shape::Point(most_right), hull);
  let hull = L::art(cell(nt.clone(), hull));
  let hull = L::name(nt.clone(), hull);
  let hull = ns(nt.clone(),||quickhull_rec_2(nt, t_line, t_points, hull));
  hull
}

fn test_input() -> List<Point> {
  list_of_vec::<Point,List<_>>( &vec![
    NameElse::Name(name_of_usize(0)),  NameElse::Else(Point{x: 2,y: 2}), // inside
    NameElse::Name(name_of_usize(1)),  NameElse::Else(Point{x: 2,y:-2}), // inside
    NameElse::Name(name_of_usize(2)),  NameElse::Else(Point{x:-2,y:-2}), // inside
    NameElse::Name(name_of_usize(3)),  NameElse::Else(Point{x:-2,y: 2}), // inside
    
    NameElse::Name(name_of_usize(4)),  NameElse::Else(Point{x: 6,y: 6}), // on hull
    NameElse::Name(name_of_usize(5)),  NameElse::Else(Point{x: 6,y:-6}), // on hull
    NameElse::Name(name_of_usize(6)),  NameElse::Else(Point{x:-6,y:-6}), // on hull
    NameElse::Name(name_of_usize(7)),  NameElse::Else(Point{x:-6,y: 6}), // on hull
    
    NameElse::Name(name_of_usize(8)),  NameElse::Else(Point{x: 5,y: 5}), // inside
    NameElse::Name(name_of_usize(9)),  NameElse::Else(Point{x: 5,y:-5}), // inside
    NameElse::Name(name_of_usize(10)),  NameElse::Else(Point{x:-5,y:-5}), // inside
    NameElse::Name(name_of_usize(11)),  NameElse::Else(Point{x:-5,y: 5}), // inside
    
    NameElse::Name(name_of_usize(12)),  NameElse::Else(Point{x: 4,y: 4}), // inside
    NameElse::Name(name_of_usize(13)),  NameElse::Else(Point{x: 4,y:-4}), // inside
    NameElse::Name(name_of_usize(14)),  NameElse::Else(Point{x:-4,y:-4}), // inside
    NameElse::Name(name_of_usize(15)),  NameElse::Else(Point{x:-4,y: 4}), // inside
    
    NameElse::Name(name_of_usize(16)),  NameElse::Else(Point{x: 3,y: 3}), // inside
    NameElse::Name(name_of_usize(17)),  NameElse::Else(Point{x: 3,y:-3}), // inside
    NameElse::Name(name_of_usize(18)),  NameElse::Else(Point{x:-3,y:-3}), // inside
    NameElse::Name(name_of_usize(19)),  NameElse::Else(Point{x:-3,y: 3}), // inside
    
    NameElse::Name(name_of_usize(20)),  NameElse::Else(Point{x:1,y:1}), // inside
    NameElse::Name(name_of_usize(21)),  NameElse::Else(Point{x:3,y:0}), // inside
    NameElse::Name(name_of_usize(22)), NameElse::Else(Point{x:0,y:3}), // inside
    NameElse::Name(name_of_usize(23)), NameElse::Else(Point{x:5,y:3}), // inside
    NameElse::Name(name_of_usize(24)), NameElse::Else(Point{x:5,y:5}), // inside
    
    NameElse::Name(name_of_usize(25)), NameElse::Else(Point{x:10,y:0}),  // on hull
    NameElse::Name(name_of_usize(26)), NameElse::Else(Point{x:0,y:10}),  // on hull
    NameElse::Name(name_of_usize(27)), NameElse::Else(Point{x:-10,y:0}), // on hull
    NameElse::Name(name_of_usize(28)), NameElse::Else(Point{x:0,y:-10}), // on hull
    
    NameElse::Name(name_of_usize(29)), NameElse::Else(Point{x:10,y:2}),  // on hull
    NameElse::Name(name_of_usize(30)), NameElse::Else(Point{x:2,y:10}),  // on hull
    NameElse::Name(name_of_usize(31)), NameElse::Else(Point{x:-10,y:2}), // on hull
    NameElse::Name(name_of_usize(32)), NameElse::Else(Point{x:2,y:-10}), // on hull
    
    NameElse::Name(name_of_usize(33)),
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

#[test]
pub fn test_qh_inc () {
  
  fn push_point(l:List<Point>) -> List<Point> {
    let l = <List<Point> as ListIntro<Point>>::cons(Point{x: 20,y:20},  l); // new hull
    let l = <List<Point> as ListIntro<Point>>::name(name_of_usize(222), l); // 
    let l = <List<Point> as ListIntro<Point>>::cons(Point{x:-20,y:20},  l); // new hull
    let l = <List<Point> as ListIntro<Point>>::name(name_of_usize(333), l); // 
    let l = <List<Point> as ListIntro<Point>>::cons(Point{x:20,y:-20},  l); // new hull
    let l = <List<Point> as ListIntro<Point>>::name(name_of_usize(444), l); // 
    let l = <List<Point> as ListIntro<Point>>::cons(Point{x:-20,y:-20}, l); // new hull
    let l = <List<Point> as ListIntro<Point>>::name(name_of_usize(555), l); // 
    l
  };
  
  fn doit(l:List<Point>) -> Vec<NameElse<Shape>> {
    //let h = inner(|| {
      let t = ns(name_of_str("tree_of_list"),
                 ||tree_of_list::<_,_,Tree<_>,_>(Dir2::Right, l));
      let h = ns(name_of_str("quickhull"),
                 ||quickhull::<_,_,List<_>>(t.clone()));
    //})
    let o = vec_of_list(h, None);
    println!("hull = {:?}\n", &o);
    o
  };

  println!("Naive run");
  init_naive();
  let inp      = test_input();
  let naive_o1 = doit(inp.clone());
  let inp      = push_point(inp);
  let naive_o2 = doit(inp);

  println!("DCG run");
  init_dcg();
  let inp    = test_input();
  let dcg_o1 = doit(inp.clone());
  let inp    = push_point(inp);
  let dcg_o2 = doit(inp);

  assert_eq!(naive_o1, dcg_o1);
  assert_eq!(naive_o2, dcg_o2);  
}


fn main() {
}
