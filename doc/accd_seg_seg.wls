(*
vectors = Fold[{a,b} |-> a && b, Map[v |-> Element[v, Vectors[3, Reals]], {T, dT, U, dU, a, b, c, d}]]
matrices = Fold[{a,b} |-> a && b, Map[m |-> Element[m, Matrices[{3, 3}, Reals]], {A, dA, B, dB}]]
*)

Share[]

(*
CompiledLineDist = Compile[{}, Minimize[{Dot[P1 + E1 t1 - P2 + E2 t2, P1 + E1 t1 - P2 + E2 t2], Element[t1 | t2, Reals] && Element[P1 | P2 | E1 | E2, Vectors[3, Reals]]}, {t1, t2}]]
*)

(*
CompiledLineIntersection = Compile[{}, Solve[{0 == Dot[P1 + E1 t1 - P2 + E2 t2, P1 + E1 t1 - P2 + E2 t2], Element[t1 | t2, Reals] && Element[P1 | P2 | E1 | E2, Vectors[3, Reals]]}, {t1, t2}]]

Print["Compiled Line Intersection"]
intersection = CompiledLineIntersection[]
*)
(*
$Assumptions = {Element[t1 | t2, Reals], Element[P1 | P2 | E1 | E2, Vectors[3, Reals]]}
*)

intersection = TensorReduce[0 == Dot[P1 + E1 t1 - P2 - E2 t2, P1 + E1 t1 - P2 - E2 t2] && Element[t1 | t2, Reals] && Element[P1 | P2 | E1 | E2, Vectors[3, Reals]]]

Print["Between two lines:"]
Print[intersection]
Print[]

Export["line_line_intersection.txt", intersection]
