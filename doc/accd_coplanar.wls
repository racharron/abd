Coplanar[x1_, x2_, x3_, x4_] := 0 == Dot[Cross[x2-x1, x4-x1], x3-x1]

$Assumptions = Element[x1 | dx1 | x2 | dx2 | x3 | dx3 | x4 | dx4, Vectors[3, Reals]] && Element[tau, Reals]

intersection = Solve[ Coplanar[dx1 tau + x1, dx2 tau + x2, dx3 tau + x3, dx4 tau + x4] && $Assumptions, tau]

Print[intersection]

Export["accd_coplanar.txt", intersection]
